"""Bridge between MuJoCo state and downstream control paths.

Responsibilities:
1) Handle SITL UDP socket (ArduPilot JSON payload + servo packets).
2) Optional ROS2 transport:
   - `/cmd_vel` and `/mavros/rc/override` input
   - IMU/DVL publish
   - DVL odometry
   - Stereo image publish
"""

import json
import os
import socket
import struct
import time
from array import array
from pathlib import Path
from typing import Callable, Optional

import mujoco
import numpy as np


class Ros2Bridge:
    """Runtime adapter between MuJoCo state and ROS2 topics."""

    def __init__(
        self,
        model: mujoco.MjModel,
        command_callback: Callable[[float, float, float, float], None],
        cmd_limit: float = 15.0,
        publish_images: bool = False,
        image_width: int = 640,
        image_height: int = 360,
        sensor_hz: float = 50.0,
        image_hz: float = 10.0,
        enable_mavros: bool = True,
        enable_sitl: bool = False,
        sitl_ip: str = "127.0.0.1",
        sitl_port: int = 9002,
        sitl_send_port: int = 9003,
        sitl_ch_forward: int = 1,
        sitl_ch_lateral: int = 0,
        sitl_ch_throttle: int = 2,
        sitl_ch_yaw: int = 3,
        sitl_forward_sign: float = 1.0,
        sitl_lateral_sign: float = 1.0,
        sitl_yaw_sign: float = 1.0,
        sitl_heave_sign: float = 1.0,
        sitl_cmd_scale: float = 1.0,
        sitl_command_debug: bool = False,
        sitl_servo_callback: Optional[Callable[[list[int]], None]] = None,
        sitl_axis_control: bool = True,
        sitl_servo_source: str = "json",
        sitl_mavlink_endpoint: str = "",
        sitl_mavlink_servo_hz: float = 20.0,
        sitl_mavlink_target_sysid: int = 0,
        sitl_mavlink_target_compid: int = 0,
        sitl_mavlink_source_sysid: int = 255,
        sitl_mavlink_source_compid: int = 190,
        camera_calib_left: str = "",
        camera_calib_right: str = "",
        enable_ros: bool = True,
    ) -> None:
        self._enable_ros = bool(enable_ros)
        self.has_mavros = False
        self.OverrideRCIn = None

        if self._enable_ros:
            # ROS2 imports are runtime-optional so the simulator can run
            # without ROS2 when only SITL UDP is needed.
            try:
                import rclpy
                from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
                from nav_msgs.msg import Odometry
                from rclpy.node import Node
                from sensor_msgs.msg import CameraInfo, Image, Imu, Range
                from std_msgs.msg import Float32, Header
            except ImportError as exc:
                raise RuntimeError(
                    "ROS2 packages not found. Install rclpy + sensor_msgs + geometry_msgs + nav_msgs."
                ) from exc

            # Try to import mavros_msgs for MAVROS compatibility
            try:
                from mavros_msgs.msg import OverrideRCIn
                self.OverrideRCIn = OverrideRCIn
                self.has_mavros = True
            except ImportError:
                if enable_mavros:
                    print("[ros2_bridge] mavros_msgs not found, MAVROS input disabled", flush=True)

            self.rclpy = rclpy
            self.Twist = Twist
            self.TwistStamped = TwistStamped
            self.PoseStamped = PoseStamped
            self.Odometry = Odometry
            self.CameraInfo = CameraInfo
            self.Image = Image
            self.Imu = Imu
            self.Range = Range
            self.Header = Header
            self.Float32 = Float32
            if not self.rclpy.ok():
                self.rclpy.init(args=None)
            self.node = Node("uuv_mujoco_bridge")
            # Keep SITL running even if ROS2 context becomes invalid at runtime.
            self._ros_ok = True
            self._ros_error_reported = False
        else:
            self.rclpy = None
            self.Twist = None
            self.TwistStamped = None
            self.PoseStamped = None
            self.Odometry = None
            self.CameraInfo = None
            self.Image = None
            self.Imu = None
            self.Range = None
            self.Header = None
            self.Float32 = None
            self.node = None
            self._ros_ok = False
            self._ros_error_reported = False

        self.command_callback = command_callback
        self.cmd_limit = float(cmd_limit)
        self.cmd_timeout_s = float(
            np.clip(
                self._env_to_float("ROS2_UUV_CMD_TIMEOUT_S", 0.45),
                0.1,
                2.0,
            )
        )
        self.last_cmd_wall = -1.0
        self.cmd_active = False
        self.enable_mavros = enable_mavros
        self._cmd_filter_norm = np.zeros(4, dtype=np.float64)
        self._cmd_filter_t = -1.0
        self._cmd_deadband_norm = float(
            np.clip(
                self._env_to_float("ROS2_UUV_CMD_DEADBAND", 0.015),
                0.0,
                0.2,
            )
        )
        self._cmd_slew_rate_norm = float(
            np.clip(
                self._env_to_float("ROS2_UUV_CMD_SLEW_RATE", 24.0),
                0.0,
                200.0,
            )
        )

        self.sensor_dt = 1.0 / max(float(sensor_hz), 1e-6)
        self.image_dt = 1.0 / max(float(image_hz), 1e-6)
        self.next_sensor_t = 0.0
        self.next_image_t = 0.0
        self.last_pub_t = -1.0
        
        # SITL State & Connection
        self.enable_sitl = enable_sitl
        self.sitl_sock = None
        self.sitl_addr = (sitl_ip, int(sitl_port))
        self.sitl_send_addr = (sitl_ip, int(sitl_send_port))
        self.sitl_listen_addr = ("0.0.0.0", int(sitl_port))
        self._sitl_client_addr = None
        self._sitl_send_target = None
        self._sitl_client_last_wall = -1.0
        self._sitl_client_logged = False
        self._sitl_no_client_warn_interval_s = 3.0
        self._sitl_last_client_missing_wall = -1.0
        self._sitl_last_command_stale_wall = -1.0
        self._sitl_last_command_hz_log = time.monotonic()
        self._sitl_last_send_wall = -1.0
        self._sitl_last_send_err_wall = -1.0
        self._sitl_last_no_client_wall = -1.0
        self._sitl_last_sensor_log_wall = -1.0
        self._sitl_send_counter = 0
        self._sitl_first_servo_wall = -1.0
        self._sitl_last_nonneutral_servo_wall = -1.0
        self._sitl_last_neutral_warn_wall = -1.0
        self._sitl_last_nonzero_cmd = None
        self._sitl_nonfinite_warned = False
        self._sitl_t0_wall = None
        self._sitl_prev_sim_t = None
        self._sitl_prev_pos_enu = None
        self._sitl_vel_lpf_enu = np.zeros(3, dtype=np.float64)
        self._sitl_cmd_debug = bool(sitl_command_debug)
        self._sitl_servo_callback = sitl_servo_callback
        self._sitl_axis_control = bool(sitl_axis_control)
        self._sitl_last_cmd_log = -1.0
        self._sitl_last_cmd_nonzero = None
        self._sitl_last_servo_pkt = None
        self._sitl_cmd_scale = max(0.0, float(sitl_cmd_scale))
        self._sitl_servo_source = str(sitl_servo_source or "json").strip().lower()
        if self._sitl_servo_source not in ("json", "mavlink"):
            self._sitl_servo_source = "json"
        self._sitl_mavlink_endpoint = str(sitl_mavlink_endpoint or "").strip()
        requested_servo_hz = float(sitl_mavlink_servo_hz)
        self._sitl_mavlink_servo_hz = float(np.clip(requested_servo_hz, 1.0, 20.0))
        if requested_servo_hz > self._sitl_mavlink_servo_hz + 1e-6:
            print(
                "[ros2_bridge] sitl_mavlink_servo_hz clamped to "
                f"{self._sitl_mavlink_servo_hz:.1f}Hz (requested {requested_servo_hz:.1f}Hz) "
                "to avoid ArduSub message-rate overrun.",
                flush=True,
            )
        self._sitl_mavlink_target_sysid = int(sitl_mavlink_target_sysid)
        self._sitl_mavlink_target_compid = int(sitl_mavlink_target_compid)
        self._sitl_mavlink_source_system = int(sitl_mavlink_source_sysid)
        self._sitl_mavlink_source_component = int(sitl_mavlink_source_compid)
        self._sitl_mavlink_target_sysid = max(0, min(255, self._sitl_mavlink_target_sysid))
        self._sitl_mavlink_target_compid = max(0, min(255, self._sitl_mavlink_target_compid))
        self._sitl_mavlink_source_system = max(1, min(255, self._sitl_mavlink_source_system or 255))
        self._sitl_mavlink_source_component = max(1, min(255, self._sitl_mavlink_source_component or 190))
        self._sitl_mav = None
        self._sitl_mavutil = None
        self._sitl_mav_hb = None
        self._sitl_mav_last_hb_wall = -1.0
        self._sitl_mav_last_msg_wall = -1.0
        self._sitl_mav_last_req_wall = -1.0
        self._sitl_mav_last_wait_warn_wall = -1.0
        self._sitl_mav_target_mismatch_warn_wall = -1.0
        self._sitl_mav_wait_warn_interval_s = 3.0
        self._sitl_servo_active_source = self._sitl_servo_source
        self._sitl_source_switch_log_wall = -1.0
        self._sitl_mavlink_timeout_s = float(
            np.clip(
                self._env_to_float("ROS2_UUV_SITL_MAVLINK_TIMEOUT_S", 1.5),
                0.1,
                5.0,
            )
        )
        self._sitl_cmd_sign = np.array(
            [
                float(sitl_forward_sign),
                float(sitl_lateral_sign),
                float(sitl_yaw_sign),
                float(sitl_heave_sign),
            ],
            dtype=np.float64,
        )
        self._sitl_cmd_sign = np.where(np.isfinite(self._sitl_cmd_sign), self._sitl_cmd_sign, 1.0)
        self._sitl_cmd_sign[np.isclose(self._sitl_cmd_sign, 0.0)] = 1.0
        # Frame conversion:
        # - This MuJoCo project uses world axes x=forward, y=left, z=up.
        # - ArduPilot JSON expects world NED: x=north, y=east(right), z=down.
        #   Therefore world->NED is a proper 180deg rotation about +x.
        self._enu_to_ned = np.diag([1.0, -1.0, -1.0])
        # MuJoCo base body axes in this model are [x=fwd, y=down, z=left] (FDL).
        # ArduPilot expects FRD [x=fwd, y=right, z=down].
        self._bmj_to_frd = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        )
        # ROS base_link convention is FLU [x=fwd, y=left, z=up].
        self._bmj_to_flu = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, -1.0, 0.0],
            ],
            dtype=np.float64,
        )
        if self.enable_sitl:
            if int(sitl_send_port) <= 0:
                raise ValueError("sitl-send-port must be a positive integer")
            self._connect_sitl()
            if self._sitl_servo_source == "mavlink":
                self._connect_sitl_mavlink()

        # DVL odometry integration state
        self._odom_pos = np.array([0.0, 0.0, 0.0])
        self._odom_yaw = 0.0
        self._last_odom_time = -1.0

        # Core sensor publishers / ROS2 control channels (optional).
        self.pub_imu = None
        self.pub_dvl_vel = None
        self.pub_dvl_vel_raw = None
        self.pub_dvl_alt = None
        self.pub_depth = None
        self.pub_bar30_pressure = None
        self.pub_dvl_odom = None
        self.pub_ground_truth = None
        self.sub_cmd = None
        self.sub_mavros_rc = None
        if self._enable_ros:
            self.pub_imu = self.node.create_publisher(self.Imu, "/imu/data", 1)
            self.pub_dvl_vel = self.node.create_publisher(self.TwistStamped, "/dvl/velocity", 1)
            self.pub_dvl_vel_raw = self.node.create_publisher(self.TwistStamped, "/dvl/velocity_raw", 1)
            self.pub_dvl_alt = self.node.create_publisher(self.Range, "/dvl/altitude", 1)
            self.pub_depth = self.node.create_publisher(self.Float32, "/depth", 1)
            self.pub_bar30_pressure = self.node.create_publisher(self.Float32, "/bar30/pressure_pa", 1)
            self.pub_dvl_odom = self.node.create_publisher(self.Odometry, "/dvl/odometry", 1)
            self.pub_ground_truth = self.node.create_publisher(self.PoseStamped, "/mujoco/ground_truth/pose", 1)

            # Command subscribers
            self.sub_cmd = self.node.create_subscription(self.Twist, "/cmd_vel", self._on_cmd_vel, 1)

            # MAVROS RC Override subscriber (for ArduSub/Pixhawk compatibility)
            if self.has_mavros and self.enable_mavros:
                self.sub_mavros_rc = self.node.create_subscription(
                    self.OverrideRCIn, "/mavros/rc/override", self._on_mavros_rc_override, 1
                )

        # PWM parameters for MAVROS
        self._pwm_min = 1100
        self._pwm_max = 1900
        self._pwm_neutral = 1500
        # MAVLink RC input order is Roll(1), Pitch(2), Throttle(3), Yaw(4).
        # Zero-based indices are used below.
        self._ch_forward = self._env_to_int("ROS2_UUV_RC_CH_FORWARD", int(sitl_ch_forward))
        self._ch_lateral = self._env_to_int("ROS2_UUV_RC_CH_LATERAL", int(sitl_ch_lateral))
        self._ch_throttle = self._env_to_int("ROS2_UUV_RC_CH_THROTTLE", int(sitl_ch_throttle))
        self._ch_yaw = self._env_to_int("ROS2_UUV_RC_CH_YAW", int(sitl_ch_yaw))
        # Optional SITL environment overrides for command sign/scale (kept compatible
        # with CLI args above for easier calibration while debugging).
        self._sitl_cmd_sign = np.array(
            [
                self._env_to_float("ROS2_UUV_SITL_FORWARD_SIGN", float(sitl_forward_sign)),
                self._env_to_float("ROS2_UUV_SITL_LATERAL_SIGN", float(sitl_lateral_sign)),
                self._env_to_float("ROS2_UUV_SITL_YAW_SIGN", float(sitl_yaw_sign)),
                self._env_to_float("ROS2_UUV_SITL_HEAVE_SIGN", float(sitl_heave_sign)),
            ],
            dtype=np.float64,
        )
        self._sitl_cmd_sign = np.where(np.isfinite(self._sitl_cmd_sign), self._sitl_cmd_sign, 1.0)
        self._sitl_cmd_sign[np.isclose(self._sitl_cmd_sign, 0.0)] = 1.0
        sitl_cmd_scale_env = self._env_to_float(
            "ROS2_UUV_SITL_CMD_SCALE", float(self._sitl_cmd_scale)
        )
        self._sitl_cmd_scale = float(np.clip(sitl_cmd_scale_env, 0.0, 2.0))
        if self._enable_ros:
            self.node.get_logger().info(
                f"Using MAVLink RC override channel map: "
                f"forward={self._ch_forward}, lateral={self._ch_lateral}, "
                f"throttle={self._ch_throttle}, yaw={self._ch_yaw}, "
                f"SITL_sign={self._sitl_cmd_sign.tolist()}, SITL_scale={self._sitl_cmd_scale:.3f}"
            )

        self.model = model
        self.sensor_ids = {}
        for sname in ("imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude", "base_pos"):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
            if sid >= 0:
                self.sensor_ids[sname] = sid

        # Get base body ID for ground truth
        self._base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._imu_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
        self._dvl_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "dvl_site")
        self._bar30_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "bar30_site")
        if self._bar30_site_id < 0:
            self._bar30_site_id = self._imu_site_id
        self._bar30_surface_pressure_pa = float(
            np.clip(self._env_to_float("ROS2_UUV_BAR30_SURFACE_PRESSURE_PA", 101325.0), 80000.0, 120000.0)
        )
        self._bar30_water_density = float(
            np.clip(self._env_to_float("ROS2_UUV_BAR30_WATER_DENSITY", 1025.0), 900.0, 1200.0)
        )
        self._bar30_gravity = float(
            np.clip(self._env_to_float("ROS2_UUV_BAR30_GRAVITY", 9.80665), 9.5, 10.0)
        )
        self._bar30_noise_pa_std = float(
            np.clip(self._env_to_float("ROS2_UUV_BAR30_NOISE_PA_STD", 20.0), 0.0, 2000.0)
        )
        bar30_lpf_hz = float(
            np.clip(self._env_to_float("ROS2_UUV_BAR30_LPF_HZ", 20.0), 0.1, 200.0)
        )
        self._bar30_lpf_tau_s = float(1.0 / (2.0 * np.pi * bar30_lpf_hz))
        self._bar30_pressure_pa_filt = None
        self._bar30_depth_prev_m = None
        self._bar30_depth_prev_t = None
        self._dvl_filter_alpha = float(
            np.clip(self._env_to_float("ROS2_UUV_DVL_LPF_ALPHA", 0.35), 0.0, 1.0)
        )
        self._dvl_vel_body_filt = None

        self.publish_images = bool(publish_images)
        self.renderers = {}
        self.cam_ids = {}
        self.image_pubs = {}
        self.info_pubs = {}
        self.cam_info = {}
        self._camera_calib_path = {
            "stereo_left": str(camera_calib_left or ""),
            "stereo_right": str(camera_calib_right or ""),
        }

        if self.publish_images and self._enable_ros:
            for cname in ("stereo_left", "stereo_right"):
                cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cname)
                if cid < 0:
                    continue
                self.cam_ids[cname] = cid
                self.renderers[cname] = mujoco.Renderer(
                    self.model,
                    height=int(image_height),
                    width=int(image_width),
                )
                topic_ns = f"/stereo/{'left' if cname.endswith('left') else 'right'}"
                self.image_pubs[cname] = self.node.create_publisher(self.Image, f"{topic_ns}/image_raw", 1)
                self.info_pubs[cname] = self.node.create_publisher(
                    self.CameraInfo, f"{topic_ns}/camera_info", 1
                )
                default_info = self._build_camera_info(cname, int(image_width), int(image_height))
                calib_path = self._camera_calib_path.get(cname, "")
                loaded_info = self._load_camera_info_yaml(
                    calib_path,
                    default_info,
                    fallback_frame_id=f"{cname}_optical",
                )
                if loaded_info is not None:
                    self.cam_info[cname] = loaded_info
                    print(f"[ros2_bridge] loaded camera calibration: {cname} <- {calib_path}", flush=True)
                else:
                    self.cam_info[cname] = default_info

        if self._enable_ros:
            topics_info = "/cmd_vel -> control, /imu/data, /dvl/velocity, /dvl/velocity_raw, /dvl/odometry, /dvl/altitude, /depth, /bar30/pressure_pa"
            if self.has_mavros and self.enable_mavros:
                topics_info += ", /mavros/rc/override"
            if self.publish_images:
                topics_info += ", /stereo/*"
            if self.enable_sitl:
                topics_info += (
                    f", SITL-json({self.sitl_addr[0]}:{self.sitl_addr[1]}->{self.sitl_send_addr[1]})"
                )
                if self._sitl_servo_source == "mavlink":
                    topics_info += (
                        f", SITL-servo-mavlink({self._sitl_mavlink_endpoint}, "
                        f"target={self._sitl_mavlink_target_sysid}/{self._sitl_mavlink_target_compid}, "
                        f"source={self._sitl_mavlink_source_system}/{self._sitl_mavlink_source_component})"
                    )
                else:
                    topics_info += ", SITL-servo-json"
            topics_info += (
                f", cmd_deadband={self._cmd_deadband_norm:.3f}, "
                f"cmd_slew={self._cmd_slew_rate_norm:.1f}/s, "
                f"cmd_timeout={self.cmd_timeout_s:.2f}s"
            )
            self.node.get_logger().info(f"ROS2 bridge active: {topics_info}")

    @staticmethod
    def _finite_or_zero(value: float) -> float:
        return float(value) if np.isfinite(value) else 0.0

    @staticmethod
    def _depth_from_pos_ned(pos_ned: np.ndarray) -> float:
        """Convert NED position to depth in meters (surface=0, down positive)."""
        if pos_ned is None or len(pos_ned) < 3:
            return 0.0
        return float(max(0.0, float(pos_ned[2])))

    @staticmethod
    def _pressure_abs_from_depth_m(depth_m: float, surface_pressure_pa: float, rho: float, gravity: float) -> float:
        depth = float(max(0.0, depth_m))
        return float(surface_pressure_pa + rho * gravity * depth)

    @staticmethod
    def _depth_from_pressure_abs_pa(pressure_abs_pa: float, surface_pressure_pa: float, rho: float, gravity: float) -> float:
        denom = float(max(1.0e-6, rho * gravity))
        return float(max(0.0, (float(pressure_abs_pa) - float(surface_pressure_pa)) / denom))

    def _bar30_depth_pressure_from_model(self, data) -> tuple[float | None, float | None]:
        """Simulate BAR30 absolute pressure and convert it back to depth."""
        z_world = None
        if self._bar30_site_id is not None and int(self._bar30_site_id) >= 0:
            z_world = float(data.site_xpos[int(self._bar30_site_id)][2])
        elif self._base_id >= 0:
            z_world = float(data.xpos[self._base_id][2])
        if z_world is None or not np.isfinite(z_world):
            return None, None

        raw_depth_m = float(max(0.0, -z_world))
        pressure_pa = self._pressure_abs_from_depth_m(
            raw_depth_m,
            self._bar30_surface_pressure_pa,
            self._bar30_water_density,
            self._bar30_gravity,
        )
        if self._bar30_noise_pa_std > 0.0:
            pressure_pa += float(np.random.normal(0.0, self._bar30_noise_pa_std))
        pressure_pa = float(max(1000.0, pressure_pa))

        if self._bar30_pressure_pa_filt is None or not np.isfinite(float(self._bar30_pressure_pa_filt)):
            self._bar30_pressure_pa_filt = pressure_pa
        else:
            dt = float(max(1.0e-4, self.sensor_dt))
            alpha = float(np.clip(dt / (self._bar30_lpf_tau_s + dt), 0.0, 1.0))
            self._bar30_pressure_pa_filt += alpha * (pressure_pa - self._bar30_pressure_pa_filt)

        depth_m = self._depth_from_pressure_abs_pa(
            float(self._bar30_pressure_pa_filt),
            self._bar30_surface_pressure_pa,
            self._bar30_water_density,
            self._bar30_gravity,
        )
        return depth_m, float(self._bar30_pressure_pa_filt)

    def _apply_cmd_deadband(self, value: float) -> float:
        value = self._finite_or_zero(value)
        if abs(value) <= self._cmd_deadband_norm:
            return 0.0
        return float(np.clip(value, -1.0, 1.0))

    @staticmethod
    def _env_to_int(env_name: str, default: int) -> int:
        value = os.getenv(env_name)
        if not value:
            return default
        try:
            parsed = int(value)
        except ValueError:
            print(f"[ros2_bridge] invalid {env_name}={value!r}, using {default}", flush=True)
            return default
        return max(0, parsed)

    @staticmethod
    def _env_to_float(env_name: str, default: float) -> float:
        value = os.getenv(env_name)
        if not value:
            return float(default)
        try:
            parsed = float(value)
        except ValueError:
            print(f"[ros2_bridge] invalid {env_name}={value!r}, using {default}", flush=True)
            return float(default)
        return float(parsed)

    def _sitl_map_and_scale(self, fwd: float, sway: float, yaw: float, heave: float) -> tuple[float, float, float, float]:
        """Apply SITL axis sign and gain settings."""
        vec = np.array([fwd, sway, yaw, heave], dtype=np.float64)
        if not np.all(np.isfinite(vec)):
            vec = np.nan_to_num(vec, nan=0.0, posinf=1.0, neginf=-1.0)
        mapped = vec * self._sitl_cmd_sign * self._sitl_cmd_scale
        mapped = np.clip(mapped, -1.0, 1.0)
        return (
            float(mapped[0]),
            float(mapped[1]),
            float(mapped[2]),
            float(mapped[3]),
        )

    def set_sitl_servo_handler(
        self,
        callback: Optional[Callable[[list[int]], None]],
        axis_control: bool = True,
    ) -> None:
        """Set raw SITL servo callback and whether axis remapping is enabled."""
        self._sitl_servo_callback = callback
        self._sitl_axis_control = bool(axis_control)
        mode = "axis+servo" if self._sitl_axis_control else "servo-direct"
        print(f"[ros2_bridge] SITL control mode set: {mode}", flush=True)

    def _handle_normalized_cmd(
        self,
        fwd_norm: float,
        sway_norm: float,
        yaw_norm: float,
        heave_norm: float,
    ) -> None:
        """Apply deadband and slew limits before passing commands to simulator."""
        now = time.monotonic()
        raw = np.array(
            [
                self._apply_cmd_deadband(fwd_norm),
                self._apply_cmd_deadband(sway_norm),
                self._apply_cmd_deadband(yaw_norm),
                self._apply_cmd_deadband(heave_norm),
            ],
            dtype=np.float64,
        )

        if self._cmd_filter_t < 0.0:
            self._cmd_filter_norm = raw
        else:
            dt = max(0.0, now - self._cmd_filter_t)
            if dt <= 1e-9:
                self._cmd_filter_norm = raw
            else:
                delta = raw - self._cmd_filter_norm
                max_delta = self._cmd_slew_rate_norm * dt
                self._cmd_filter_norm = self._cmd_filter_norm + np.clip(
                    delta, -max_delta, max_delta
                )

        self._cmd_filter_t = now
        self.last_cmd_wall = now
        self.cmd_active = True
        self.command_callback(
            float(self._cmd_filter_norm[0] * self.cmd_limit),
            float(self._cmd_filter_norm[1] * self.cmd_limit),
            float(self._cmd_filter_norm[2] * self.cmd_limit),
            float(self._cmd_filter_norm[3] * self.cmd_limit),
        )

    def _connect_sitl(self) -> None:
        """Initialize UDP socket for ArduPilot SITL JSON interface.

        ArduPilot JSON SITL sends servo packets to `sitl_port` from a dynamic
        source port. Sensor JSON must be sent back to that source port.
        """
        try:
            self.sitl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sitl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sitl_rcvbuf = 1024 * 1024
            sitl_sndbuf = 1024 * 1024
            try:
                env_sitl_rcvbuf = int(os.getenv("ROS2_UUV_SITL_RCVBUF", str(sitl_rcvbuf)))
                if env_sitl_rcvbuf > 0:
                    sitl_rcvbuf = env_sitl_rcvbuf
            except ValueError:
                pass
            try:
                env_sitl_sndbuf = int(os.getenv("ROS2_UUV_SITL_SNDBUF", str(sitl_sndbuf)))
                if env_sitl_sndbuf > 0:
                    sitl_sndbuf = env_sitl_sndbuf
            except ValueError:
                pass
            self.sitl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, sitl_rcvbuf)
            self.sitl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, sitl_sndbuf)
            self.sitl_sock.bind(self.sitl_listen_addr)
            self.sitl_sock.setblocking(False)
            sock_addr = self.sitl_sock.getsockname()
            print(
                f"[ros2_bridge] SITL socket initialized (listen {sock_addr}, servo_target={self.sitl_addr}, sensor_target={self.sitl_send_addr})",
                flush=True,
            )
        except Exception as e:
            print(f"[ros2_bridge] Failed to init SITL socket: {e}", flush=True)

    def _connect_sitl_mavlink(self) -> None:
        """Initialize MAVLink input channel for SITL servo outputs."""
        try:
            from pymavlink import mavutil
        except Exception as exc:
            raise RuntimeError(
                "pymavlink is required for --sitl-servo-source mavlink"
            ) from exc

        endpoint = self._sitl_mavlink_endpoint
        if not endpoint:
            endpoint = f"udpin:0.0.0.0:{int(os.getenv('ROS2_UUV_SITL_MAV_PORT', '14660'))}"
            self._sitl_mavlink_endpoint = endpoint
        self._sitl_mav = mavutil.mavlink_connection(
            endpoint,
            source_system=self._sitl_mavlink_source_system,
            source_component=self._sitl_mavlink_source_component,
            autoreconnect=True,
        )
        self._sitl_mavutil = mavutil
        print(
            f"[ros2_bridge] SITL MAVLink servo input enabled: endpoint={endpoint}",
            flush=True,
        )

    def _request_sitl_mavlink_servo_stream(self) -> None:
        """Request SERVO_OUTPUT_RAW stream from ArduPilot over MAVLink."""
        if self._sitl_mav is None:
            return
        if self._sitl_mav_hb is None and (
            self._sitl_mavlink_target_sysid <= 0 or self._sitl_mavlink_target_compid <= 0
        ):
            return
        if self._sitl_mavutil is None:
            return
        now_wall = time.monotonic()
        stream_fresh = (
            self._sitl_mav_last_msg_wall > 0.0
            and (now_wall - self._sitl_mav_last_msg_wall)
            <= max(2.0, 4.0 / max(self._sitl_mavlink_servo_hz, 1.0))
        )
        req_period_s = 12.0 if stream_fresh else 2.0
        if now_wall - self._sitl_mav_last_req_wall < req_period_s:
            return
        self._sitl_mav_last_req_wall = now_wall
        if self._sitl_mavlink_target_sysid > 0 and self._sitl_mavlink_target_compid > 0:
            target_sys = int(self._sitl_mavlink_target_sysid)
            target_comp = int(self._sitl_mavlink_target_compid)
        elif self._sitl_mav_hb is not None:
            target_sys = int(self._sitl_mav_hb.get_srcSystem())
            target_comp = int(self._sitl_mav_hb.get_srcComponent())
        else:
            return
        interval_us = float(max(1.0, 1.0e6 / self._sitl_mavlink_servo_hz))
        try:
            mavlink_defs = self._sitl_mavutil.mavlink
            self._sitl_mav.mav.command_long_send(
                target_sys,
                target_comp,
                mavlink_defs.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavlink_defs.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW),
                interval_us,
                0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    def _handle_sitl_pwm_values(self, pwm_values: list[int], now_wall: float, source: str) -> None:
        """Common handler for SITL PWM vectors from JSON or MAVLink sources."""
        pwm_head = pwm_values[:8]
        nonneutral = False
        valid_pwm = []
        for value in pwm_head:
            iv = int(value)
            if iv <= 0 or iv == 65535:
                continue
            valid_pwm.append(iv)
            if abs(iv - 1500) > 12:
                nonneutral = True
                break
        if nonneutral:
            self._sitl_last_nonneutral_servo_wall = now_wall
        else:
            since_nonneutral = (
                now_wall - self._sitl_last_nonneutral_servo_wall
                if self._sitl_last_nonneutral_servo_wall > 0.0
                else now_wall - self._sitl_first_servo_wall
            )
            if since_nonneutral > 3.0 and now_wall - self._sitl_last_neutral_warn_wall > 3.0:
                if not valid_pwm:
                    print(
                        f"[ros2_bridge] SITL({source}) servo stream has no active outputs (all 0/65535). "
                        "Check: vehicle ARM state and JSON sensor stream health.",
                        flush=True,
                    )
                else:
                    print(
                        f"[ros2_bridge] SITL({source}) servo stream is neutral (all near 1500). "
                        "Check: vehicle ARM state, QGC joystick enabled, MANUAL mode.",
                        flush=True,
                    )
                self._sitl_last_neutral_warn_wall = now_wall

        if self._sitl_servo_callback is not None:
            try:
                self._sitl_servo_callback(pwm_values)
            except Exception as exc:
                print(f"[ros2_bridge] SITL servo callback failed: {exc}", flush=True)

        if not self._sitl_axis_control:
            if self._sitl_cmd_debug:
                now = time.monotonic()
                pkt8 = tuple(int(v) for v in pwm_values[:8])
                if (self._sitl_last_servo_pkt != pkt8) and (now - self._sitl_last_cmd_log > 0.15):
                    print(f"[ros2_bridge] SITL({source}) servo pwm[1..8]={pkt8}", flush=True)
                    self._sitl_last_cmd_log = now
                    self._sitl_last_servo_pkt = pkt8
            return

        if len(pwm_values) <= max(self._ch_forward, self._ch_lateral, self._ch_throttle, self._ch_yaw):
            return

        fwd = self._pwm_to_normalized(int(pwm_values[self._ch_forward]))
        sway = self._pwm_to_normalized(int(pwm_values[self._ch_lateral]))
        heave = self._pwm_to_normalized(int(pwm_values[self._ch_throttle]))
        yaw = self._pwm_to_normalized(int(pwm_values[self._ch_yaw]))
        raw_fwd, raw_sway, raw_yaw, raw_heave = fwd, sway, yaw, heave
        fwd, sway, yaw, heave = self._sitl_map_and_scale(fwd, sway, yaw, heave)
        if self._sitl_cmd_debug and (self._sitl_last_cmd_nonzero != (fwd, sway, yaw, heave)):
            now = time.monotonic()
            if now - self._sitl_last_cmd_log > 0.15:
                print(
                    f"[ros2_bridge] SITL({source}) raw->mapped pwm: "
                    f"raw({pwm_values[self._ch_forward]},{pwm_values[self._ch_lateral]},{pwm_values[self._ch_throttle]},{pwm_values[self._ch_yaw]}) => "
                    f"norm({raw_fwd:+.3f},{raw_sway:+.3f},{raw_yaw:+.3f},{raw_heave:+.3f}) -> "
                    f"mapped({fwd:+.3f},{sway:+.3f},{yaw:+.3f},{heave:+.3f})",
                    flush=True,
                )
                self._sitl_last_cmd_log = now
                self._sitl_last_cmd_nonzero = (fwd, sway, yaw, heave)
        nonzero = abs(fwd) > 0.01 or abs(sway) > 0.01 or abs(heave) > 0.01 or abs(yaw) > 0.01
        if nonzero:
            now = time.monotonic()
            if self._sitl_last_nonzero_cmd is None or now - self._sitl_last_command_hz_log > 2.0:
                print(
                    "[ros2_bridge] SITL servo input: "
                    f"ch[{self._ch_forward},{self._ch_lateral},{self._ch_throttle},{self._ch_yaw}]="
                    f"{fwd:+.3f},{sway:+.3f},{heave:+.3f},{yaw:+.3f}",
                    flush=True,
                )
                self._sitl_last_command_hz_log = now
                self._sitl_last_nonzero_cmd = (fwd, sway, yaw, heave)
        self._handle_normalized_cmd(fwd, sway, yaw, heave)

    def _poll_sitl_servo_mavlink(self) -> None:
        """Poll SITL servo PWM from MAVLink SERVO_OUTPUT_RAW."""
        if self._sitl_mav is None:
            return
        now_wall = time.monotonic()
        got_any = False
        while True:
            try:
                msg = self._sitl_mav.recv_match(
                    type=["HEARTBEAT", "SERVO_OUTPUT_RAW"],
                    blocking=False,
                )
            except Exception:
                break
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                src_sys = int(msg.get_srcSystem())
                src_comp = int(msg.get_srcComponent())
                try:
                    ap = int(getattr(msg, "autopilot", -1))
                except Exception:
                    ap = -1
                autopilot_mega = -1
                if self._sitl_mavutil is not None:
                    try:
                        autopilot_mega = int(self._sitl_mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
                    except Exception:
                        autopilot_mega = -1
                target_sys = self._sitl_mavlink_target_sysid
                target_comp = self._sitl_mavlink_target_compid
                target_must_match = target_sys > 0 or target_comp > 0
                if target_sys > 0 and src_sys != target_sys:
                    if now_wall - self._sitl_mav_target_mismatch_warn_wall >= 2.0:
                        print(
                            f"[ros2_bridge] Ignoring HEARTBEAT from src-system={src_sys}, src-comp={src_comp}; "
                            f"expecting sys={target_sys}, comp={target_comp}.",
                            flush=True,
                        )
                        self._sitl_mav_target_mismatch_warn_wall = now_wall
                    continue
                if target_comp > 0 and src_comp != target_comp:
                    if now_wall - self._sitl_mav_target_mismatch_warn_wall >= 2.0:
                        print(
                            f"[ros2_bridge] Ignoring HEARTBEAT from src-system={src_sys}, src-comp={src_comp}; "
                            f"expecting sys={target_sys}, comp={target_comp}.",
                            flush=True,
                        )
                        self._sitl_mav_target_mismatch_warn_wall = now_wall
                    continue
                if (not target_must_match) and (ap != autopilot_mega):
                    continue
                if target_must_match or ap == autopilot_mega:
                    self._sitl_mav_hb = msg
                    self._sitl_mav_last_hb_wall = now_wall
                    self._request_sitl_mavlink_servo_stream()
                continue
            if mtype != "SERVO_OUTPUT_RAW":
                continue
            got_any = True
            self._sitl_client_last_wall = now_wall
            self._sitl_mav_last_msg_wall = now_wall
            if self._sitl_first_servo_wall <= 0.0:
                self._sitl_first_servo_wall = now_wall
            self._sitl_last_command_stale_wall = -1.0
            try:
                d = msg.to_dict()
                pwm_values = [int(d.get(f"servo{i}_raw", 0)) for i in range(1, 9)]
            except Exception:
                continue
            self._handle_sitl_pwm_values(pwm_values, now_wall, source="mavlink")

        # Keep stream request alive even if heartbeat arrives slowly.
        self._request_sitl_mavlink_servo_stream()
        if not got_any:
            no_msg_age = (
                now_wall - self._sitl_mav_last_msg_wall
                if self._sitl_mav_last_msg_wall > 0.0
                else float("inf")
            )
            if (
                no_msg_age >= self._sitl_mav_wait_warn_interval_s
                and now_wall - self._sitl_mav_last_wait_warn_wall >= self._sitl_mav_wait_warn_interval_s
            ):
                print(
                    "[ros2_bridge] Waiting for SITL MAVLink SERVO_OUTPUT_RAW "
                    f"on {self._sitl_mavlink_endpoint}",
                    flush=True,
                )
                self._sitl_mav_last_wait_warn_wall = now_wall

    def _poll_sitl_servo(self) -> None:
        """Poll SITL servo input.

        In MAVLink mode, keep MAVLink as the only servo source to avoid
        source flapping (mavlink<->json) that can introduce control jitter.
        """
        if self._sitl_servo_source == "mavlink":
            self._poll_sitl_servo_mavlink()
            now_wall = time.monotonic()
            mav_fresh = (
                self._sitl_mav_last_msg_wall > 0.0
                and (now_wall - self._sitl_mav_last_msg_wall) <= self._sitl_mavlink_timeout_s
            )
            if mav_fresh:
                if self._sitl_servo_active_source != "mavlink":
                    if now_wall - self._sitl_source_switch_log_wall > 1.0:
                        print(
                            "[ros2_bridge] SITL servo source switched: json -> mavlink",
                            flush=True,
                        )
                        self._sitl_source_switch_log_wall = now_wall
                self._sitl_servo_active_source = "mavlink"
            else:
                # Keep source fixed to MAVLink even when stream is momentarily stale.
                # This prevents servo source oscillation and command jitter.
                self._sitl_servo_active_source = "mavlink"
            return

        self._poll_sitl_servo_endpoint()

    def _poll_sitl_servo_endpoint(self) -> None:
        """Poll incoming ArduPilot servo packets and apply as control input."""
        if not self.sitl_sock:
            return

        now_wall = time.monotonic()
        got_any = False
        while True:
            try:
                pkt, addr = self.sitl_sock.recvfrom(2048)
            except BlockingIOError:
                break
            except Exception:
                break

            # Expected binary packet starts with uint16 magic:
            # 18458 (16ch) or 29569 (32ch).
            if len(pkt) < 8:
                continue
            magic = int.from_bytes(pkt[0:2], byteorder="little", signed=False)
            if magic not in (18458, 29569):
                continue

            if self._sitl_client_addr != addr:
                if self._sitl_client_addr is None:
                    print(f"[ros2_bridge] SITL servo endpoint discovered: {addr}", flush=True)
                else:
                    print(
                        f"[ros2_bridge] SITL servo endpoint changed: "
                        f"{self._sitl_client_addr} -> {addr}",
                        flush=True,
                    )
                self._sitl_client_addr = addr
                self._sitl_client_logged = True
            
            # Keep last-received wall time updated on every packet to avoid
            # emitting false stale warnings when the source port is stable.
            self._sitl_client_last_wall = time.monotonic()
            if self._sitl_first_servo_wall <= 0.0:
                self._sitl_first_servo_wall = self._sitl_client_last_wall
            self._sitl_last_command_stale_wall = -1.0
            got_any = True

            # 16-ch packet: magic(2) + frame_rate(2) + frame_count(4) + pwm(16*2)
            # 32-ch packet: magic(2) + frame_rate(2) + frame_count(4) + pwm(32*2)
            frame_size = 2 + 2 + 4 + (16 * 2 if magic == 18458 else 32 * 2)
            if len(pkt) < frame_size:
                continue
            try:
                pwm_values = list(struct.unpack_from("<16H" if magic == 18458 else "<32H", pkt, 8))
            except struct.error:
                continue

            self._handle_sitl_pwm_values(pwm_values, now_wall, source="json")

        # Emit clear warnings when ArduPilot hasn't started sending servo packets yet.
        # In connected operation, this means QGC is not forwarding manual RC input.
        now_wall = time.monotonic()
        if self._sitl_client_addr is None:
            if now_wall - self._sitl_last_client_missing_wall >= self._sitl_no_client_warn_interval_s:
                print(
                    f"[ros2_bridge] Waiting for SITL servo packets on {self.sitl_addr} "
                    "(QGC joystick enabled, vehicle armed, and mode MANUAL/STABILIZE/DEPTH_HOLD).",
                    flush=True,
                )
                self._sitl_last_client_missing_wall = now_wall
        elif self._sitl_client_last_wall > 0.0 and now_wall - self._sitl_client_last_wall >= self._sitl_no_client_warn_interval_s:
            if now_wall - self._sitl_last_command_stale_wall >= self._sitl_no_client_warn_interval_s:
                print(
                    f"[ros2_bridge] No new SITL servo packets for {now_wall - self._sitl_client_last_wall:.1f}s "
                    f"from {self._sitl_client_addr}",
                    flush=True,
                )
                self._sitl_last_command_stale_wall = now_wall

    def _send_sitl_data(
        self,
        t: float,
        gyro: np.ndarray,
        acc: np.ndarray,
        vel: np.ndarray,
        pos: np.ndarray,
        quat: np.ndarray,
        depth_m: float | None = None,
    ) -> None:
        """Send JSON packet to ArduPilot SITL."""
        if not self.sitl_sock:
            return

        # ArduPilot JSON expects:
        # - gyro: rad/s (body FRD)
        # - accel_body: m/s^2 (body FRD)
        # - position/velocity: NED frame
        # - quaternion: [w, x, y, z] body->NED (newer JSON parser)
        # - attitude: [roll, pitch, yaw] radians (legacy JSON parser)
        now_wall = time.monotonic()
        if self._sitl_t0_wall is None:
            self._sitl_t0_wall = now_wall
        sitl_t = now_wall - self._sitl_t0_wall

        roll, pitch, yaw = self._quat_to_rpy(quat)
        attitude = [float(roll), float(pitch), float(yaw)]

        if depth_m is None or not np.isfinite(float(depth_m)):
            depth_m = self._depth_from_pos_ned(pos)
        depth_m = float(max(0.0, float(depth_m)))

        payload = {
            "timestamp": float(sitl_t),
            "imu": {
                "gyro": [float(x) for x in gyro],
                "accel_body": [float(x) for x in acc]
            },
            "position": [float(x) for x in pos],
            "velocity": [float(x) for x in vel],
            "attitude": attitude,
            "quaternion": [float(x) for x in quat],
            "rng_1": float(depth_m),
            # Force non-time-synced JSON mode to avoid EKF divergence in
            # Submarine SITL (known issue path with external JSON feeds).
            "no_time_sync": True,
        }

        # Keep startup orientation/position consistent from t=0 and only clamp
        # dynamics to conservative ranges (avoids initial frame jump).
        payload["imu"]["gyro"] = [float(x) for x in np.clip(gyro, -20.0, 20.0)]
        payload["imu"]["accel_body"] = [float(x) for x in np.clip(acc, -40.0, 40.0)]
        payload["velocity"] = [float(x) for x in np.clip(vel, -20.0, 20.0)]
        payload["attitude"] = attitude

        # Skip invalid payloads because ArduPilot JSON parser is strict enough
        # to reject non-finite values and then stall lockstep.
        arrs = (
            np.array(payload["imu"]["gyro"], dtype=np.float64),
            np.array(payload["imu"]["accel_body"], dtype=np.float64),
            np.array(payload["velocity"], dtype=np.float64),
            np.array(payload["position"], dtype=np.float64),
            np.array(payload["quaternion"], dtype=np.float64),
            np.array(payload["attitude"], dtype=np.float64),
            np.array([payload["rng_1"]], dtype=np.float64),
        )
        if not np.isfinite(float(payload["timestamp"])) or any(not np.all(np.isfinite(a)) for a in arrs):
            if not self._sitl_nonfinite_warned:
                self._sitl_nonfinite_warned = True
                print("[ros2_bridge] skip SITL packet: non-finite sensor value", flush=True)
            return

        if now_wall - self._sitl_last_sensor_log_wall >= 2.0:
            self._sitl_last_sensor_log_wall = now_wall
            try:
                print(
                    "[ros2_bridge] SITL tx sample "
                    f"t={payload['timestamp']:.3f} "
                    f"pos={payload['position']} vel={payload['velocity']} "
                    f"acc={payload['imu']['accel_body']} rng_1={payload['rng_1']:.3f} "
                    f"bar30_abs_pa={self._bar30_pressure_pa_filt if self._bar30_pressure_pa_filt is not None else float('nan'):.1f}",
                    flush=True,
                )
            except Exception:
                pass

        target = self._sitl_client_addr if self._sitl_client_addr is not None else self.sitl_send_addr
        try:
            prev_target = self._sitl_send_target
            # ArduPilot JSON parser expects line-delimited JSON records.
            msg = (json.dumps(payload) + "\n").encode("utf-8")
            sent = self.sitl_sock.sendto(msg, target)
            self._sitl_send_counter += 1
            self._sitl_send_target = target
            now = time.monotonic()
            if now - self._sitl_last_send_wall > 5.0:
                if self._sitl_servo_source == "mavlink":
                    print(
                        f"[ros2_bridge] SITL send ok (sensor_target={target}, bytes={sent}, packets_sent={self._sitl_send_counter})",
                        flush=True,
                    )
                elif self._sitl_client_addr is None:
                    print(
                        f"[ros2_bridge] SITL send target still default (endpoint not discovered yet): {target} "
                        f"packets_sent={self._sitl_send_counter}",
                        flush=True,
                    )
                    self._sitl_last_no_client_wall = now
                else:
                    print(
                        f"[ros2_bridge] SITL send ok (target={target}, bytes={sent}, packets_sent={self._sitl_send_counter})",
                        flush=True,
                    )
                self._sitl_last_send_wall = now
            if prev_target is not None and prev_target != target:
                print(
                    f"[ros2_bridge] SITL send target changed to servo source: {target}",
                    flush=True,
                )
            if now - self._sitl_last_send_err_wall > 20.0:
                if sent <= 0:
                    print(
                        f"[ros2_bridge] SITL send warning: sent {sent} bytes to {target}",
                        flush=True,
                    )
                    self._sitl_last_send_err_wall = now
        except Exception as exc:
            now = time.monotonic()
            if now - self._sitl_last_send_err_wall > 2.0:
                print(f"[ros2_bridge] SITL send failed to {target}: {exc}", flush=True)
                self._sitl_last_send_err_wall = now
            self._sitl_last_send_wall = now


    def _sensor_slice(self, name: str, data: mujoco.MjData) -> np.ndarray | None:
        """Return a copy of sensor vector by name, or None if missing."""
        sid = self.sensor_ids.get(name, -1)
        if sid < 0:
            return None
        adr = int(self.model.sensor_adr[sid])
        dim = int(self.model.sensor_dim[sid])
        return data.sensordata[adr : adr + dim]

    def _pwm_to_normalized(self, pwm: int) -> float:
        """Convert PWM value (1100-1900) to normalized (-1.0 to 1.0)."""
        if pwm <= 0 or pwm == 65535:  # Invalid or UINT16_MAX (passthrough)
            return 0.0
        if pwm >= self._pwm_neutral:
            denom = max(1.0, float(self._pwm_max - self._pwm_neutral))
        else:
            denom = max(1.0, float(self._pwm_neutral - self._pwm_min))
        return float(np.clip((pwm - self._pwm_neutral) / denom, -1.0, 1.0))

    def _on_mavros_rc_override(self, msg) -> None:
        """Handle MAVROS RC Override messages (from ArduSub/Pixhawk)."""
        if len(msg.channels) < 6:
            return
        
        # Extract channels (ArduSub mapping: 1-indexed in docs, 0-indexed in arrays)
        forward_pwm = msg.channels[self._ch_forward] if self._ch_forward < len(msg.channels) else 1500
        lateral_pwm = msg.channels[self._ch_lateral] if self._ch_lateral < len(msg.channels) else 1500
        throttle_pwm = msg.channels[self._ch_throttle] if self._ch_throttle < len(msg.channels) else 1500
        yaw_pwm = msg.channels[self._ch_yaw] if self._ch_yaw < len(msg.channels) else 1500

        # Convert to normalized values
        fwd = self._pwm_to_normalized(forward_pwm)
        sway = self._pwm_to_normalized(lateral_pwm)
        heave = self._pwm_to_normalized(throttle_pwm)
        yaw = self._pwm_to_normalized(yaw_pwm)
        fwd, sway, yaw, heave = self._sitl_map_and_scale(fwd, sway, yaw, heave)
        if self._sitl_cmd_debug and self.enable_sitl:
            now = time.monotonic()
            if now - self._sitl_last_cmd_log > 0.15:
                print(
                    "[ros2_bridge] MAVROS RC override raw->mapped: "
                    f"raw({forward_pwm},{lateral_pwm},{throttle_pwm},{yaw_pwm}) -> "
                    f"mapped({fwd:+.3f},{sway:+.3f},{yaw:+.3f},{heave:+.3f})",
                    flush=True,
                )
                self._sitl_last_cmd_log = now
        self._handle_normalized_cmd(fwd, sway, yaw, heave)

    def _quat_to_rpy(self, quat: np.ndarray) -> tuple[float, float, float]:
        """Convert quaternion [w, x, y, z] to roll/pitch/yaw in radians."""
        w, x, y, z = quat
        # Normalize defensively to avoid NaN on malformed values.
        norm = float(np.linalg.norm(quat))
        if norm <= 1e-12:
            return 0.0, 0.0, 0.0
        q0, q1, q2, q3 = w / norm, x / norm, y / norm, z / norm

        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            pitch = np.copysign(np.pi / 2.0, sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return float(roll), float(pitch), float(yaw)

    def _on_cmd_vel(self, msg) -> None:
        """Map normalized ROS2 cmd_vel into bounded simulator command units."""
        clip = lambda v: float(np.clip(v, -1.0, 1.0))
        fwd = clip(msg.linear.x)
        sway = clip(msg.linear.y)
        heave = clip(msg.linear.z)
        yaw = clip(msg.angular.z)
        self._handle_normalized_cmd(fwd, sway, yaw, heave)

    @staticmethod
    def _is_ros_context_shutdown_error(exc: Exception) -> bool:
        """Return True when exception indicates normal ROS2 context shutdown."""
        msg = str(exc).lower()
        return (
            "context is not valid" in msg
            or "context is invalid" in msg
            or "rcl_shutdown" in msg
            or "rcl_init() was not called" in msg
        )

    def spin_once(self) -> None:
        """Advance ROS2 callbacks and enforce cmd timeout fail-safe."""
        if not self._enable_ros:
            if self.enable_sitl:
                self._poll_sitl_servo()
            if self.cmd_active and (time.monotonic() - self.last_cmd_wall > self.cmd_timeout_s):
                self._cmd_filter_t = time.monotonic()
                self._cmd_filter_norm = np.zeros(4, dtype=np.float64)
                self.command_callback(0.0, 0.0, 0.0, 0.0)
                self.cmd_active = False
            return
        if not self._ros_ok:
            return
        try:
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            if not self._ros_error_reported:
                self._ros_error_reported = True
                if self._is_ros_context_shutdown_error(exc):
                    print(
                        "[ros2_bridge] ROS2 context closed; callbacks stopped (SITL stays active).",
                        flush=True,
                    )
                else:
                    print(
                        f"[ros2_bridge] ROS2 callbacks disabled (SITL still active): {exc}",
                        flush=True,
                    )
            self._ros_ok = False
            return
        if self.cmd_active and (time.monotonic() - self.last_cmd_wall > self.cmd_timeout_s):
            self._cmd_filter_t = time.monotonic()
            self._cmd_filter_norm = np.zeros(4, dtype=np.float64)
            self.command_callback(0.0, 0.0, 0.0, 0.0)
            self.cmd_active = False

    def _build_camera_info(self, cname: str, width: int, height: int):
        """Build pinhole camera_info from MuJoCo camera fovy."""
        cid = self.cam_ids[cname]
        fovy_deg = float(self.model.cam_fovy[cid])
        fovy = np.deg2rad(fovy_deg)
        fy = 0.5 * height / max(np.tan(0.5 * fovy), 1e-9)
        fx = fy
        cx = 0.5 * width
        cy = 0.5 * height

        msg = self.CameraInfo()
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.header.frame_id = f"{cname}_optical"
        return msg

    def _load_camera_info_yaml(self, path: str, fallback_info, fallback_frame_id: str):
        """Load ROS camera_info YAML and return CameraInfo, or None on failure."""
        path = str(path or "").strip()
        if not path:
            return None

        ypath = Path(path).expanduser()
        if not ypath.exists():
            print(f"[ros2_bridge] camera calibration not found: {ypath}", flush=True)
            return None

        try:
            import yaml
        except Exception:
            print("[ros2_bridge] PyYAML not available; camera calibration skipped", flush=True)
            return None

        try:
            payload = yaml.safe_load(ypath.read_text()) or {}
            if not isinstance(payload, dict):
                return None
        except Exception as exc:
            print(f"[ros2_bridge] failed to read camera calibration {ypath}: {exc}", flush=True)
            return None

        def _extract_vec(container_key: str, flat_key: str, expected_len: int, fallback: list[float]) -> list[float]:
            val = payload.get(flat_key)
            if isinstance(val, list) and len(val) == expected_len:
                return [float(x) for x in val]
            node = payload.get(container_key, {})
            if isinstance(node, dict):
                data = node.get("data")
                if isinstance(data, list) and len(data) == expected_len:
                    return [float(x) for x in data]
            return list(fallback)

        msg = self.CameraInfo()
        msg.width = int(payload.get("image_width", fallback_info.width))
        msg.height = int(payload.get("image_height", fallback_info.height))
        msg.distortion_model = str(payload.get("distortion_model", fallback_info.distortion_model or "plumb_bob"))
        msg.d = _extract_vec("distortion_coefficients", "D", 5, list(fallback_info.d)[:5] or [0.0] * 5)
        msg.k = _extract_vec("camera_matrix", "K", 9, list(fallback_info.k))
        msg.r = _extract_vec("rectification_matrix", "R", 9, list(fallback_info.r))
        msg.p = _extract_vec("projection_matrix", "P", 12, list(fallback_info.p))
        msg.header.frame_id = str(payload.get("frame_id", fallback_frame_id))
        return msg

    def _quat_to_yaw(self, quat: np.ndarray) -> float:
        """Extract yaw angle from quaternion [w, x, y, z]."""
        w, x, y, z = quat
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    @staticmethod
    def _rotmat_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion [w, x, y, z]."""
        m = rot
        tr = float(m[0, 0] + m[1, 1] + m[2, 2])
        if tr > 0.0:
            s = np.sqrt(tr + 1.0) * 2.0
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        q = np.array([w, x, y, z], dtype=np.float64)
        n = float(np.linalg.norm(q))
        return q / max(n, 1e-12)

    @staticmethod
    def _quat_wxyz_to_rotmat(quat: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to rotation matrix."""
        w, x, y, z = quat
        n = float(np.linalg.norm([w, x, y, z]))
        if n <= 1e-12:
            return np.eye(3, dtype=np.float64)
        w, x, y, z = w / n, x / n, y / n, z / n
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _imu_vectors_in_body(
        self,
        data: mujoco.MjData,
        gyro: np.ndarray | None,
        acc: np.ndarray | None,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Transform IMU vectors from imu_site axes to base body axes."""
        gyro_bmj = np.array(gyro, dtype=np.float64) if gyro is not None else None
        acc_bmj = np.array(acc, dtype=np.float64) if acc is not None else None
        if self._base_id < 0 or self._imu_site_id < 0:
            return gyro_bmj, acc_bmj
        try:
            base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
            imu_rot_enu = data.site_xmat[self._imu_site_id].reshape(3, 3).copy()
            rot_bmj_imu = base_rot_enu.T @ imu_rot_enu
            if gyro_bmj is not None:
                gyro_bmj = rot_bmj_imu @ gyro_bmj
            if acc_bmj is not None:
                acc_bmj = rot_bmj_imu @ acc_bmj
        except Exception:
            pass
        return gyro_bmj, acc_bmj

    def _dvl_velocity_body(
        self,
        data: mujoco.MjData,
        dvl_vel_sensor: np.ndarray | None,
        gyro_bmj: np.ndarray | None,
    ) -> np.ndarray | None:
        """Return filtered COM linear velocity in base body frame from DVL sensor."""
        if dvl_vel_sensor is None:
            return None

        vel_body = np.array(dvl_vel_sensor, dtype=np.float64)
        if self._base_id >= 0 and self._dvl_site_id >= 0:
            try:
                base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
                dvl_rot_enu = data.site_xmat[self._dvl_site_id].reshape(3, 3).copy()
                rot_bmj_dvl = base_rot_enu.T @ dvl_rot_enu
                vel_body = rot_bmj_dvl @ vel_body
                if gyro_bmj is not None:
                    r_enu = data.site_xpos[self._dvl_site_id] - data.xpos[self._base_id]
                    r_body = base_rot_enu.T @ r_enu
                    vel_body = vel_body - np.cross(gyro_bmj, r_body)
            except Exception:
                pass

        vel_body = np.nan_to_num(vel_body, nan=0.0, posinf=0.0, neginf=0.0)
        alpha = float(self._dvl_filter_alpha)
        if alpha <= 0.0:
            self._dvl_vel_body_filt = vel_body
            return vel_body
        if self._dvl_vel_body_filt is None:
            self._dvl_vel_body_filt = vel_body
        else:
            self._dvl_vel_body_filt = ((1.0 - alpha) * self._dvl_vel_body_filt) + (alpha * vel_body)
        return self._dvl_vel_body_filt.copy()

    def publish(self, data: mujoco.MjData) -> None:
        """Publish sensors/images at configured rates using simulation time."""
        if self.enable_sitl:
            self._poll_sitl_servo()

        sim_t = float(data.time)
        if sim_t <= self.last_pub_t:
            return
        self.last_pub_t = sim_t

        # Extract sensor data for both ROS2 and SITL.
        quat = self._sensor_slice("imu_quat", data)
        gyro = self._sensor_slice("imu_gyro", data)
        acc = self._sensor_slice("imu_acc", data)
        dvl_vel_sensor = self._sensor_slice("dvl_vel_body", data)
        gyro_bmj, acc_bmj = self._imu_vectors_in_body(data, gyro, acc)
        dvl_vel_body = self._dvl_velocity_body(data, dvl_vel_sensor, gyro_bmj)
        bar30_depth_m, bar30_pressure_pa = self._bar30_depth_pressure_from_model(data)

        # Send to SITL if enabled
        if self.enable_sitl and self._base_id >= 0 and gyro_bmj is not None and acc_bmj is not None:
            base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
            base_pos_enu = data.xpos[self._base_id].copy()
            # Do not use data.cvel for JSON NED velocity.
            # In this scene it can include large spatial terms and produce
            # non-physical NED velocity spikes in ArduSub (depth-hold diverges).
            base_vel_enu = np.zeros(3, dtype=np.float64)
            prev_t = self._sitl_prev_sim_t
            prev_pos = self._sitl_prev_pos_enu
            if prev_t is not None and prev_pos is not None:
                dt = sim_t - float(prev_t)
                if 1.0e-4 <= dt <= 0.2:
                    vel_fd = (base_pos_enu - prev_pos) / dt
                    if np.all(np.isfinite(vel_fd)):
                        vel_fd = np.clip(vel_fd, -8.0, 8.0)
                        self._sitl_vel_lpf_enu = 0.75 * self._sitl_vel_lpf_enu + 0.25 * vel_fd
                        base_vel_enu = self._sitl_vel_lpf_enu.copy()
            self._sitl_prev_sim_t = sim_t
            self._sitl_prev_pos_enu = base_pos_enu.copy()

            # Convert vectors/pose to ArduPilot conventions (NED + FRD).
            pos_ned = self._enu_to_ned @ base_pos_enu
            vel_ned = self._enu_to_ned @ base_vel_enu
            if bar30_depth_m is not None and np.isfinite(float(bar30_depth_m)):
                pos_ned[2] = float(max(0.0, float(bar30_depth_m)))
                if self._bar30_depth_prev_m is not None and self._bar30_depth_prev_t is not None:
                    dt_baro = float(sim_t - float(self._bar30_depth_prev_t))
                    if 1.0e-4 <= dt_baro <= 0.2:
                        vz_baro = (float(bar30_depth_m) - float(self._bar30_depth_prev_m)) / dt_baro
                        if np.isfinite(vz_baro):
                            vel_ned[2] = float(np.clip(vz_baro, -5.0, 5.0))
                self._bar30_depth_prev_m = float(bar30_depth_m)
                self._bar30_depth_prev_t = float(sim_t)
            gyro_frd = self._bmj_to_frd @ gyro_bmj
            # Provide body-frame specific force to ArduSub JSON SITL.
            # Zeroing accel_body makes vertical EKF channels drift (z/vz runaway).
            acc_frd = np.nan_to_num(
                self._bmj_to_frd @ acc_bmj,
                nan=0.0,
                posinf=0.0,
                neginf=0.0,
            )

            rot_ned_bfrd = self._enu_to_ned @ base_rot_enu @ self._bmj_to_frd.T
            quat_ned_bfrd = self._rotmat_to_quat_wxyz(rot_ned_bfrd)

            self._send_sitl_data(
                sim_t,
                gyro_frd,
                acc_frd,
                vel_ned,
                pos_ned,
                quat_ned_bfrd,
                depth_m=bar30_depth_m,
            )

        # ROS2 context may be unavailable while SITL continues running.
        if not self._enable_ros or not self._ros_ok:
            return

        def _safe_publish(publisher, msg, label: str) -> bool:
            try:
                publisher.publish(msg)
                return True
            except Exception as exc:
                if not self._ros_error_reported:
                    self._ros_error_reported = True
                    if self._is_ros_context_shutdown_error(exc):
                        print(
                            f"[ros2_bridge] ROS2 context closed; publish stopped ({label}).",
                            flush=True,
                        )
                    else:
                        print(f"[ros2_bridge] ROS2 publish blocked ({label}): {exc}", flush=True)
                self._ros_ok = False
                return False

        try:
            stamp = self.node.get_clock().now().to_msg()
        except Exception as exc:
            if not self._ros_error_reported:
                self._ros_error_reported = True
                if self._is_ros_context_shutdown_error(exc):
                    print(
                        "[ros2_bridge] ROS2 context closed; publishing stopped (SITL stays active).",
                        flush=True,
                    )
                else:
                    print(
                        f"[ros2_bridge] ROS2 publishing disabled (SITL still active): {exc}",
                        flush=True,
                    )
            self._ros_ok = False
            return

        if sim_t + 1e-9 >= self.next_sensor_t:
            self.next_sensor_t = sim_t + self.sensor_dt

            # (Already extracted above: quat, gyro, acc, dvl_vel)
            dvl_alt = self._sensor_slice("dvl_altitude", data)
            depth_m = bar30_depth_m
            if depth_m is None and self._base_id >= 0:
                depth_m = float(max(0.0, -float(data.xpos[self._base_id][2])))
            if bar30_pressure_pa is None and depth_m is not None:
                bar30_pressure_pa = self._pressure_abs_from_depth_m(
                    float(depth_m),
                    self._bar30_surface_pressure_pa,
                    self._bar30_water_density,
                    self._bar30_gravity,
                )

            # Build ROS-FLU orientation/vectors from model body frame.
            quat_ros = None
            gyro_ros = None
            acc_ros = None
            dvl_vel_body_ros = None
            if quat is not None:
                rot_world_body = self._quat_wxyz_to_rotmat(np.array(quat, dtype=np.float64))
                rot_world_flu = rot_world_body @ self._bmj_to_flu.T
                quat_ros = self._rotmat_to_quat_wxyz(rot_world_flu)
            if gyro_bmj is not None:
                gyro_ros = self._bmj_to_flu @ np.array(gyro_bmj, dtype=np.float64)
            if acc_bmj is not None:
                acc_ros = self._bmj_to_flu @ np.array(acc_bmj, dtype=np.float64)
            if dvl_vel_body is not None:
                dvl_vel_body_ros = self._bmj_to_flu @ np.array(dvl_vel_body, dtype=np.float64)

            # Publish IMU (ROS FLU frame).
            if quat is not None and gyro_bmj is not None and acc_bmj is not None:
                imu = self.Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = "imu_link"
                # MuJoCo framequat is [w, x, y, z]
                imu.orientation.w = float(quat_ros[0])
                imu.orientation.x = float(quat_ros[1])
                imu.orientation.y = float(quat_ros[2])
                imu.orientation.z = float(quat_ros[3])
                imu.angular_velocity.x = float(gyro_ros[0])
                imu.angular_velocity.y = float(gyro_ros[1])
                imu.angular_velocity.z = float(gyro_ros[2])
                imu.linear_acceleration.x = float(acc_ros[0])
                imu.linear_acceleration.y = float(acc_ros[1])
                imu.linear_acceleration.z = float(acc_ros[2])
                imu.orientation_covariance[0] = 1e-4
                imu.orientation_covariance[4] = 1e-4
                imu.orientation_covariance[8] = 1e-4
                imu.angular_velocity_covariance[0] = 5e-4
                imu.angular_velocity_covariance[4] = 5e-4
                imu.angular_velocity_covariance[8] = 5e-4
                imu.linear_acceleration_covariance[0] = 1e-2
                imu.linear_acceleration_covariance[4] = 1e-2
                imu.linear_acceleration_covariance[8] = 1e-2
                if not _safe_publish(self.pub_imu, imu, "/imu/data"):
                    return

            # Publish DVL raw velocity (sensor frame).
            if dvl_vel_sensor is not None and self.pub_dvl_vel_raw is not None:
                tw_raw = self.TwistStamped()
                tw_raw.header.stamp = stamp
                tw_raw.header.frame_id = "dvl_link"
                tw_raw.twist.linear.x = float(dvl_vel_sensor[0])
                tw_raw.twist.linear.y = float(dvl_vel_sensor[1])
                tw_raw.twist.linear.z = float(dvl_vel_sensor[2])
                if not _safe_publish(self.pub_dvl_vel_raw, tw_raw, "/dvl/velocity_raw"):
                    return

            # Publish DVL velocity corrected to base_link COM/body frame (ROS FLU).
            if dvl_vel_body_ros is not None:
                tw = self.TwistStamped()
                tw.header.stamp = stamp
                tw.header.frame_id = "base_link"
                tw.twist.linear.x = float(dvl_vel_body_ros[0])
                tw.twist.linear.y = float(dvl_vel_body_ros[1])
                tw.twist.linear.z = float(dvl_vel_body_ros[2])
                if not _safe_publish(self.pub_dvl_vel, tw, "/dvl/velocity"):
                    return

            # Publish DVL altitude
            if dvl_alt is not None:
                rg = self.Range()
                rg.header.stamp = stamp
                rg.header.frame_id = "dvl_link"
                rg.radiation_type = self.Range.ULTRASOUND
                rg.field_of_view = 0.25
                rg.min_range = 0.05
                rg.max_range = 30.0
                rng = float(dvl_alt[0])
                rg.range = float("inf") if rng < 0.0 else rng
                if not _safe_publish(self.pub_dvl_alt, rg, "/dvl/altitude"):
                    return

            # Publish depth (meters below surface, clipped at 0 when above water)
            if depth_m is not None and self.pub_depth is not None:
                depth_msg = self.Float32()
                depth_msg.data = float(depth_m)
                if not _safe_publish(self.pub_depth, depth_msg, "/depth"):
                    return
            if bar30_pressure_pa is not None and self.pub_bar30_pressure is not None:
                baro_msg = self.Float32()
                baro_msg.data = float(bar30_pressure_pa)
                if not _safe_publish(self.pub_bar30_pressure, baro_msg, "/bar30/pressure_pa"):
                    return

            # Publish DVL odometry (integrated from corrected body velocity).
            if dvl_vel_body is not None and quat is not None:
                if self._last_odom_time < 0.0:
                    dt = self.sensor_dt
                else:
                    dt = float(np.clip(sim_t - self._last_odom_time, 1e-4, 0.2))
                self._last_odom_time = sim_t

                rot_world_body = self._quat_wxyz_to_rotmat(np.array(quat, dtype=np.float64))
                vel_world = rot_world_body @ np.array(dvl_vel_body, dtype=np.float64)
                self._odom_pos += vel_world * dt
                self._odom_yaw = self._quat_to_yaw(np.array(quat, dtype=np.float64))

                odom = self.Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = float(self._odom_pos[0])
                odom.pose.pose.position.y = float(self._odom_pos[1])
                odom.pose.pose.position.z = float(self._odom_pos[2])
                odom.pose.pose.orientation.w = float(quat_ros[0])
                odom.pose.pose.orientation.x = float(quat_ros[1])
                odom.pose.pose.orientation.y = float(quat_ros[2])
                odom.pose.pose.orientation.z = float(quat_ros[3])
                odom.twist.twist.linear.x = float(dvl_vel_body_ros[0])
                odom.twist.twist.linear.y = float(dvl_vel_body_ros[1])
                odom.twist.twist.linear.z = float(dvl_vel_body_ros[2])
                odom.pose.covariance[0] = 0.03
                odom.pose.covariance[7] = 0.03
                odom.pose.covariance[14] = 0.06
                odom.pose.covariance[21] = 0.08
                odom.pose.covariance[28] = 0.08
                odom.pose.covariance[35] = 0.04
                odom.twist.covariance[0] = 0.02
                odom.twist.covariance[7] = 0.02
                odom.twist.covariance[14] = 0.03
                if not _safe_publish(self.pub_dvl_odom, odom, "/dvl/odometry"):
                    return
            elif dvl_vel_body is None:
                self._last_odom_time = -1.0

            # Publish ground truth pose from MuJoCo
            if self._base_id >= 0:
                pos = data.xpos[self._base_id]
                quat_mj = data.xquat[self._base_id]  # MuJoCo quat is [w, x, y, z]
                
                gt = self.PoseStamped()
                gt.header.stamp = stamp
                gt.header.frame_id = "world"
                gt.pose.position.x = float(pos[0])
                gt.pose.position.y = float(pos[1])
                gt.pose.position.z = float(pos[2])
                rot_world_body = self._quat_wxyz_to_rotmat(np.array(quat_mj, dtype=np.float64))
                rot_world_flu = rot_world_body @ self._bmj_to_flu.T
                quat_gt_ros = self._rotmat_to_quat_wxyz(rot_world_flu)
                gt.pose.orientation.w = float(quat_gt_ros[0])
                gt.pose.orientation.x = float(quat_gt_ros[1])
                gt.pose.orientation.y = float(quat_gt_ros[2])
                gt.pose.orientation.z = float(quat_gt_ros[3])
                if not _safe_publish(self.pub_ground_truth, gt, "/mujoco/ground_truth/pose"):
                    return

        if self.publish_images and sim_t + 1e-9 >= self.next_image_t:
            self.next_image_t = sim_t + self.image_dt
            for cname, renderer in self.renderers.items():
                renderer.update_scene(data, camera=cname)
                rgb = renderer.render()
                if rgb is None:
                    continue
                img = np.ascontiguousarray(rgb)
                msg = self.Image()
                msg.header.stamp = stamp
                msg.header.frame_id = f"{cname}_optical"
                msg.height = int(img.shape[0])
                msg.width = int(img.shape[1])
                msg.encoding = "rgb8"
                msg.is_bigendian = False
                msg.step = int(img.shape[1] * 3)
                # sensor_msgs/msg/Image.data setter is much cheaper for array('B')
                # than for raw bytes (avoids Python-level element-by-element checks).
                msg.data = array("B", img.tobytes())
                if not _safe_publish(self.image_pubs[cname], msg, f"/stereo/{cname}/image_raw"):
                    return

                info = self.cam_info[cname]
                info.header.stamp = stamp
                if not _safe_publish(self.info_pubs[cname], info, f"/stereo/{cname}/camera_info"):
                    return

    def reset_odometry(self) -> None:
        """Reset DVL odometry integration to zero."""
        self._odom_pos = np.array([0.0, 0.0, 0.0])
        self._odom_yaw = 0.0

    def shutdown(self) -> None:
        """Release renderers and shutdown ROS2 node cleanly."""
        for renderer in self.renderers.values():
            try:
                renderer.close()
            except Exception:
                pass
        if not self._enable_ros:
            return
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass
