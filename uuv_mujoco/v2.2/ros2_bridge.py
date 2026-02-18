"""ROS2 bridge for MuJoCo UUV simulation.

Responsibilities:
1) Subscribe `/cmd_vel` and convert it to internal command channels.
2) Subscribe `/mavros/rc/override` for MAVROS/ArduSub compatibility.
3) Publish IMU/DVL sensor streams from MuJoCo sensor data.
4) Publish DVL odometry for position controller compatibility.
5) Publish ground truth pose for debugging.
6) Optionally publish stereo images and camera_info.
"""

import json
import os
import socket
import struct
import time
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
        camera_calib_left: str = "",
        camera_calib_right: str = "",
    ) -> None:
        # ROS2 imports are runtime-optional so the simulator can run without ROS2.
        try:
            import rclpy
            from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
            from nav_msgs.msg import Odometry
            from rclpy.node import Node
            from sensor_msgs.msg import CameraInfo, Image, Imu, Range
            from std_msgs.msg import Header
        except ImportError as exc:
            raise RuntimeError(
                "ROS2 packages not found. Install rclpy + sensor_msgs + geometry_msgs + nav_msgs."
            ) from exc

        # Try to import mavros_msgs for MAVROS compatibility
        self.has_mavros = False
        try:
            from mavros_msgs.msg import OverrideRCIn
            self.OverrideRCIn = OverrideRCIn
            self.has_mavros = True
        except ImportError:
            self.OverrideRCIn = None
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

        if not self.rclpy.ok():
            self.rclpy.init(args=None)
        self.node = Node("uuv_mujoco_bridge")
        # Keep SITL running even if ROS2 context becomes invalid at runtime.
        self._ros_ok = True
        self._ros_error_reported = False

        self.command_callback = command_callback
        self.cmd_limit = float(cmd_limit)
        self.cmd_timeout_s = 0.7
        self.last_cmd_wall = -1.0
        self.cmd_active = False
        self.enable_mavros = enable_mavros
        self._cmd_filter_norm = np.zeros(4, dtype=np.float64)
        self._cmd_filter_t = -1.0
        self._cmd_deadband_norm = 0.05
        self._cmd_slew_rate_norm = 3.0

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
        self._sitl_last_command_hz_log = time.monotonic()
        self._sitl_last_send_wall = -1.0
        self._sitl_last_send_err_wall = -1.0
        self._sitl_last_no_client_wall = -1.0
        self._sitl_send_counter = 0
        self._sitl_last_nonzero_cmd = None
        self._sitl_nonfinite_warned = False
        self._sitl_t0_wall = None
        # Frame conversion:
        # - MuJoCo world is treated as ENU-like (z-up).
        # - ArduPilot JSON expects NED world and FRD body vectors.
        self._enu_to_ned = np.diag([1.0, 1.0, -1.0])
        # MuJoCo body axes in this model are [x=fwd, y=down, z=right] (FDR),
        # while ArduPilot expects FRD [x=fwd, y=right, z=down].
        self._bmj_to_frd = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        )
        if self.enable_sitl:
            if int(sitl_send_port) <= 0:
                raise ValueError("sitl-send-port must be a positive integer")
            self._connect_sitl()

        # DVL odometry integration state
        self._odom_pos = np.array([0.0, 0.0, 0.0])
        self._odom_yaw = 0.0
        self._last_odom_time = -1.0

        # Core sensor publishers
        self.pub_imu = self.node.create_publisher(self.Imu, "/imu/data", 10)
        self.pub_dvl_vel = self.node.create_publisher(self.TwistStamped, "/dvl/velocity", 10)
        self.pub_dvl_alt = self.node.create_publisher(self.Range, "/dvl/altitude", 10)
        self.pub_dvl_odom = self.node.create_publisher(self.Odometry, "/dvl/odometry", 10)
        self.pub_ground_truth = self.node.create_publisher(self.PoseStamped, "/mujoco/ground_truth/pose", 10)
        
        # Command subscribers
        self.sub_cmd = self.node.create_subscription(self.Twist, "/cmd_vel", self._on_cmd_vel, 10)
        
        # MAVROS RC Override subscriber (for ArduSub/Pixhawk compatibility)
        self.sub_mavros_rc = None
        if self.has_mavros and self.enable_mavros:
            self.sub_mavros_rc = self.node.create_subscription(
                self.OverrideRCIn, "/mavros/rc/override", self._on_mavros_rc_override, 10
            )

        # PWM parameters for MAVROS
        self._pwm_min = 1100
        self._pwm_max = 1900
        self._pwm_neutral = 1500
        # MAVLink RC input order is Roll(1), Pitch(2), Throttle(3), Yaw(4).
        # Zero-based indices are used below.
        self._ch_forward = self._env_to_int("ROS2_UUV_RC_CH_FORWARD", 1)
        self._ch_lateral = self._env_to_int("ROS2_UUV_RC_CH_LATERAL", 0)
        self._ch_throttle = self._env_to_int("ROS2_UUV_RC_CH_THROTTLE", 2)
        self._ch_yaw = self._env_to_int("ROS2_UUV_RC_CH_YAW", 3)
        self.node.get_logger().info(
            f"Using MAVLink RC override channel map: "
            f"forward={self._ch_forward}, lateral={self._ch_lateral}, "
            f"throttle={self._ch_throttle}, yaw={self._ch_yaw}"
        )

        self.model = model
        self.sensor_ids = {}
        for sname in ("imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude", "base_pos"):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
            if sid >= 0:
                self.sensor_ids[sname] = sid

        # Get base body ID for ground truth
        self._base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

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

        if self.publish_images:
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
                self.image_pubs[cname] = self.node.create_publisher(self.Image, f"{topic_ns}/image_raw", 2)
                self.info_pubs[cname] = self.node.create_publisher(
                    self.CameraInfo, f"{topic_ns}/camera_info", 2
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

        topics_info = "/cmd_vel -> control, /imu/data, /dvl/velocity, /dvl/odometry, /dvl/altitude"
        if self.has_mavros and self.enable_mavros:
            topics_info += ", /mavros/rc/override"
        if self.publish_images:
            topics_info += ", /stereo/*"
        if self.enable_sitl:
            topics_info += f", SITL({self.sitl_addr[0]}:{self.sitl_addr[1]})"
        self.node.get_logger().info(f"ROS2 bridge active: {topics_info}")

    @staticmethod
    def _finite_or_zero(value: float) -> float:
        return float(value) if np.isfinite(value) else 0.0

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
            self.sitl_sock.bind(self.sitl_listen_addr)
            self.sitl_sock.setblocking(False)
            sock_addr = self.sitl_sock.getsockname()
            print(
                f"[ros2_bridge] SITL socket initialized (listen {sock_addr}, default target {self.sitl_addr})",
                flush=True,
            )
        except Exception as e:
            print(f"[ros2_bridge] Failed to init SITL socket: {e}", flush=True)

    def _poll_sitl_servo_endpoint(self) -> None:
        """Poll incoming ArduPilot servo packets and apply as control input."""
        if not self.sitl_sock:
            return
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
                if not self._sitl_client_logged:
                    self._sitl_client_logged = True

            self._sitl_client_last_wall = time.monotonic()

            # 16-ch packet: magic(2) + frame_rate(2) + frame_count(4) + pwm(16*2)
            # 32-ch packet: magic(2) + frame_rate(2) + frame_count(4) + pwm(32*2)
            frame_size = 2 + 2 + 4 + (16 * 2 if magic == 18458 else 32 * 2)
            if len(pkt) < frame_size:
                continue
            try:
                pwm_values = list(struct.unpack_from("<16H" if magic == 18458 else "<32H", pkt, 8))
            except struct.error:
                continue

            if len(pwm_values) <= max(self._ch_forward, self._ch_lateral, self._ch_throttle, self._ch_yaw):
                continue

            fwd = self._pwm_to_normalized(int(pwm_values[self._ch_forward]))
            sway = self._pwm_to_normalized(int(pwm_values[self._ch_lateral]))
            heave = self._pwm_to_normalized(int(pwm_values[self._ch_throttle]))
            yaw = self._pwm_to_normalized(int(pwm_values[self._ch_yaw]))
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

    def _send_sitl_data(self, t: float, gyro: np.ndarray, acc: np.ndarray, vel: np.ndarray, pos: np.ndarray, quat: np.ndarray) -> None:
        """Send JSON packet to ArduPilot SITL."""
        if not self.sitl_sock:
            return

        # ArduPilot JSON expects:
        # - gyro: rad/s (body FRD)
        # - accel_body: m/s^2 (body FRD)
        # - position/velocity: NED frame
        # - quaternion: [w, x, y, z] body->NED
        now_wall = time.monotonic()
        if self._sitl_t0_wall is None:
            self._sitl_t0_wall = now_wall
        sitl_t = now_wall - self._sitl_t0_wall

        payload = {
            "timestamp": float(sitl_t),
            "imu": {
                "gyro": [float(x) for x in gyro],
                "accel_body": [float(x) for x in acc]
            },
            "position": [float(x) for x in pos],
            "velocity": [float(x) for x in vel],
            "quaternion": [float(x) for x in quat],
        }

        # ArduSub SITL can stall at startup if the first IMU samples contain
        # extreme transients. Seed the first second with a calm packet and
        # clamp subsequent values to conservative ranges.
        if sitl_t < 1.0:
            payload = {
                "timestamp": float(sitl_t),
                "imu": {
                    "gyro": [0.0, 0.0, 0.0],
                    "accel_body": [0.0, 0.0, 0.0],
                },
                "position": [0.0, 0.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
                "quaternion": [1.0, 0.0, 0.0, 0.0],
            }
        else:
            payload["imu"]["gyro"] = [float(x) for x in np.clip(gyro, -20.0, 20.0)]
            payload["imu"]["accel_body"] = [float(x) for x in np.clip(acc, -40.0, 40.0)]
            payload["velocity"] = [float(x) for x in np.clip(vel, -20.0, 20.0)]

        # Skip invalid payloads because ArduPilot JSON parser is strict enough
        # to reject non-finite values and then stall lockstep.
        arrs = (
            np.array(payload["imu"]["gyro"], dtype=np.float64),
            np.array(payload["imu"]["accel_body"], dtype=np.float64),
            np.array(payload["velocity"], dtype=np.float64),
            np.array(payload["position"], dtype=np.float64),
            np.array(payload["quaternion"], dtype=np.float64),
        )
        if not np.isfinite(float(payload["timestamp"])) or any(not np.all(np.isfinite(a)) for a in arrs):
            if not self._sitl_nonfinite_warned:
                self._sitl_nonfinite_warned = True
                print("[ros2_bridge] skip SITL packet: non-finite sensor value", flush=True)
            return

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
                if self._sitl_client_addr is None:
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
        self._handle_normalized_cmd(fwd, sway, yaw, heave)

    def _on_cmd_vel(self, msg) -> None:
        """Map normalized ROS2 cmd_vel into bounded simulator command units."""
        clip = lambda v: float(np.clip(v, -1.0, 1.0))
        fwd = clip(msg.linear.x)
        sway = clip(msg.linear.y)
        heave = clip(msg.linear.z)
        yaw = clip(msg.angular.z)
        self._handle_normalized_cmd(fwd, sway, yaw, heave)

    def spin_once(self) -> None:
        """Advance ROS2 callbacks and enforce cmd timeout fail-safe."""
        if not self._ros_ok:
            return
        try:
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            if not self._ros_error_reported:
                self._ros_error_reported = True
                print(f"[ros2_bridge] ROS2 callbacks disabled (SITL still active): {exc}", flush=True)
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

    def publish(self, data: mujoco.MjData) -> None:
        """Publish sensors/images at configured rates using simulation time."""
        if self.enable_sitl:
            self._poll_sitl_servo_endpoint()

        sim_t = float(data.time)
        if sim_t <= self.last_pub_t:
            return
        self.last_pub_t = sim_t

        # Extract sensor data for both ROS2 and SITL
        quat = self._sensor_slice("imu_quat", data)
        gyro = self._sensor_slice("imu_gyro", data)
        acc = self._sensor_slice("imu_acc", data)
        dvl_vel = self._sensor_slice("dvl_vel_body", data)
        base_pos = None
        if self._base_id >= 0:
            base_pos = data.xpos[self._base_id]

        # Send to SITL if enabled
        if self.enable_sitl and self._base_id >= 0 and gyro is not None and acc is not None:
            base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
            base_pos_enu = data.xpos[self._base_id].copy()
            base_vel_enu = data.cvel[self._base_id, 3:6].copy()

            # Convert vectors/pose to ArduPilot conventions (NED + FRD).
            pos_ned = self._enu_to_ned @ base_pos_enu
            vel_ned = self._enu_to_ned @ base_vel_enu
            gyro_frd = self._bmj_to_frd @ gyro
            acc_frd = self._bmj_to_frd @ acc

            rot_ned_bfrd = self._enu_to_ned @ base_rot_enu @ self._bmj_to_frd.T
            quat_ned_bfrd = self._rotmat_to_quat_wxyz(rot_ned_bfrd)

            self._send_sitl_data(sim_t, gyro_frd, acc_frd, vel_ned, pos_ned, quat_ned_bfrd)

        # ROS2 context may be unavailable while SITL continues running.
        if not self._ros_ok:
            return

        def _safe_publish(publisher, msg, label: str) -> bool:
            try:
                publisher.publish(msg)
                return True
            except Exception as exc:
                if not self._ros_error_reported:
                    self._ros_error_reported = True
                    print(f"[ros2_bridge] ROS2 publish blocked ({label}): {exc}", flush=True)
                self._ros_ok = False
                return False

        try:
            stamp = self.node.get_clock().now().to_msg()
        except Exception as exc:
            if not self._ros_error_reported:
                self._ros_error_reported = True
                print(f"[ros2_bridge] ROS2 publishing disabled (SITL still active): {exc}", flush=True)
            self._ros_ok = False
            return

        if sim_t + 1e-9 >= self.next_sensor_t:
            self.next_sensor_t = sim_t + self.sensor_dt

            # (Already extracted above: quat, gyro, acc, dvl_vel)
            dvl_alt = self._sensor_slice("dvl_altitude", data)

            # Publish IMU
            if quat is not None and gyro is not None and acc is not None:
                imu = self.Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = "imu_link"
                # MuJoCo framequat is [w, x, y, z]
                imu.orientation.w = float(quat[0])
                imu.orientation.x = float(quat[1])
                imu.orientation.y = float(quat[2])
                imu.orientation.z = float(quat[3])
                imu.angular_velocity.x = float(gyro[0])
                imu.angular_velocity.y = float(gyro[1])
                imu.angular_velocity.z = float(gyro[2])
                imu.linear_acceleration.x = float(acc[0])
                imu.linear_acceleration.y = float(acc[1])
                imu.linear_acceleration.z = float(acc[2])
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

            # Publish DVL velocity
            if dvl_vel is not None:
                tw = self.TwistStamped()
                tw.header.stamp = stamp
                tw.header.frame_id = "dvl_link"
                tw.twist.linear.x = float(dvl_vel[0])
                tw.twist.linear.y = float(dvl_vel[1])
                tw.twist.linear.z = float(dvl_vel[2])
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

            # Publish DVL Odometry (integrated from velocity)
            if dvl_vel is not None and quat is not None:
                dt = self.sensor_dt
                yaw = self._quat_to_yaw(quat)
                
                # Integrate velocity in world frame
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)
                vx_world = dvl_vel[0] * cos_yaw - dvl_vel[1] * sin_yaw
                vy_world = dvl_vel[0] * sin_yaw + dvl_vel[1] * cos_yaw
                vz_world = dvl_vel[2]
                
                self._odom_pos[0] += vx_world * dt
                self._odom_pos[1] += vy_world * dt
                self._odom_pos[2] += vz_world * dt
                self._odom_yaw = yaw

                odom = self.Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = float(self._odom_pos[0])
                odom.pose.pose.position.y = float(self._odom_pos[1])
                odom.pose.pose.position.z = float(self._odom_pos[2])
                odom.pose.pose.orientation.w = float(quat[0])
                odom.pose.pose.orientation.x = float(quat[1])
                odom.pose.pose.orientation.y = float(quat[2])
                odom.pose.pose.orientation.z = float(quat[3])
                odom.twist.twist.linear.x = float(dvl_vel[0])
                odom.twist.twist.linear.y = float(dvl_vel[1])
                odom.twist.twist.linear.z = float(dvl_vel[2])
                if not _safe_publish(self.pub_dvl_odom, odom, "/dvl/odometry"):
                    return

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
                gt.pose.orientation.w = float(quat_mj[0])
                gt.pose.orientation.x = float(quat_mj[1])
                gt.pose.orientation.y = float(quat_mj[2])
                gt.pose.orientation.z = float(quat_mj[3])
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
                msg.data = img.tobytes()
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
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass
