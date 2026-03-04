"""Main runtime for the UUV MuJoCo simulator.

The script keeps simulation, controls, calibration, validation, and optional
ROS2 publishing in a single entrypoint so that model tuning is reproducible.
"""

import argparse
import json
import os
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

MODEL_PATH = Path(__file__).resolve().parent / "urdf_full_scene.xml"
PROFILE_PATH = Path(__file__).resolve().parent / "sim_profiles.json"
THRUSTER_PERF_PATH = Path(__file__).resolve().parent / "thruster_performance.json"


def main() -> None:
    """Parse CLI options, initialize runtime state, and execute selected mode."""
    parser = argparse.ArgumentParser(description="UUV MuJoCo runner")
    parser.add_argument(
        "--scene",
        type=str,
        default=str(MODEL_PATH),
        help="MJCF scene path (default: urdf_full_scene.xml)",
    )
    parser.add_argument(
        "--imu-stabilize",
        action="store_true",
        help="Apply IMU-based roll/pitch/yaw stabilization to keep upright",
    )
    parser.add_argument(
        "--imu-stab-kp",
        type=float,
        default=8.0,
        help="IMU stabilization proportional gain",
    )
    parser.add_argument(
        "--imu-stab-kd",
        type=float,
        default=2.0,
        help="IMU stabilization derivative gain",
    )
    parser.add_argument(
        "--imu-stab-max",
        type=float,
        default=10.0,
        help="Max stabilization torque (N*m) per axis",
    )
    parser.add_argument(
        "--imu-stab-mode",
        type=str,
        default="both",
        choices=("thrusters", "torque", "both"),
        help="IMU stabilization mode: thrusters | torque | both (default: both)",
    )
    parser.add_argument(
        "--imu-stab-ki",
        type=float,
        default=0.6,
        help="IMU stabilization integral gain",
    )
    parser.add_argument(
        "--imu-stab-int-max",
        type=float,
        default=0.6,
        help="Max integral term (rad*s)",
    )
    parser.add_argument(
        "--depth-hold",
        action="store_true",
        help="Enable simple depth hold (z-axis hold) using vertical thrusters",
    )
    parser.add_argument(
        "--depth-hold-target-z",
        type=float,
        default=None,
        help="Target world z depth for hold (default: first measured depth)",
    )
    parser.add_argument(
        "--depth-hold-kp",
        type=float,
        default=3.5,
        help="Depth hold proportional gain",
    )
    parser.add_argument(
        "--depth-hold-kd",
        type=float,
        default=1.2,
        help="Depth hold derivative gain",
    )
    parser.add_argument(
        "--depth-hold-ki",
        type=float,
        default=0.05,
        help="Depth hold integral gain",
    )
    parser.add_argument(
        "--depth-hold-int-max",
        type=float,
        default=0.6,
        help="Depth hold integral clamp",
    )
    parser.add_argument(
        "--depth-hold-cmd-max",
        type=float,
        default=0.75,
        help="Max normalized depth-hold command before combining with user heave",
    )
    parser.add_argument(
        "--depth-hold-user-deadband",
        type=float,
        default=0.05,
        help="If abs(user heave) is below this, depth hold can be applied",
    )
    parser.add_argument(
        "--depth-hold-deadband",
        type=float,
        default=0.02,
        help="Deadband on depth error (m) for depth hold",
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="sim_real",
        help="Simulation profile name (see --list-profiles)",
    )
    parser.add_argument(
        "--profile-file",
        type=str,
        default=str(PROFILE_PATH),
        help="Simulation profile JSON path",
    )
    parser.add_argument(
        "--thruster-perf-file",
        type=str,
        default=str(THRUSTER_PERF_PATH),
        help="PWM-thrust performance curve JSON file",
    )
    parser.add_argument(
        "--thruster-voltage",
        type=float,
        default=None,
        help="Select nearest thrust curve voltage from performance file (ex. 10,12,14,16,18,20). If omitted, use profile value.",
    )
    parser.add_argument(
        "--buoyancy-scale",
        type=float,
        default=None,
        help="Override profile buoyancy scale at runtime (ex. 0.98 for weaker buoyancy).",
    )
    parser.add_argument(
        "--disable-thruster-perf",
        action="store_true",
        help="Force linear thruster mapping and ignore performance curve JSON",
    )
    parser.add_argument(
        "--list-profiles",
        action="store_true",
        help="Print available simulation profiles and exit",
    )
    parser.add_argument(
        "--ros2",
        action="store_true",
        help="Enable ROS2 transport topics (/cmd_vel, /imu/data, /dvl/*)",
    )
    parser.add_argument(
        "--ros2-images",
        action="store_true",
        help="Publish stereo camera images and camera_info over ROS2",
    )
    parser.add_argument(
        "--ros2-image-width",
        type=int,
        default=640,
        help="Stereo image width for ROS2 image topics",
    )
    parser.add_argument(
        "--ros2-image-height",
        type=int,
        default=360,
        help="Stereo image height for ROS2 image topics",
    )
    parser.add_argument(
        "--ros2-sensor-hz",
        type=float,
        default=120.0,
        help="ROS2 IMU/DVL publish rate (Hz)",
    )
    parser.add_argument(
        "--ros2-image-hz",
        type=float,
        default=10.0,
        help="ROS2 stereo image publish rate (Hz)",
    )
    parser.add_argument(
        "--ros2-camera-calib-left",
        type=str,
        default="",
        help="Path to left camera calibration YAML (camera_info format)",
    )
    parser.add_argument(
        "--ros2-camera-calib-right",
        type=str,
        default="",
        help="Path to right camera calibration YAML (camera_info format)",
    )
    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Enable ArduPilot SITL JSON bridge (sends IMU/Pose)",
    )
    parser.add_argument(
        "--sitl-ip",
        type=str,
        default="127.0.0.1",
        help="ArduPilot SITL JSON interface IP",
    )
    parser.add_argument(
        "--sitl-port",
        type=int,
        default=9002,
        help="ArduPilot SITL JSON servo recv/listen Port (ArduPilot sends servo packets here)",
    )
    parser.add_argument(
        "--sitl-send-port",
        type=int,
        default=9003,
        help="ArduPilot SITL JSON sensor send Port (ArduPilot expects JSON sensor packets here)",
    )
    parser.add_argument(
        "--sitl-servo-source",
        type=str,
        default="json",
        choices=("json", "mavlink"),
        help="Source for SITL servo PWM input to MuJoCo (default: json).",
    )
    parser.add_argument(
        "--sitl-mavlink-endpoint",
        type=str,
        default="udpin:0.0.0.0:14660",
        help="MAVLink endpoint to receive SERVO_OUTPUT_RAW when --sitl-servo-source mavlink.",
    )
    parser.add_argument(
        "--sitl-mavlink-servo-hz",
        type=float,
        default=20.0,
        help="Requested SERVO_OUTPUT_RAW rate over MAVLink (Hz).",
    )
    parser.add_argument(
        "--sitl-mavlink-target-sysid",
        type=int,
        default=0,
        help="Target vehicle sysid expected for SERVO_OUTPUT_RAW (0=auto from heartbeat).",
    )
    parser.add_argument(
        "--sitl-mavlink-target-compid",
        type=int,
        default=0,
        help="Target vehicle compid expected for SERVO_OUTPUT_RAW (0=auto from heartbeat).",
    )
    parser.add_argument(
        "--sitl-mavlink-source-sysid",
        type=int,
        default=200,
        help="Source sysid for MuJoCo MAVLink listener (use non-255 to avoid GCS collision).",
    )
    parser.add_argument(
        "--sitl-mavlink-source-compid",
        type=int,
        default=190,
        help="Source component id for MuJoCo MAVLink listener.",
    )
    parser.add_argument(
        "--sitl-rc-ch-forward",
        type=int,
        default=1,
        help="SITL RC channel index for forward/pitch axis (0-based).",
    )
    parser.add_argument(
        "--sitl-rc-ch-lateral",
        type=int,
        default=0,
        help="SITL RC channel index for lateral/roll axis (0-based).",
    )
    parser.add_argument(
        "--sitl-rc-ch-throttle",
        type=int,
        default=2,
        help="SITL RC channel index for heave/throttle axis (0-based).",
    )
    parser.add_argument(
        "--sitl-rc-ch-yaw",
        type=int,
        default=3,
        help="SITL RC channel index for yaw axis (0-based).",
    )
    parser.add_argument(
        "--sitl-forward-sign",
        type=float,
        default=1.0,
        help="Sign multiplier for SITL forward axis input.",
    )
    parser.add_argument(
        "--sitl-lateral-sign",
        type=float,
        default=1.0,
        help="Sign multiplier for SITL lateral axis input.",
    )
    parser.add_argument(
        "--sitl-yaw-sign",
        type=float,
        default=1.0,
        help="Sign multiplier for SITL yaw axis input.",
    )
    parser.add_argument(
        "--sitl-heave-sign",
        type=float,
        default=1.0,
        help="Sign multiplier for SITL heave axis input.",
    )
    parser.add_argument(
        "--sitl-cmd-scale",
        type=float,
        default=1.0,
        help="Scale factor for SITL command amplitude (0.0~1.0 recommended).",
    )
    parser.add_argument(
        "--sitl-command-debug",
        action="store_true",
        help="Print SITL channel->command conversion diagnostics.",
    )
    parser.add_argument(
        "--sitl-direct-thrusters",
        dest="sitl_direct_thrusters",
        action="store_true",
        help="Use ArduSub servo PWM as direct per-thruster commands (recommended in --sitl).",
    )
    parser.add_argument(
        "--no-sitl-direct-thrusters",
        dest="sitl_direct_thrusters",
        action="store_false",
        help="Disable direct per-thruster mapping and use legacy axis remapping.",
    )
    parser.add_argument(
        "--sitl-servo-map",
        type=str,
        default="yaw_rr,yaw_lr,yaw_rf,yaw_lf,ver_lf,ver_rf,ver_lr,ver_rr",
        help="Comma-separated thruster names mapped from SITL servo outputs 1..N, or 'auto' for mixer-inverse mode.",
    )
    parser.add_argument(
        "--sitl-servo-signs",
        type=str,
        default="1,1,1,1,1,1,1,1",
        help="Comma-separated sign multipliers for --sitl-servo-map entries, or 'auto'.",
    )
    parser.add_argument(
        "--sitl-servo-scale",
        type=float,
        default=0.75,
        help="Scale applied to direct-thruster normalized command from SITL PWM.",
    )
    parser.add_argument(
        "--thruster-loop-hz",
        type=float,
        default=60.0,
        help="Thruster force update rate (Hz), decoupled from physics timestep.",
    )
    parser.add_argument(
        "--sitl-roll-scale",
        type=float,
        default=0.45,
        help="Additional scale for roll command in SITL mixer-inverse mode.",
    )
    parser.add_argument(
        "--sitl-pitch-scale",
        type=float,
        default=0.45,
        help="Additional scale for pitch command in SITL mixer-inverse mode.",
    )
    parser.add_argument(
        "--sitl-mixer-frame",
        type=str,
        default="auto",
        choices=("auto", "vectored", "vectored_6dof"),
        help="Frame for SITL mixer-inverse mode when --sitl-servo-map=auto (default: auto detect).",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run real-time simulation loop without GLFW viewer",
    )
    parser.add_argument(
        "--allow-mavros-rc-in-sitl",
        action="store_true",
        help="Allow /mavros/rc/override input even in --sitl mode (default: disabled in SITL)",
    )
    parser.set_defaults(sitl_direct_thrusters=True)
    args = parser.parse_args()

    # Profile values are externalized so users can retune dynamics without code edits.
    default_profiles = {
        "sim_fast": {
            "half_height": 0.147,
            "buoyancy_scale": 1.002,
            "cob_torque_scale": 1.0,
            "buoyancy_point_blend": 1.0,
            "align_com_to_thruster_plane": False,
            "cob_y_offset": 0.0,
            "thruster_voltage": 20.0,
            "thruster_force_max": 65.0,
            "linear_drag": 0.95,
            "angular_drag": 0.10,
            "spin_gain": 26.0,
            "validation_timestep": 0.004,
            "validation_iterations": 20,
            "validation_ls_iterations": 8,
            "validation_step_start": 0.5,
            "validation_step_end": 1.4,
            "validation_total_time": 3.2,
            "validation_step_amp_ratio": 0.35,
        },
        "sim_real": {
            "half_height": 0.147,
            "buoyancy_scale": 0.992,
            "cob_torque_scale": 1.0,
            "buoyancy_point_blend": 1.0,
            "align_com_to_thruster_plane": False,
            "cob_y_offset": -0.010,
            "thruster_voltage": 16.0,
            "thruster_force_max": 21.0,
            "linear_drag": 1.30,
            "angular_drag": 0.32,
            "spin_gain": 22.0,
            "validation_timestep": 0.005,
            "validation_iterations": 20,
            "validation_ls_iterations": 8,
            "validation_step_start": 0.5,
            "validation_step_end": 1.4,
            "validation_total_time": 3.2,
            "validation_step_amp_ratio": 0.30,
        },
        "sim_hover": {
            "half_height": 0.147,
            "buoyancy_scale": 1.002,
            "cob_torque_scale": 1.05,
            "buoyancy_point_blend": 1.0,
            "align_com_to_thruster_plane": True,
            "cob_y_offset": -0.015,
            "thruster_voltage": 16.0,
            "thruster_force_max": 62.0,
            "linear_drag": 0.78,
            "angular_drag": 0.14,
            "spin_gain": 22.0,
            "validation_timestep": 0.005,
            "validation_iterations": 20,
            "validation_ls_iterations": 8,
            "validation_step_start": 0.5,
            "validation_step_end": 1.4,
            "validation_total_time": 3.2,
            "validation_step_amp_ratio": 0.25,
        },
    }
    profile_path = Path(args.profile_file).expanduser()
    if not profile_path.exists():
        profile_path.write_text(json.dumps(default_profiles, indent=2))
    profiles = dict(default_profiles)
    try:
        payload = json.loads(profile_path.read_text())
        if isinstance(payload, dict):
            for name, cfg in payload.items():
                if not isinstance(cfg, dict):
                    continue
                merged = dict(profiles.get(name, {}))
                merged.update(cfg)
                profiles[name] = merged
    except json.JSONDecodeError:
        print(f"[profile] invalid json: {profile_path}, using built-in defaults", flush=True)

    if args.list_profiles:
        print("Available profiles:", flush=True)
        for name in sorted(profiles):
            print(f"  - {name}", flush=True)
        return

    if args.profile not in profiles:
        print(f"[profile] unknown profile: {args.profile}", flush=True)
        print("Available profiles:", flush=True)
        for name in sorted(profiles):
            print(f"  - {name}", flush=True)
        raise SystemExit(2)

    sim_profile = dict(profiles[args.profile])
    print(f"[profile] using '{args.profile}' from {profile_path}", flush=True)
    if args.buoyancy_scale is not None:
        sim_profile["buoyancy_scale"] = float(np.clip(args.buoyancy_scale, 0.0, 2.0))
        print(
            f"[profile] override buoyancy_scale={sim_profile['buoyancy_scale']:.3f}",
            flush=True,
        )
    profile_thruster_voltage = float(sim_profile.get("thruster_voltage", 20.0))
    if args.thruster_voltage is None:
        active_thruster_voltage = profile_thruster_voltage
        print(
            f"[profile] using thruster_voltage={active_thruster_voltage:.1f}V from profile",
            flush=True,
        )
    else:
        active_thruster_voltage = float(args.thruster_voltage)
        print(
            f"[profile] override thruster_voltage={active_thruster_voltage:.1f}V (profile {profile_thruster_voltage:.1f}V)",
            flush=True,
        )

    # Optional thruster PWM->force profile.
    perf_cfg = {
        "active": False,
        "requested_voltage": float(active_thruster_voltage),
        "selected_voltage": None,
        "pwm": np.array([], dtype=np.float64),
        "force": np.array([], dtype=np.float64),
    }

    def _to_float_array(values) -> np.ndarray | None:
        if not isinstance(values, list) or not values:
            return None
        out = []
        for value in values:
            try:
                out.append(float(value))
            except (TypeError, ValueError):
                return None
        return np.array(out, dtype=np.float64)

    def _normalize_thruster_perf_voltage(value: float | str | None) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(active_thruster_voltage)

    def _load_thruster_performance(path: Path) -> None:
        perf_path = path.expanduser()
        if not perf_path.exists():
            print(f"[thruster perf] file not found: {perf_path}", flush=True)
            return
        try:
            payload = json.loads(perf_path.read_text())
        except (OSError, json.JSONDecodeError):
            print(f"[thruster perf] invalid json: {perf_path}", flush=True)
            return

        curves_raw = payload.get("curves") if isinstance(payload, dict) else None
        if not isinstance(curves_raw, list):
            print(f"[thruster perf] missing curves in: {perf_path}", flush=True)
            return

        candidates = []
        for curve in curves_raw:
            if not isinstance(curve, dict):
                continue
            voltage = curve.get("voltage_v")
            pwm = _to_float_array(curve.get("pwm_us"))
            force = _to_float_array(curve.get("force_n"))
            if voltage is None or pwm is None or force is None:
                continue
            if pwm.size != force.size:
                continue
            if pwm.size < 2:
                continue
            order = np.argsort(pwm)
            pwm = pwm[order]
            force = force[order]
            valid = np.isfinite(pwm) & np.isfinite(force)
            if not np.any(valid):
                continue
            candidates.append(
                {
                    "voltage": float(voltage),
                    "pwm": pwm[valid],
                    "force": force[valid],
                }
            )

        if not candidates:
            print(f"[thruster perf] no usable curve in: {perf_path}", flush=True)
            return

        requested = _normalize_thruster_perf_voltage(active_thruster_voltage)
        selected = min(candidates, key=lambda item: abs(item["voltage"] - requested))
        perf_cfg.update(
            {
                "active": True,
                "requested_voltage": requested,
                "selected_voltage": float(selected["voltage"]),
                "pwm": selected["pwm"],
                "force": selected["force"],
            }
        )
        print(
            f"[thruster perf] loaded curve {selected['voltage']}V from {perf_path} "
            f"(requested {requested}V)",
            flush=True,
        )

    if not args.disable_thruster_perf:
        _load_thruster_performance(Path(args.thruster_perf_file).expanduser())

    def pwm_to_force_from_perf(norm_cmd: float) -> float:
        pwm = float(np.clip(norm_cmd, -1.0, 1.0) * 400.0 + 1500.0)
        return float(np.interp(pwm, perf_cfg["pwm"], perf_cfg["force"]))

    # Load model/state once and reuse for runtime, validation, and calibration paths.
    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    # Initialize derived state once before runtime loops so launch start poses,
    # sensor readings, and depth-hold reference use valid base position.
    mujoco.mj_forward(model, data)

    # Identify base body
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_id < 0:
        raise SystemExit("[runtime] base_link body not found in model")

    # Build body-child table once so subtree mass can be computed robustly.
    body_children = [[] for _ in range(model.nbody)]
    for body_idx in range(1, model.nbody):
        parent_idx = int(model.body_parentid[body_idx])
        if 0 <= parent_idx < model.nbody:
            body_children[parent_idx].append(body_idx)

    def body_subtree_mass(root_body_id: int) -> float:
        """Return total mass of the given body and all descendants."""
        if not (0 <= root_body_id < model.nbody):
            return 0.0
        total = 0.0
        stack = [int(root_body_id)]
        while stack:
            bid = stack.pop()
            total += float(model.body_mass[bid])
            stack.extend(body_children[bid])
        return total

    # Actuator indices
    act = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
        for i in range(model.nu)
    }
    ctrlrange = model.actuator_ctrlrange.copy()
    tune_path = Path(__file__).resolve().parent / "thruster_tune.json"
    thruster_params_path = Path(__file__).resolve().parent / "thruster_params.json"

    # Shared command state. ROS2 and SITL handlers write into this state.
    cmd = {"forward": 0.0, "heave": 0.0, "yaw": 0.0, "sway": 0.0}
    state = {"step": 2.0, "max": 15.0}
    cmd_lock = threading.Lock()
    stop_event = threading.Event()
    paused_flag = {"value": False}
    viewer_control_mode = {"value": True}
    show_thruster_labels = {"value": True}
    follow_camera = {"value": False}
    follow_camera_init = {"value": False}
    show_sensor_overlay = {"value": True}
    camera_mode = {"value": "free"}  # free | follow | stereo_left | stereo_right

    def clamp(value: float, max_val: float) -> float:
        return max(-max_val, min(max_val, value))

    def toggle_pause() -> None:
        paused_flag["value"] = not paused_flag["value"]

    def toggle_mode() -> None:
        viewer_control_mode["value"] = not viewer_control_mode["value"]

    def toggle_thruster_labels() -> None:
        show_thruster_labels["value"] = not show_thruster_labels["value"]

    def toggle_follow_camera() -> None:
        follow_camera["value"] = not follow_camera["value"]
        # Re-apply follow defaults each time follow mode is enabled.
        if follow_camera["value"]:
            follow_camera_init["value"] = False
            camera_mode["value"] = "follow"
        else:
            camera_mode["value"] = "free"

    def toggle_sensor_overlay() -> None:
        show_sensor_overlay["value"] = not show_sensor_overlay["value"]

    def apply_ros_cmd(forward: float, sway: float, yaw: float, heave: float) -> None:
        with cmd_lock:
            max_val = state["max"]
            cmd["forward"] = clamp(forward, max_val)
            cmd["sway"] = clamp(sway, max_val)
            cmd["yaw"] = clamp(yaw, max_val)
            cmd["heave"] = clamp(heave, max_val)
        # ROS2 command stream should directly drive the robot.
        viewer_control_mode["value"] = False

    ros_bridge = None
    try:
        from ros2_bridge import Ros2Bridge

        enable_ros2 = bool(args.ros2)
        enable_mavros_rc = True
        if args.sitl and not args.allow_mavros_rc_in_sitl:
            enable_mavros_rc = False
            if enable_ros2:
                print(
                    "[ros2] SITL mode: /mavros/rc/override input disabled to avoid external override conflicts.",
                    flush=True,
                )

        if args.sitl or enable_ros2:
            ros_bridge = Ros2Bridge(
                model=model,
                command_callback=apply_ros_cmd,
                cmd_limit=state["max"],
                publish_images=args.ros2_images,
                image_width=args.ros2_image_width,
                image_height=args.ros2_image_height,
                sensor_hz=args.ros2_sensor_hz,
                image_hz=args.ros2_image_hz,
                enable_mavros=enable_mavros_rc if enable_ros2 else False,
                enable_sitl=args.sitl,
                sitl_ip=args.sitl_ip,
                sitl_port=args.sitl_port,
                sitl_send_port=args.sitl_send_port,
                sitl_ch_forward=int(args.sitl_rc_ch_forward),
                sitl_ch_lateral=int(args.sitl_rc_ch_lateral),
                sitl_ch_throttle=int(args.sitl_rc_ch_throttle),
                sitl_ch_yaw=int(args.sitl_rc_ch_yaw),
                sitl_forward_sign=float(args.sitl_forward_sign),
                sitl_lateral_sign=float(args.sitl_lateral_sign),
                sitl_yaw_sign=float(args.sitl_yaw_sign),
                sitl_heave_sign=float(args.sitl_heave_sign),
                sitl_cmd_scale=float(args.sitl_cmd_scale),
                sitl_command_debug=bool(args.sitl_command_debug),
                sitl_servo_source=str(args.sitl_servo_source),
                sitl_mavlink_endpoint=str(args.sitl_mavlink_endpoint),
                sitl_mavlink_servo_hz=float(args.sitl_mavlink_servo_hz),
                sitl_mavlink_target_sysid=int(args.sitl_mavlink_target_sysid),
                sitl_mavlink_target_compid=int(args.sitl_mavlink_target_compid),
                sitl_mavlink_source_sysid=int(args.sitl_mavlink_source_sysid),
                sitl_mavlink_source_compid=int(args.sitl_mavlink_source_compid),
                camera_calib_left=args.ros2_camera_calib_left,
                camera_calib_right=args.ros2_camera_calib_right,
                enable_ros=enable_ros2,
            )
            if enable_ros2:
                viewer_control_mode["value"] = False
                input_topics = "/cmd_vel"
                if enable_mavros_rc:
                    input_topics += ", /mavros/rc/override"
                print(
                    f"[bridge] enabled: {input_topics} -> control, /imu/data, /dvl/velocity, /dvl/odometry, /dvl/altitude, /depth, /bar30/pressure_pa, /mujoco/ground_truth/pose"
                    + (", /stereo/*" if args.ros2_images else ""),
                    flush=True,
                )
            elif args.sitl:
                print(
                    "[bridge] SITL transport enabled "
                    f"(sensor=UDP JSON, servo={args.sitl_servo_source}, ROS2 disabled).",
                    flush=True,
                )
                viewer_control_mode["value"] = False
    except Exception as exc:
        if args.sitl:
            raise SystemExit(
                f"[runtime] --sitl initialization failed: {exc}"
            )
        if args.ros2:
            print(f"[ros2] bridge init failed: {exc}", flush=True)


    if args.sitl:
        print("[runtime] Local terminal/joystick control disabled in SITL mode (QGC remote control only).", flush=True)
    else:
        print("[runtime] Manual keyboard/joystick path removed. Use ROS2 /cmd_vel or SITL/QGC control.", flush=True)

    # Thruster sets
    yaw_names = ["yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr"]
    # Ordering for horizontal thrusters: 1=rf, 2=lf, 3=rr, 4=lr
    horiz_order = ["yaw_rf", "yaw_lf", "yaw_rr", "yaw_lr"]
    ver_names = ["ver_lf", "ver_lr", "ver_rf", "ver_rr"]

    def normalize(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            return v
        return v / n

    sensor_names = ["imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude"]
    sensor_ids = {}
    for sname in sensor_names:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
        if sid >= 0:
            sensor_ids[sname] = sid

    camera_ids = {}
    for cname in ("stereo_left", "stereo_right"):
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cname)
        if cid >= 0:
            camera_ids[cname] = cid

    sensor_site_ids = {
        "imu": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_site"),
        "bar30": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "bar30_site"),
        "dvl": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "dvl_site"),
        "cam_left": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cam_left_site"),
        "cam_right": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cam_right_site"),
    }

    def sensor_value(name: str) -> np.ndarray | None:
        sid = sensor_ids.get(name, -1)
        if sid < 0:
            return None
        adr = int(model.sensor_adr[sid])
        dim = int(model.sensor_dim[sid])
        return data.sensordata[adr : adr + dim].copy()

    def update_stabilization(dt: float) -> None:
        """Estimate tilt error and build body-frame stabilization torque."""
        if not imu_stabilize_active["value"]:
            imu_stab_cache["tau_body"][:] = 0.0
            imu_stab_cache["tau_world"][:] = 0.0
            return
        base_rot = data.xmat[base_id].reshape(3, 3)
        # Model body frame is FDR (x=forward, y=down, z=right). We regulate
        # roll(x), pitch(z), and yaw(y) when yaw command is not driving.
        body_down_world = base_rot @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        world_down = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        tilt_axis_world = np.cross(world_down, body_down_world)
        tilt_err_body = base_rot.T @ tilt_axis_world
        err_roll_pitch = np.array([tilt_err_body[0], tilt_err_body[2]], dtype=np.float64)
        imu_stab_int[:2] = np.clip(
            imu_stab_int[:2] + err_roll_pitch * max(1e-6, dt), -imu_stab_int_max, imu_stab_int_max
        )
        ang_vel_world = data.cvel[base_id, 0:3]
        ang_vel_body = base_rot.T @ ang_vel_world

        # When yaw command is nearly zero, apply a light damping on yaw angular rate
        # to suppress free-spinning drift that can appear with zero input.
        yaw_cmd_norm = float(yaw_cmd_for_stab["value"])
        yaw_damp = 0.0 if yaw_cmd_norm > 0.12 else -0.75 * imu_stab_kd * ang_vel_body[1]

        tau_body = np.array(
            [
                -imu_stab_kp * err_roll_pitch[0] - imu_stab_kd * ang_vel_body[0] - imu_stab_ki * imu_stab_int[0],
                yaw_damp,
                -imu_stab_kp * err_roll_pitch[1] - imu_stab_kd * ang_vel_body[2] - imu_stab_ki * imu_stab_int[1],
            ],
            dtype=np.float64,
        )
        tau_body = np.clip(tau_body, -imu_stab_max, imu_stab_max)
        imu_stab_cache["tau_body"] = tau_body
        imu_stab_cache["tau_world"] = base_rot @ tau_body

    def update_depth_hold(dt: float, user_heave_cmd: float) -> float:
        """Estimate depth error and generate normalized heave correction."""
        if (
            not depth_hold_active["value"]
            or dt <= 1e-9
            or not np.isfinite(depth_hold_kp)
            or not np.isfinite(depth_hold_kd)
            or not np.isfinite(depth_hold_ki)
        ):
            return 0.0

        if abs(float(user_heave_cmd)) > depth_hold_user_deadband:
            depth_hold_int["value"] = 0.0
            return 0.0

        if depth_hold_target_z["value"] is None:
            depth_hold_target_z["value"] = float(data.xipos[base_id][2]) if args.depth_hold_target_z is None else float(args.depth_hold_target_z)
        target_z = float(depth_hold_target_z["value"])
        if not np.isfinite(target_z):
            depth_hold_target_z["value"] = float(data.xipos[base_id][2])
            target_z = depth_hold_target_z["value"]

        err = float(data.xipos[base_id][2] - target_z)
        if abs(err) <= depth_hold_error_deadband:
            depth_hold_int["value"] *= 0.9
            err = 0.0
        else:
            depth_hold_int["value"] += err * dt

        depth_hold_int["value"] = float(
            np.clip(depth_hold_int["value"], -depth_hold_int_max, depth_hold_int_max)
        )

        # Coordinate convention:
        # world z-axis is up, and positive vertical thruster command pushes down.
        # z_err > 0 -> robot is too shallow, so push down (+);
        # z_err < 0 -> robot is too deep, so pull up (-).
        z_rate_world = float(data.cvel[base_id, 5])
        raw = depth_hold_kp * err + depth_hold_kd * z_rate_world + depth_hold_ki * depth_hold_int["value"]
        return float(np.clip(raw, -depth_hold_cmd_max, depth_hold_cmd_max))

    if args.sitl:
        if args.imu_stabilize or args.depth_hold:
            print(
                "[simulation] SITL mode: internal stabilization/depth-hold are disabled; "
                "control authority is delegated to ArduPilot.",
                flush=True,
            )
        args.imu_stabilize = False
        args.depth_hold = False

    imu_stabilize_active = {"value": bool(args.imu_stabilize)}
    imu_stab_kp = float(args.imu_stab_kp)
    imu_stab_kd = float(args.imu_stab_kd)
    imu_stab_max = float(args.imu_stab_max)
    imu_stab_mode = str(args.imu_stab_mode)
    imu_stab_ki = float(args.imu_stab_ki)
    imu_stab_int_max = float(args.imu_stab_int_max)
    imu_stab_int = np.zeros(3, dtype=np.float64)
    imu_stab_cache = {"tau_body": np.zeros(3, dtype=np.float64), "tau_world": np.zeros(3, dtype=np.float64)}
    yaw_cmd_for_stab = {"value": 0.0}

    depth_hold_active = {"value": bool(args.depth_hold)}
    depth_hold_target_z = {"value": None}
    depth_hold_int = {"value": 0.0}
    depth_hold_kp = float(args.depth_hold_kp)
    depth_hold_kd = float(args.depth_hold_kd)
    depth_hold_ki = float(args.depth_hold_ki)
    depth_hold_int_max = float(args.depth_hold_int_max)
    depth_hold_cmd_max = float(np.clip(args.depth_hold_cmd_max, 0.05, 1.0))
    depth_hold_user_deadband = float(args.depth_hold_user_deadband)
    depth_hold_error_deadband = float(args.depth_hold_deadband)
    depth_hold_target_z["value"] = args.depth_hold_target_z

    def ensure_tune_file() -> None:
        if tune_path.exists():
            return
        gears = {}
        for name in ver_names + yaw_names:
            gears[name] = model.actuator_gear[act[name], :3].tolist()
        payload = {"gears": gears}
        tune_path.write_text(json.dumps(payload, indent=2))

    def load_tuning() -> bool:
        if not tune_path.exists():
            return False
        try:
            payload = json.loads(tune_path.read_text())
        except json.JSONDecodeError:
            return False
        gears = payload.get("gears", {})
        changed = False
        for name, vec in gears.items():
            if name not in act:
                continue
            if not isinstance(vec, list) or len(vec) != 3:
                continue
            v = np.array(vec, dtype=np.float64)
            v = normalize(v)
            if np.linalg.norm(v) < 1e-9:
                continue
            if not np.allclose(model.actuator_gear[act[name], :3], v, atol=1e-6):
                model.actuator_gear[act[name], :3] = v
                changed = True
        return changed

    ensure_tune_file()
    last_tune_mtime = tune_path.stat().st_mtime if tune_path.exists() else 0.0
    load_tuning()
    cob_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cob_site")
    align_com_to_thruster_plane = bool(sim_profile.get("align_com_to_thruster_plane", False))
    cob_y_offset = float(sim_profile.get("cob_y_offset", 0.0))

    def align_com_to_horizontal_thruster_plane() -> None:
        ys = []
        for name in horiz_order:
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            if sid >= 0:
                ys.append(float(model.site_pos[sid][1]))
        if len(ys) < 2:
            return
        target_y = float(np.mean(ys))
        old_y = float(model.body_ipos[base_id][1])
        if abs(target_y - old_y) < 1e-5:
            return
        model.body_ipos[base_id][1] = target_y
        mujoco.mj_setConst(model, data)
        print(f"[model] aligned CoM.y: {old_y:.4f} -> {target_y:.4f}", flush=True)

    def align_cob_to_com_with_offset() -> None:
        if cob_site_id < 0:
            return
        target_y = float(model.body_ipos[base_id][1] + cob_y_offset)
        old_y = float(model.site_pos[cob_site_id][1])
        if abs(target_y - old_y) < 1e-6:
            return
        model.site_pos[cob_site_id][1] = target_y
        print(f"[model] aligned CoB.y: {old_y:.4f} -> {target_y:.4f} (offset={cob_y_offset:+.4f})", flush=True)

    if align_com_to_thruster_plane:
        align_com_to_horizontal_thruster_plane()
    align_cob_to_com_with_offset()

    # Geometry-aware horizontal allocator:
    # maps desired [forward, sway, yaw] in normalized units to 4 horizontal thruster commands.
    horiz_pinv = np.zeros((len(horiz_order), 3), dtype=np.float64)
    horiz_alloc = np.zeros((3, len(horiz_order)), dtype=np.float64)
    horiz_stab_yaw_scale = {"value": 40.0}

    def build_horizontal_allocator() -> None:
        nonlocal horiz_pinv, horiz_alloc
        com_body = model.body_ipos[base_id].copy()
        alloc = np.zeros((3, len(horiz_order)), dtype=np.float64)
        for i, name in enumerate(horiz_order):
            aid = act[name]
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            fdir = normalize(model.actuator_gear[aid, :3].copy())
            r = model.site_pos[sid].copy() - com_body
            tau = np.cross(r, fdir)
            # [Fx, Fz, My] in body frame
            alloc[:, i] = np.array([fdir[0], fdir[2], tau[1]], dtype=np.float64)

        row_scale = np.sum(np.abs(alloc), axis=1)
        row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
        horiz_pinv = np.linalg.pinv(alloc / row_scale[:, None])
        horiz_alloc = alloc

    def mix_horizontal_thrusters(fwd_cmd: float, sway_cmd: float, yaw_cmd: float) -> np.ndarray:
        """Map normalized [forward, sway, yaw] wrench into 4 horizontal thrusters."""
        wrench_cmd = np.array([fwd_cmd, sway_cmd, yaw_cmd], dtype=np.float64)
        u = horiz_pinv @ wrench_cmd
        max_abs = float(np.max(np.abs(u)))
        if max_abs > 1.0:
            u /= max_abs
        return np.clip(u, -1.0, 1.0)

    build_horizontal_allocator()

    # Water/fluid basics
    water_surface_z = 0.0
    rho = float(model.opt.density)
    g = abs(float(model.opt.gravity[2]))
    total_mass_all = float(np.sum(model.body_mass[1:]))
    vehicle_mass = body_subtree_mass(base_id)
    if vehicle_mass <= 1e-9:
        vehicle_mass = float(model.body_mass[base_id])
    neutral_volume = vehicle_mass / max(rho, 1e-6)
    if total_mass_all > vehicle_mass * 1.2:
        print(
            "[physics] buoyancy mass reference: "
            f"vehicle_subtree={vehicle_mass:.3f}kg (all_nonworld={total_mass_all:.3f}kg)",
            flush=True,
        )

    all_thruster_names = ver_names + yaw_names
    thr_state = {name: 0.0 for name in all_thruster_names}
    thr_target = {name: 0.0 for name in all_thruster_names}
    thruster_scale = {name: 1.0 for name in all_thruster_names}
    prop_phase = {name: 0.0 for name in all_thruster_names}
    prop_qpos_adr = {}
    prop_dof_adr = {}
    prop_spin_sign = {}
    for name in all_thruster_names:
        jname = f"prop_{name}_j"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            prop_qpos_adr[name] = int(model.jnt_qposadr[jid])
            prop_dof_adr[name] = int(model.jnt_dofadr[jid])
            # Alternate rotation direction for a more natural visual.
            prop_spin_sign[name] = 1.0 if name.endswith(("lf", "rr")) else -1.0

    sitl_direct_thrusters = bool(args.sitl and args.sitl_direct_thrusters)
    sitl_servo_cmd_norm = {name: 0.0 for name in all_thruster_names}
    sitl_servo_last_wall = {"value": -1.0}
    sitl_servo_timeout_s = 0.8
    sitl_servo_scale = float(np.clip(args.sitl_servo_scale, 0.0, 2.0))
    sitl_roll_scale = float(np.clip(args.sitl_roll_scale, 0.0, 2.0))
    sitl_pitch_scale = float(np.clip(args.sitl_pitch_scale, 0.0, 2.0))
    sitl_use_mixer_inverse = False
    sitl_mixer_cmd = {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "throttle": 0.0,
        "forward": 0.0,
        "lateral": 0.0,
    }
    sitl_mixer_frame_locked = {"value": None}
    sitl_mixer_last_cmd = {"value": None}
    sitl_mixer_last_log = {"value": -1.0}

    # ArduSub AP_Motors6DOF motor factors (roll, pitch, yaw, throttle, forward, lateral).
    sitl_motor_mix = {
        "vectored": np.array(
            [
                [0.0, 0.0, 1.0, 0.0, -1.0, 1.0],
                [0.0, 0.0, -1.0, 0.0, -1.0, -1.0],
                [0.0, 0.0, -1.0, 0.0, 1.0, 1.0],
                [0.0, 0.0, 1.0, 0.0, 1.0, -1.0],
                [1.0, 0.0, 0.0, -1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, -1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        ),
        "vectored_6dof": np.array(
            [
                [0.0, 0.0, 1.0, 0.0, -1.0, 1.0],
                [0.0, 0.0, -1.0, 0.0, -1.0, -1.0],
                [0.0, 0.0, -1.0, 0.0, 1.0, 1.0],
                [0.0, 0.0, 1.0, 0.0, 1.0, -1.0],
                [1.0, -1.0, 0.0, -1.0, 0.0, 0.0],
                [-1.0, -1.0, 0.0, -1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0, -1.0, 0.0, 0.0],
                [-1.0, 1.0, 0.0, -1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        ),
    }
    sitl_motor_mix_pinv = {
        name: np.linalg.pinv(mat) for name, mat in sitl_motor_mix.items()
    }

    def sitl_pwm_to_norm(pwm: int) -> float:
        # ArduSub bidirectional motor range: 1100..1900, neutral=1500.
        if pwm <= 0 or pwm == 65535:
            return 0.0
        return float(np.clip((float(pwm) - 1500.0) / 400.0, -1.0, 1.0))

    if sitl_direct_thrusters:
        servo_map_arg = str(args.sitl_servo_map).strip()
        sitl_use_mixer_inverse = servo_map_arg.lower() == "auto"

        if sitl_use_mixer_inverse:
            forced_frame = str(args.sitl_mixer_frame).strip().lower()

            def resolve_mixer_frame(pwm_values: list[int]) -> str:
                if sitl_mixer_frame_locked["value"] in ("vectored", "vectored_6dof"):
                    return str(sitl_mixer_frame_locked["value"])
                if forced_frame in ("vectored", "vectored_6dof"):
                    sitl_mixer_frame_locked["value"] = forced_frame
                    return forced_frame
                if len(pwm_values) >= 8:
                    p7 = int(pwm_values[6])
                    p8 = int(pwm_values[7])
                    # Do not permanently lock to vectored from startup zeros.
                    # ArduSub often publishes ch7/ch8 as 0 while disarmed and only
                    # drives valid PWM (1100..1900) after arm/mode transitions.
                    valid7 = 1000 <= p7 <= 2000
                    valid8 = 1000 <= p8 <= 2000
                    if valid7 and valid8:
                        return "vectored_6dof"
                # Conservative fallback until 7/8 are confirmed active.
                return "vectored"

            def on_sitl_servo_packet(pwm_values: list[int]) -> None:
                frame_name = resolve_mixer_frame(pwm_values)
                mix = sitl_motor_mix[frame_name]
                pinv = sitl_motor_mix_pinv[frame_name]
                motors = np.zeros(mix.shape[0], dtype=np.float64)
                n = min(len(pwm_values), mix.shape[0])
                for i in range(n):
                    motors[i] = sitl_pwm_to_norm(int(pwm_values[i]))
                cmd = pinv @ motors
                cmd = np.clip(
                    np.nan_to_num(cmd, nan=0.0, posinf=1.0, neginf=-1.0),
                    -1.0,
                    1.0,
                )
                sitl_mixer_cmd["roll"] = float(cmd[0])
                sitl_mixer_cmd["pitch"] = float(cmd[1])
                sitl_mixer_cmd["yaw"] = float(cmd[2])
                sitl_mixer_cmd["throttle"] = float(cmd[3])
                sitl_mixer_cmd["forward"] = float(cmd[4])
                # ArduSub lateral(+right) and this model's sway axis are opposite.
                sitl_mixer_cmd["lateral"] = float(-cmd[5])
                sitl_servo_last_wall["value"] = time.monotonic()
                if args.sitl_command_debug:
                    now = time.monotonic()
                    packed = (
                        frame_name,
                        round(sitl_mixer_cmd["forward"], 3),
                        round(sitl_mixer_cmd["lateral"], 3),
                        round(sitl_mixer_cmd["yaw"], 3),
                        round(sitl_mixer_cmd["throttle"], 3),
                        round(sitl_mixer_cmd["roll"], 3),
                        round(sitl_mixer_cmd["pitch"], 3),
                    )
                    if packed != sitl_mixer_last_cmd["value"] and (now - sitl_mixer_last_log["value"]) > 0.15:
                        print(
                            "[sitl] mixer decode "
                            f"[{frame_name}] fwd={sitl_mixer_cmd['forward']:+.3f} "
                            f"lat={sitl_mixer_cmd['lateral']:+.3f} yaw={sitl_mixer_cmd['yaw']:+.3f} "
                            f"thr={sitl_mixer_cmd['throttle']:+.3f} "
                            f"roll={sitl_mixer_cmd['roll']:+.3f} pitch={sitl_mixer_cmd['pitch']:+.3f}",
                            flush=True,
                        )
                        sitl_mixer_last_cmd["value"] = packed
                        sitl_mixer_last_log["value"] = now

            if ros_bridge is not None:
                ros_bridge.set_sitl_servo_handler(on_sitl_servo_packet, axis_control=False)
            print(
                "[sitl] mixer-inverse mode enabled: "
                f"frame={args.sitl_mixer_frame}, servo-scale={sitl_servo_scale:.2f}, "
                f"roll-scale={sitl_roll_scale:.2f}, pitch-scale={sitl_pitch_scale:.2f}",
                flush=True,
            )
        else:
            raw_map = [token.strip() for token in servo_map_arg.split(",") if token.strip()]
            if not raw_map:
                raise SystemExit("[runtime] --sitl-servo-map is empty")
            if len(raw_map) > 16:
                raise SystemExit("[runtime] --sitl-servo-map supports up to 16 channels")
            for thr_name in raw_map:
                if thr_name not in all_thruster_names:
                    raise SystemExit(
                        f"[runtime] invalid thruster in --sitl-servo-map: {thr_name} "
                        f"(valid: {','.join(all_thruster_names)})"
                    )

            servo_sign_arg = str(args.sitl_servo_signs).strip().lower()
            if servo_sign_arg == "auto":
                servo_signs = [1.0 for _ in raw_map]
            else:
                sign_tokens = [token.strip() for token in str(args.sitl_servo_signs).split(",") if token.strip()]
                if not sign_tokens:
                    servo_signs = [1.0 for _ in raw_map]
                elif len(sign_tokens) == 1:
                    servo_signs = [float(sign_tokens[0]) for _ in raw_map]
                elif len(sign_tokens) == len(raw_map):
                    servo_signs = [float(token) for token in sign_tokens]
                else:
                    raise SystemExit(
                        "[runtime] --sitl-servo-signs must have 1 value or match --sitl-servo-map length"
                    )
            servo_signs = [float(np.sign(v)) if abs(float(v)) > 1e-9 else 1.0 for v in servo_signs]

            def on_sitl_servo_packet(pwm_values: list[int]) -> None:
                for thr_name in all_thruster_names:
                    sitl_servo_cmd_norm[thr_name] = 0.0
                for idx, thr_name in enumerate(raw_map):
                    if idx >= len(pwm_values):
                        break
                    norm = sitl_pwm_to_norm(int(pwm_values[idx])) * servo_signs[idx]
                    sitl_servo_cmd_norm[thr_name] = float(np.clip(norm, -1.0, 1.0))
                sitl_servo_last_wall["value"] = time.monotonic()

            if ros_bridge is not None:
                ros_bridge.set_sitl_servo_handler(on_sitl_servo_packet, axis_control=False)
            print(
                "[sitl] direct thruster mode enabled: "
                + ", ".join(
                    f"ch{idx + 1}->{thr_name}*{servo_signs[idx]:+0.0f}"
                    for idx, thr_name in enumerate(raw_map)
                )
                + f", servo-scale={sitl_servo_scale:.2f}",
                flush=True,
            )
    elif args.sitl:
        print("[sitl] using legacy axis remapping mode (not direct-thruster).", flush=True)

    def ensure_thruster_params_file() -> None:
        if thruster_params_path.exists():
            return
        payload = {
            "global": {
                "deadzone": 0.05,
                "tau_up": 0.12,
                "tau_down": 0.18,
                "reverse_asymmetry": 0.75,
                "forward_poly": [0.0, 3.5, 7.0, 12.0],
                "reverse_poly": [0.0, 2.8, 5.5, 9.5],
                "command_limit": 0.65,
            },
            "per_thruster": {name: {"gain_scale": 1.0} for name in all_thruster_names},
        }
        thruster_params_path.write_text(json.dumps(payload, indent=2))

    def load_thruster_params() -> bool:
        for name in all_thruster_names:
            thruster_scale[name] = 1.0
        if not thruster_params_path.exists():
            return False
        try:
            payload = json.loads(thruster_params_path.read_text())
        except json.JSONDecodeError:
            return False
        changed = False
        per_thruster = payload.get("per_thruster", {})
        if not isinstance(per_thruster, dict):
            return False
        for name, cfg in per_thruster.items():
            if name not in thruster_scale:
                continue
            if not isinstance(cfg, dict):
                continue
            gain = cfg.get("gain_scale", 1.0)
            if not isinstance(gain, (int, float)):
                continue
            new_gain = float(np.clip(float(gain), 0.4, 1.8))
            if abs(new_gain - thruster_scale[name]) > 1e-8:
                changed = True
            thruster_scale[name] = new_gain
        return changed

    ensure_thruster_params_file()
    load_thruster_params()

    # Simple underwater model (v1.1-like): no current/waves/turbulence,
    # direct thruster commands, buoyancy + linear damping only.
    half_height = float(sim_profile.get("half_height", 0.147))
    buoyancy_scale = float(sim_profile.get("buoyancy_scale", 1.0))
    cob_torque_scale = float(sim_profile.get("cob_torque_scale", 1.0))
    buoyancy_point_blend = float(np.clip(sim_profile.get("buoyancy_point_blend", 1.0), 0.0, 1.0))
    thruster_force_max = float(sim_profile.get("thruster_force_max", 50.0))
    if perf_cfg.get("active") and perf_cfg.get("force").size > 0:
        perf_max = float(np.max(np.abs(perf_cfg["force"])) )
        if perf_max > 0.0:
            thruster_force_max = perf_max
    linear_drag = float(sim_profile.get("linear_drag", 1.2))
    angular_drag = float(sim_profile.get("angular_drag", 0.12))
    # Keep tiny damping in air; full damping only when submerged.
    air_linear_drag = float(
        np.clip(sim_profile.get("air_linear_drag", linear_drag * 0.03), 0.0, linear_drag)
    )
    air_angular_drag = float(
        np.clip(sim_profile.get("air_angular_drag", angular_drag * 0.05), 0.0, angular_drag)
    )
    spin_gain = float(sim_profile.get("spin_gain", 22.0))
    thruster_loop_hz = float(np.clip(args.thruster_loop_hz, 1.0, 500.0))
    thruster_loop_dt = 1.0 / thruster_loop_hz
    next_thruster_sim_time = {"value": -1.0}

    def thruster_update_due() -> bool:
        sim_t = float(data.time)
        if next_thruster_sim_time["value"] < 0.0:
            next_thruster_sim_time["value"] = sim_t
        if sim_t + 1e-9 < next_thruster_sim_time["value"]:
            return False
        while sim_t + 1e-9 >= next_thruster_sim_time["value"]:
            next_thruster_sim_time["value"] += thruster_loop_dt
        return True

    print(
        f"[runtime] thruster loop rate: {thruster_loop_hz:.1f} Hz "
        f"(physics dt={float(model.opt.timestep):.4f}s)",
        flush=True,
    )
    print(
        "[physics] buoyancy setup: "
        f"scale={buoyancy_scale:.3f}, mass={vehicle_mass:.3f}kg, "
        f"rho={rho:.1f}, neutral_volume={neutral_volume:.5f}m^3, half_height={half_height:.3f}",
        flush=True,
    )

    # Vertical allocators for roll/pitch control via vertical thrusters.
    # - vert_pinv: physical torque-domain inverse (used by IMU stabilization)
    # - vert_att_pinv: normalized command-domain inverse (used by SITL mixer decode)
    vert_pinv = np.zeros((len(ver_names), 2), dtype=np.float64)
    vert_att_pinv = np.zeros((len(ver_names), 2), dtype=np.float64)

    def build_vertical_allocator() -> None:
        nonlocal vert_pinv, vert_att_pinv
        com_body = model.body_ipos[base_id].copy()
        alloc = np.zeros((2, len(ver_names)), dtype=np.float64)
        for i, name in enumerate(ver_names):
            aid = act[name]
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            if sid < 0:
                continue
            fdir = normalize(model.actuator_gear[aid, :3].copy())
            r = model.site_pos[sid].copy() - com_body
            tau = np.cross(r, fdir * thruster_force_max)
            # Map roll (x) and pitch (z) torque.
            alloc[:, i] = np.array([tau[0], tau[2]], dtype=np.float64)
        if alloc.shape[1] > 0:
            vert_pinv = np.linalg.pinv(alloc)
            row_scale = np.sum(np.abs(alloc), axis=1)
            row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
            vert_att_pinv = np.linalg.pinv(alloc / row_scale[:, None])

    build_vertical_allocator()

    def mix_vertical_attitude_cmd(roll_cmd: float, pitch_cmd: float) -> np.ndarray:
        """Map normalized roll/pitch commands into vertical thruster commands."""
        if not vert_att_pinv.size:
            return np.zeros(len(ver_names), dtype=np.float64)
        cmd = np.array([roll_cmd, pitch_cmd], dtype=np.float64)
        u = vert_att_pinv @ cmd
        max_abs = float(np.max(np.abs(u))) if u.size else 0.0
        if max_abs > 1.0:
            u = u / max_abs
        return np.clip(u, -1.0, 1.0)

    def update_horizontal_stab_gain() -> None:
        """Recompute normalized yaw correction gain from allocator and thrust limit."""
        if not horiz_alloc.size or not horiz_order:
            horiz_stab_yaw_scale["value"] = 1.0
            return
        yaw_basis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        u = horiz_pinv @ yaw_basis
        tau_per_unit = float(np.dot(horiz_alloc[2], u) * thruster_force_max)
        if abs(tau_per_unit) < 1e-6:
            tau_per_unit = 1.0
        horiz_stab_yaw_scale["value"] = abs(tau_per_unit)

    update_horizontal_stab_gain()

    def apply_stabilization_thrusters() -> None:
        if not (imu_stabilize_active["value"] and imu_stab_mode in ("thrusters", "both")):
            return
        tau_cmd = np.array([imu_stab_cache["tau_body"][0], imu_stab_cache["tau_body"][2]], dtype=np.float64)
        u = vert_pinv @ tau_cmd if vert_pinv.size else np.zeros(len(ver_names))
        u = np.clip(u, -0.5, 0.5)
        for i, name in enumerate(ver_names):
            thr_target[name] = float(np.clip(thr_target[name] + u[i], -1.0, 1.0))

        # Keep yaw stable when user command is near zero (thrusters-only mode).
        if imu_stab_mode == "thrusters":
            yaw_torque = float(imu_stab_cache["tau_body"][1])
            if abs(yaw_torque) > 1e-6:
                yaw_cmd = np.clip(
                    -yaw_torque / horiz_stab_yaw_scale["value"],
                    -0.5,
                    0.5,
                )
                yaw_u = mix_horizontal_thrusters(0.0, 0.0, float(yaw_cmd))
                for i, name in enumerate(horiz_order):
                    thr_target[name] = float(np.clip(thr_target[name] + yaw_u[i], -1.0, 1.0))

    # Debug/validation cache
    last_flow_world = np.zeros(3, dtype=np.float64)
    last_buoy_force = np.zeros(3, dtype=np.float64)
    last_buoy_point = np.zeros(3, dtype=np.float64)

    def update_thruster_forces(_dt: float) -> None:
        for name in all_thruster_names:
            thr_state[name] = thr_target[name]
            aid = act[name]
            lo, hi = ctrlrange[aid]
            gain = float(thruster_scale.get(name, 1.0))
            target_norm = float(np.clip(thr_target[name], -1.0, 1.0))
            if perf_cfg.get("active") and perf_cfg.get("force").size > 0:
                force = pwm_to_force_from_perf(target_norm) * gain
            else:
                force = target_norm * thruster_force_max * gain
            data.ctrl[aid] = float(np.clip(force, lo, hi))

    def update_propeller_visuals(dt: float) -> None:
        # Visual-only propeller spin (no reaction torque applied to vehicle).
        for name in all_thruster_names:
            qadr = prop_qpos_adr.get(name)
            dadr = prop_dof_adr.get(name)
            if qadr is None or dadr is None:
                continue
            omega = prop_spin_sign.get(name, 1.0) * float(data.ctrl[act[name]]) * spin_gain
            if dt > 0.0:
                prop_phase[name] += omega * dt
            data.qpos[qadr] = prop_phase[name]
            data.qvel[dadr] = 0.0

    def apply_underwater_wrench(_dt: float) -> None:
        """Apply simplified underwater forces: buoyancy + damping + optional torque."""
        nonlocal last_flow_world, last_buoy_force, last_buoy_point

        data.xfrc_applied[base_id, :] = 0.0
        com = data.xipos[base_id].copy()
        cob = data.site_xpos[cob_site_id].copy() if cob_site_id >= 0 else com
        buoy_point = ((1.0 - buoyancy_point_blend) * com) + (buoyancy_point_blend * cob)

        # Partial-submersion buoyancy based on base COM depth.
        depth = water_surface_z - float(com[2])
        frac = np.clip((depth + half_height) / (2.0 * half_height), 0.0, 1.0)
        buoy = rho * g * neutral_volume * frac * buoyancy_scale
        buoy_force_world = np.array([0.0, 0.0, buoy], dtype=np.float64)
        submerged = float(frac)

        # MuJoCo xfrc_applied layout: [force_xyz, torque_xyz].
        data.xfrc_applied[base_id, 0:3] += buoy_force_world
        if abs(cob_torque_scale) > 1e-9:
            r = buoy_point - com
            buoy_tau_world = np.cross(r, buoy_force_world) * cob_torque_scale
            data.xfrc_applied[base_id, 3:6] += buoy_tau_world

        # Blend damping by immersion: near-surface transitions are smooth.
        ang_vel = data.cvel[base_id, 0:3]
        lin_vel = data.cvel[base_id, 3:6]
        lin_drag_coeff = air_linear_drag + (linear_drag - air_linear_drag) * submerged
        ang_drag_coeff = air_angular_drag + (angular_drag - air_angular_drag) * submerged
        data.xfrc_applied[base_id, 0:3] -= lin_drag_coeff * lin_vel
        data.xfrc_applied[base_id, 3:6] -= ang_drag_coeff * ang_vel

        if imu_stabilize_active["value"] and imu_stab_mode in ("torque", "both"):
            data.xfrc_applied[base_id, 3:6] += imu_stab_cache["tau_world"]

        last_flow_world = np.zeros(3, dtype=np.float64)
        last_buoy_force = buoy_force_world
        last_buoy_point = buoy_point

    def viewer_key_callback(keycode):
        # GLFW keycodes for SPACE is 32
        if keycode == 32:
            paused_flag["value"] = not paused_flag["value"]
        if keycode in (77, 109):  # M or m
            viewer_control_mode["value"] = not viewer_control_mode["value"]
        if keycode in (76, 108):  # L or l
            show_thruster_labels["value"] = not show_thruster_labels["value"]
        if keycode in (73, 105):  # I or i
            toggle_sensor_overlay()
        if keycode in (67, 99):  # C or c
            toggle_follow_camera()
        if keycode == 49:  # 1
            camera_mode["value"] = "stereo_left"
            follow_camera["value"] = False
        if keycode == 50:  # 2
            camera_mode["value"] = "stereo_right"
            follow_camera["value"] = False
        if keycode == 48:  # 0
            camera_mode["value"] = "free"
            follow_camera["value"] = False

    def publish_ros_once() -> None:
        nonlocal ros_bridge
        if ros_bridge is None:
            return
        try:
            ros_bridge.publish(data)
        except Exception as exc:
            print(f"[ros2] publish failed, disabling bridge: {exc}", flush=True)
            ros_bridge.shutdown()
            ros_bridge = None

    def run_step(is_paused: bool, publish_ros: bool = True) -> tuple[float, float, float, float]:
        """Run one control + physics + publish cycle."""
        nonlocal ros_bridge, last_tune_mtime
        if ros_bridge is not None:
            try:
                ros_bridge.spin_once()
            except Exception as exc:
                print(f"[ros2] spin_once failed, disabling bridge: {exc}", flush=True)
                ros_bridge.shutdown()
                ros_bridge = None

        thruster_due = thruster_update_due()

        if sitl_direct_thrusters:
            now = time.monotonic()
            stale = (
                sitl_servo_last_wall["value"] <= 0.0
                or (now - sitl_servo_last_wall["value"]) > sitl_servo_timeout_s
            )
            if stale:
                for name in all_thruster_names:
                    thr_target[name] = 0.0
            else:
                if sitl_use_mixer_inverse:
                    fwd_cmd = float(
                        np.clip(sitl_mixer_cmd["forward"] * sitl_servo_scale, -1.0, 1.0)
                    )
                    sway_cmd = float(
                        np.clip(sitl_mixer_cmd["lateral"] * sitl_servo_scale, -1.0, 1.0)
                    )
                    yaw_cmd = float(
                        np.clip(sitl_mixer_cmd["yaw"] * sitl_servo_scale, -1.0, 1.0)
                    )
                    # ArduSub throttle factor is negative in vectored frames,
                    # while this model's positive vertical command pushes down.
                    heave_cmd = float(
                        np.clip(-sitl_mixer_cmd["throttle"] * sitl_servo_scale, -1.0, 1.0)
                    )
                    roll_cmd = float(
                        np.clip(
                            sitl_mixer_cmd["roll"] * sitl_servo_scale * sitl_roll_scale,
                            -1.0,
                            1.0,
                        )
                    )
                    pitch_cmd = float(
                        np.clip(
                            sitl_mixer_cmd["pitch"] * sitl_servo_scale * sitl_pitch_scale,
                            -1.0,
                            1.0,
                        )
                    )
                    horiz_cmd = mix_horizontal_thrusters(fwd_cmd, sway_cmd, yaw_cmd)
                    for name in all_thruster_names:
                        thr_target[name] = 0.0
                    for i, name in enumerate(horiz_order):
                        thr_target[name] = float(horiz_cmd[i])
                    for name in ver_names:
                        thr_target[name] = heave_cmd
                    vert_att_cmd = mix_vertical_attitude_cmd(roll_cmd, pitch_cmd)
                    for i, name in enumerate(ver_names):
                        thr_target[name] = float(np.clip(thr_target[name] + vert_att_cmd[i], -1.0, 1.0))
                    yaw_cmd_for_stab["value"] = abs(yaw_cmd)
                else:
                    for name in all_thruster_names:
                        thr_target[name] = float(
                            np.clip(sitl_servo_cmd_norm.get(name, 0.0) * sitl_servo_scale, -1.0, 1.0)
                        )
                    yaw_cmd_for_stab["value"] = 0.0
            if stale:
                yaw_cmd_for_stab["value"] = 0.0
            thr_dt = model.opt.timestep if not is_paused else 0.0
            if thruster_due:
                update_thruster_forces(thr_dt)
            update_propeller_visuals(thr_dt)
            apply_underwater_wrench(model.opt.timestep if not is_paused else 0.0)
            if not is_paused:
                mujoco.mj_step(model, data)
            if publish_ros:
                publish_ros_once()
            return 0.0, 0.0, 0.0, 0.0

        if not viewer_control_mode["value"]:
            # Remote commands (ROS2/MAVROS)
            with cmd_lock:
                forward = cmd["forward"]
                heave = cmd["heave"]
                yaw = cmd["yaw"]
                sway = cmd["sway"]

            # Reload tuning if file changed
            if tune_path.exists():
                mtime = tune_path.stat().st_mtime
                if mtime > last_tune_mtime:
                    if load_tuning():
                        build_horizontal_allocator()
                    last_tune_mtime = mtime

            # Terminal commands are normalized to [-1, 1] before thruster dynamics.
            cmd_scale = max(state["max"], 1e-6)
            fwd_cmd = np.clip(forward / cmd_scale, -1.0, 1.0)
            sway_cmd = np.clip(sway / cmd_scale, -1.0, 1.0)
            yaw_cmd = np.clip(yaw / cmd_scale, -1.0, 1.0)
            depth_hold_cmd = update_depth_hold(model.opt.timestep, heave)
            heave_cmd = np.clip((heave + depth_hold_cmd * cmd_scale) / cmd_scale, -1.0, 1.0)
            yaw_cmd_for_stab["value"] = abs(float(yaw_cmd))

            horiz_cmd = mix_horizontal_thrusters(fwd_cmd, sway_cmd, yaw_cmd)

            for name in all_thruster_names:
                thr_target[name] = 0.0
            for name in ver_names:
                thr_target[name] = heave_cmd
            for i, name in enumerate(horiz_order):
                thr_target[name] = float(horiz_cmd[i])
            if imu_stabilize_active["value"]:
                update_stabilization(model.opt.timestep)
            apply_stabilization_thrusters()

            thr_dt = model.opt.timestep if not is_paused else 0.0
            if thruster_due:
                update_thruster_forces(thr_dt)
            update_propeller_visuals(thr_dt)
        else:
            forward = 0.0
            yaw = 0.0
            heave = 0.0
            sway = 0.0
            yaw_cmd_for_stab["value"] = 0.0
            if imu_stabilize_active["value"]:
                update_stabilization(model.opt.timestep)
            depth_hold_cmd = update_depth_hold(model.opt.timestep, 0.0)
            depth_hold_cmd_for_thrusters = depth_hold_cmd
            for name in all_thruster_names:
                thr_target[name] = 0.0
            if imu_stabilize_active["value"] and imu_stab_mode in ("thrusters", "both"):
                apply_stabilization_thrusters()
            for name in ver_names:
                thr_target[name] = float(np.clip(thr_target[name] + depth_hold_cmd_for_thrusters, -1.0, 1.0))
            thr_dt = model.opt.timestep if not is_paused else 0.0
            if thruster_due:
                update_thruster_forces(thr_dt)
            update_propeller_visuals(thr_dt)

        apply_underwater_wrench(model.opt.timestep if not is_paused else 0.0)

        if not is_paused:
            mujoco.mj_step(model, data)
        if publish_ros:
            publish_ros_once()
        return forward, sway, yaw, heave

    if args.headless:
        print("[runtime] headless mode enabled: running without GLFW viewer", flush=True)
        target_dt = float(max(model.opt.timestep, 1e-6))
        next_wall = time.perf_counter()
        while not stop_event.is_set():
            is_paused = paused_flag["value"]
            run_step(is_paused)
            # Keep real-time pacing without accumulating extra delay from
            # compute time (important for SITL stabilization responsiveness).
            next_wall += target_dt
            now_wall = time.perf_counter()
            sleep_s = next_wall - now_wall
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_wall = now_wall
        if ros_bridge is not None:
            ros_bridge.shutdown()
        stop_event.set()
        return

    with mujoco.viewer.launch_passive(model, data, key_callback=viewer_key_callback) as viewer:
        has_set_texts = hasattr(viewer, "set_texts")
        target_dt = float(max(model.opt.timestep, 1e-6))
        next_step_wall = time.perf_counter()
        sensor_hz = float(max(args.ros2_sensor_hz, 1.0))
        sensor_dt = 1.0 / sensor_hz
        next_sensor_wall = time.perf_counter()
        max_catchup_steps = max(4, int(round(0.10 / target_dt)))
        max_sensor_catchup = max(2, int(round(0.10 / sensor_dt)))
        forward = 0.0
        sway = 0.0
        yaw = 0.0
        heave = 0.0
        while viewer.is_running() and not stop_event.is_set():
            # Respect GUI pause in passive viewer
            paused = False
            if hasattr(viewer, "is_paused"):
                flag = viewer.is_paused
                paused = flag() if callable(flag) else bool(flag)
            is_paused = paused_flag["value"] or paused
            now_wall = time.perf_counter()
            if is_paused:
                forward, sway, yaw, heave = run_step(True, publish_ros=False)
                next_step_wall = now_wall + target_dt
                next_sensor_wall = now_wall + sensor_dt
            else:
                step_count = 0
                while now_wall >= next_step_wall and step_count < max_catchup_steps:
                    forward, sway, yaw, heave = run_step(False, publish_ros=False)
                    next_step_wall += target_dt
                    step_count += 1
                    now_wall = time.perf_counter()
                if step_count >= max_catchup_steps and now_wall >= next_step_wall:
                    next_step_wall = now_wall + target_dt

                sensor_count = 0
                while ros_bridge is not None and now_wall >= next_sensor_wall and sensor_count < max_sensor_catchup:
                    publish_ros_once()
                    next_sensor_wall += sensor_dt
                    sensor_count += 1
                    now_wall = time.perf_counter()
                if sensor_count >= max_sensor_catchup and now_wall >= next_sensor_wall:
                    next_sensor_wall = now_wall + sensor_dt

            # Debug arrows for thruster directions (world frame)
            with viewer.lock():
                user_scn = viewer.user_scn
                user_scn.ngeom = 0

                base_rot = data.xmat[base_id].reshape(3, 3)
                com = data.xipos[base_id].copy()

                # Camera control: fixed stereo, follow, or free.
                if camera_mode["value"] in camera_ids:
                    cam = viewer.cam
                    cam.type = int(mujoco.mjtCamera.mjCAMERA_FIXED)
                    cam.fixedcamid = int(camera_ids[camera_mode["value"]])
                    cam.trackbodyid = -1
                elif follow_camera["value"]:
                    cam = viewer.cam
                    cam.type = int(mujoco.mjtCamera.mjCAMERA_TRACKING)
                    cam.trackbodyid = int(base_id)
                    if not follow_camera_init["value"]:
                        cam.distance = 2.2
                        cam.elevation = -20.0
                        cam.azimuth = 135.0
                        follow_camera_init["value"] = True
                else:
                    if int(viewer.cam.type) in (
                        int(mujoco.mjtCamera.mjCAMERA_TRACKING),
                        int(mujoco.mjtCamera.mjCAMERA_FIXED),
                    ):
                        viewer.cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
                        viewer.cam.fixedcamid = -1
                        viewer.cam.trackbodyid = -1

                def add_arrow(start, direction, magnitude, rgba, thickness=0.02):
                    if user_scn.ngeom >= user_scn.maxgeom:
                        return
                    length = 0.08 + 0.01 * abs(magnitude)
                    end = start + direction * length
                    geom = user_scn.geoms[user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_ARROW,
                        np.zeros(3),
                        np.zeros(3),
                        np.eye(3).flatten(),
                        np.array(rgba, dtype=np.float32),
                    )
                    mujoco.mjv_connector(
                        geom,
                        mujoco.mjtGeom.mjGEOM_ARROW,
                        thickness,
                        start,
                        end,
                    )
                    user_scn.ngeom += 1

                def add_sphere(position, radius, rgba):
                    if user_scn.ngeom >= user_scn.maxgeom:
                        return
                    geom = user_scn.geoms[user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([radius, 0.0, 0.0]),
                        position,
                        np.eye(3).flatten(),
                        np.array(rgba, dtype=np.float32),
                    )
                    user_scn.ngeom += 1

                def add_bubble_stream(start, exhaust_dir, thrust_mag):
                    if thrust_mag < 2.0:
                        return
                    d = normalize(exhaust_dir)
                    up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
                    if abs(float(np.dot(d, up))) > 0.9:
                        up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                    s1 = normalize(np.cross(d, up))
                    s2 = normalize(np.cross(d, s1))
                    strength = min(1.0, thrust_mag / max(thruster_force_max, 1e-6))
                    count = 3 + int(3 * strength)
                    for k in range(count):
                        phase = (float(data.time) * 2.4 + k * 0.31) % 1.0
                        dist = 0.04 + phase * (0.22 + 0.08 * strength)
                        swirl = 0.008 * (1.0 - phase) * (0.6 + 0.4 * strength)
                        wobble = np.sin(2.0 * np.pi * (phase + 0.17 * k))
                        wobble2 = np.cos(2.0 * np.pi * (phase + 0.11 * k))
                        pos = start + d * dist + s1 * (swirl * wobble) + s2 * (swirl * wobble2)
                        radius = 0.004 + 0.003 * strength * (1.0 - 0.5 * phase)
                        alpha = 0.35 * (1.0 - phase)
                        add_sphere(pos, radius, (0.82, 0.93, 1.0, alpha))

                def add_label(text, position, rgba):
                    if user_scn.ngeom >= user_scn.maxgeom:
                        return
                    geom = user_scn.geoms[user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_LABEL,
                        np.zeros(3),
                        position,
                        np.eye(3).flatten(),
                        np.array(rgba, dtype=np.float32),
                    )
                    geom.size[:] = np.array([0.03, 0.03, 0.03])
                    geom.label = text
                    user_scn.ngeom += 1

                # Thruster force vectors (blue): direction includes the force sign.
                for name in ver_names + yaw_names:
                    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
                    start = data.site_xpos[sid].copy()
                    force = float(data.ctrl[act[name]])
                    fdir = model.actuator_gear[act[name], :3]
                    fdir = fdir / (np.linalg.norm(fdir) + 1e-9)
                    world_dir = base_rot @ fdir
                    draw_dir = world_dir if force >= 0.0 else -world_dir
                    add_arrow(start, draw_dir, abs(force), (0.2, 0.6, 1.0, 1.0))
                    add_bubble_stream(start, -draw_dir, abs(force))
                    if show_thruster_labels["value"]:
                        add_label(
                            f"{name}:{force:+.1f}",
                            start + np.array([0.0, 0.03, 0.0]),
                            (0.8, 0.9, 1.0, 1.0),
                        )

                # Sensor and camera markers
                if show_sensor_overlay["value"]:
                    imu_id = sensor_site_ids.get("imu", -1)
                    bar30_id = sensor_site_ids.get("bar30", -1)
                    dvl_id = sensor_site_ids.get("dvl", -1)
                    cam_l_id = sensor_site_ids.get("cam_left", -1)
                    cam_r_id = sensor_site_ids.get("cam_right", -1)
                    if imu_id >= 0:
                        pos = data.site_xpos[imu_id].copy()
                        add_sphere(pos, 0.025, (1.0, 1.0, 0.1, 1.0))
                        add_label("IMU", pos + np.array([0.0, 0.03, 0.0]), (1.0, 1.0, 0.3, 1.0))
                    if bar30_id >= 0:
                        pos = data.site_xpos[bar30_id].copy()
                        add_sphere(pos, 0.022, (0.2, 0.8, 1.0, 1.0))
                        add_label("BAR30", pos + np.array([0.0, 0.03, 0.0]), (0.3, 0.9, 1.0, 1.0))
                    if dvl_id >= 0:
                        pos = data.site_xpos[dvl_id].copy()
                        add_sphere(pos, 0.025, (0.1, 1.0, 1.0, 1.0))
                        add_label("DVL", pos + np.array([0.0, 0.03, 0.0]), (0.3, 1.0, 1.0, 1.0))
                    if cam_l_id >= 0:
                        pos = data.site_xpos[cam_l_id].copy()
                        add_sphere(pos, 0.022, (1.0, 0.1, 1.0, 1.0))
                        add_label("CAM_L", pos + np.array([0.0, 0.03, 0.0]), (1.0, 0.3, 1.0, 1.0))
                    if cam_r_id >= 0:
                        pos = data.site_xpos[cam_r_id].copy()
                        add_sphere(pos, 0.022, (1.0, 0.5, 0.1, 1.0))
                        add_label("CAM_R", pos + np.array([0.0, 0.03, 0.0]), (1.0, 0.6, 0.2, 1.0))

                # Net thrust (red) and buoyancy (green) at CoB
                net_force = np.zeros(3)
                for name in ver_names + yaw_names:
                    fdir = model.actuator_gear[act[name], :3]
                    fdir = fdir / (np.linalg.norm(fdir) + 1e-9)
                    net_force += (base_rot @ fdir) * data.ctrl[act[name]]
                net_mag = float(np.linalg.norm(net_force))
                if net_mag > 1e-6:
                    add_arrow(com, net_force / net_mag, net_mag, (0.95, 0.95, 0.95, 1.0))
                    add_label("NET", com + np.array([0.0, 0.05, 0.0]), (0.95, 0.95, 0.95, 1.0))

                buoy_mag = float(np.linalg.norm(last_buoy_force))
                if buoy_mag > 1e-6:
                    add_arrow(last_buoy_point, np.array([0.0, 0.0, 1.0]), buoy_mag * 0.05, (0.2, 1.0, 0.2, 1.0))
                    add_label("BUOY", last_buoy_point + np.array([0.0, 0.08, 0.0]), (0.6, 1.0, 0.6, 1.0))

            # On-screen help (if available)
            if has_set_texts:
                overlay_line = "Space: pause, M: mode, C: follow, 1/2: stereo cam, 0: free, I: sensors"
                if show_sensor_overlay["value"]:
                    imu_g = sensor_value("imu_gyro")
                    imu_a = sensor_value("imu_acc")
                    dvl_v = sensor_value("dvl_vel_body")
                    dvl_alt = sensor_value("dvl_altitude")
                    if imu_g is not None and imu_a is not None and dvl_v is not None and dvl_alt is not None:
                        overlay_line = (
                            f"IMU gyro[{imu_g[0]:+.2f},{imu_g[1]:+.2f},{imu_g[2]:+.2f}] "
                            f"acc[{imu_a[0]:+.2f},{imu_a[1]:+.2f},{imu_a[2]:+.2f}] "
                            f"DVL vel[{dvl_v[0]:+.2f},{dvl_v[1]:+.2f},{dvl_v[2]:+.2f}] alt={dvl_alt[0]:.2f}m"
                        )
                viewer.set_texts([
                    (
                        None,
                        None,
                        f"Mode: {'viewer' if viewer_control_mode['value'] else 'terminal'} | fwd {forward:+.1f} sway {sway:+.1f} yaw {yaw:+.1f} heave {heave:+.1f} | Cam: {camera_mode['value']}",
                        overlay_line,
                    )
                ])

            viewer.sync()
            # Passive viewer sync already handles GUI/event pacing. Additional
            # fixed sleep here increases control/sensor latency for SITL.

    if ros_bridge is not None:
        ros_bridge.shutdown()
    stop_event.set()


if __name__ == "__main__":
    main()
