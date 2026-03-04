#!/bin/bash
# Launch MuJoCo competition simulation
# Usage:
#   ./launch_competition_sim.sh [--headless] [--sitl] [--images] [--ros2] [--force-clean]
#   ./launch_competition_sim.sh --sitl --images --calib-left calibration/left.yaml --calib-right calibration/right.yaml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
VIDEO_BRIDGE_SCRIPT="${SCRIPT_DIR}/scripts/ros2_to_qgc_video.py"
VIDEO_BRIDGE_PID=""

# Parse arguments
HEADLESS=false
IMAGES=""
ROS2_REQUESTED=false
HEADLESS_ARG=""
SITL_ARG=""
SITL_PORT="9002"
SITL_SEND_PORT="9003"
EXTRA_ARGS=()
PROFILE=""
HOVER_STABLE_REQUESTED=false
FORCE_CLEAN=false
SITL_MAVLINK_TARGET_SYSID=1
SITL_MAVLINK_TARGET_COMPID=1
SITL_MAVLINK_SOURCE_SYSID=200
SITL_MAVLINK_SOURCE_COMPID=190
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=true
            shift
            ;;
        --images)
            IMAGES="--ros2-images"
            ROS2_REQUESTED=true
            shift
            ;;
        --sitl)
            SITL_ARG="--sitl"
            shift
            ;;
        --ros2)
            ROS2_REQUESTED=true
            EXTRA_ARGS+=("--ros2")
            shift
            ;;
        --sitl-port)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --sitl-port"
                exit 2
            fi
            SITL_PORT="$2"
            shift 2
            ;;
        --sitl-send-port)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --sitl-send-port"
                exit 2
            fi
            SITL_SEND_PORT="$2"
            shift 2
            ;;
        --hover-stable)
            HOVER_STABLE_REQUESTED=true
            # Backward-compatible alias for legacy non-SITL behavior.
            # In SITL, stability is delegated to ArduPilot/QGC.
            if [[ -n "$SITL_ARG" ]]; then
                echo "[launch] Note: --hover-stable is ignored in --sitl mode."
            else
                EXTRA_ARGS+=("--depth-hold" "--imu-stabilize")
                PROFILE="sim_hover"
            fi
            shift
            ;;
        --force-clean)
            FORCE_CLEAN=true
            shift
            ;;
        --profile)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --profile"
                exit 2
            fi
            PROFILE="$2"
            EXTRA_ARGS+=("--profile" "$2")
            shift 2
            ;;
        --thruster-voltage)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --thruster-voltage"
                exit 2
            fi
            EXTRA_ARGS+=("--thruster-voltage" "$2")
            shift 2
            ;;
        --calib-left)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --calib-left"
                exit 2
            fi
            EXTRA_ARGS+=("--ros2-camera-calib-left" "$2")
            shift 2
            ;;
        --calib-right)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --calib-right"
                exit 2
            fi
            EXTRA_ARGS+=("--ros2-camera-calib-right" "$2")
            shift 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

extra_arg_present() {
    local needle="$1"
    local token
    for token in "${EXTRA_ARGS[@]}"; do
        if [[ "$token" == "$needle" || "$token" == "$needle="* ]]; then
            return 0
        fi
    done
    return 1
}

get_arg_value() {
    local needle="$1"
    local token idx next_idx
    for idx in "${!EXTRA_ARGS[@]}"; do
        token="${EXTRA_ARGS[$idx]}"
        if [[ "$token" == "$needle="* ]]; then
            printf "%s" "${token#*=}"
            return 0
        fi
        if [[ "$token" == "$needle" ]]; then
            next_idx=$((idx + 1))
            if (( next_idx < ${#EXTRA_ARGS[@]} )); then
                printf "%s" "${EXTRA_ARGS[$next_idx]}"
            fi
            return 0
        fi
    done
    return 1
}

collect_existing_mujoco_pids() {
    pgrep -f "run_urdf_full.py --scene competition_scene.xml" || true
}

kill_pid_list() {
    local pids="$1"
    [[ -z "$pids" ]] && return 0
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null || true
    sleep 0.6
    local alive=""
    # shellcheck disable=SC2086
    for pid in $pids; do
        if kill -0 "$pid" 2>/dev/null; then
            alive+=" $pid"
        fi
    done
    if [[ -n "$alive" ]]; then
        # shellcheck disable=SC2086
        kill -9 $alive 2>/dev/null || true
    fi
}

EXISTING_MJ_PIDS="$(collect_existing_mujoco_pids | xargs)"
if [[ -n "$EXISTING_MJ_PIDS" ]]; then
    if [[ "$FORCE_CLEAN" == true ]]; then
        echo "[launch] --force-clean: stopping existing MuJoCo runtimes:$EXISTING_MJ_PIDS"
        kill_pid_list "$EXISTING_MJ_PIDS"
    else
        echo "[error] existing MuJoCo runtime detected:$EXISTING_MJ_PIDS"
        echo "        Stop old process first or rerun with --force-clean."
        echo "        Example: pkill -f 'run_urdf_full.py --scene competition_scene.xml'"
        exit 1
    fi
fi

# Set display for headless mode
if [ "$HEADLESS" = true ]; then
    HEADLESS_ARG="--headless"
    export DISPLAY=""
    export MUJOCO_GL=egl
    echo "[launch] Running in headless mode (EGL rendering)"
fi

echo "[launch] Starting MuJoCo UUV Competition Simulation"
echo "[launch] Scene: competition_scene.xml"
if [ "$ROS2_REQUESTED" = true ] && [[ "$HEADLESS" == false ]]; then
    echo "[launch] Source ROS2 environment for --ros2 topics and stereo image transport."
    source /opt/ros/humble/setup.bash
fi
echo "[launch] Bridge:"
if [ "$ROS2_REQUESTED" = true ]; then
    echo "  ROS2 Transport: enabled"
    echo "    Input:  /cmd_vel, /mavros/rc/override"
    echo "    Output: /imu/data, /dvl/velocity, /dvl/odometry, /dvl/altitude"
    echo "    Debug:  /mujoco/ground_truth/pose"
    if [ -n "$IMAGES" ]; then
        echo "    Images: /stereo/left/image_raw, /stereo/right/image_raw"
    fi
else
    echo "  ROS2 Transport: disabled (SITL UDP JSON only)"
fi
if [[ -n "$SITL_ARG" ]]; then
    echo "[launch] SITL UDP JSON:"
    echo "  listen port  = ${SITL_PORT}  (ArduPilot --sim-port-out, where servo packets are sent)"
    echo "  send target = ${SITL_SEND_PORT}  (ArduPilot --sim-port-in, where sensor packets are sent)"
fi
echo ""

if [[ -n "$SITL_ARG" ]]; then
    if [[ "$SITL_PORT" == "$SITL_SEND_PORT" ]]; then
        echo "[warn] --sitl-port and --sitl-send-port are identical (${SITL_PORT}); SITL usually requires distinct direction-specific UDP ports."
    fi
    # Intentionally do not force extra stabilization/depth-hold for SITL.
    # ArduPilot/QGC depth and attitude modes should own these loops.
    if [[ -z "$PROFILE" ]]; then
        PROFILE="sim_real"
        EXTRA_ARGS+=(--profile "$PROFILE")
    fi
    # Raise SITL sensor feed rate by default to reduce EKF lag in stabilize mode.
    if ! extra_arg_present "--ros2-sensor-hz"; then
        EXTRA_ARGS+=(--ros2-sensor-hz 300)
        echo "[launch] SITL mode: overriding sensor publish rate to 300 Hz (use --ros2-sensor-hz to set manually)."
    fi
    if ! extra_arg_present "--thruster-loop-hz"; then
        EXTRA_ARGS+=(--thruster-loop-hz 60)
        echo "[launch] SITL mode: overriding thruster loop rate to 60 Hz (use --thruster-loop-hz to set manually)."
    fi
    # Use MAVLink as the default servo source for the standard QGC control path.
    if ! extra_arg_present "--sitl-servo-source"; then
        EXTRA_ARGS+=(--sitl-servo-source mavlink)
        echo "[launch] SITL mode: defaulting servo source to mavlink."
    else
        if [[ "$(get_arg_value --sitl-servo-source)" == "json" ]]; then
            echo "[launch] SITL mode: using servo source json."
        elif [[ "$(get_arg_value --sitl-servo-source)" == "mavlink" ]]; then
            echo "[launch] SITL mode: using servo source mavlink (SERVO_OUTPUT_RAW)."
        else
            echo "[launch] SITL mode: custom servo source '$(get_arg_value --sitl-servo-source)'."
        fi
    fi
    if ! extra_arg_present "--sitl-mavlink-target-sysid"; then
        EXTRA_ARGS+=(--sitl-mavlink-target-sysid "${SITL_MAVLINK_TARGET_SYSID}")
    fi
    if ! extra_arg_present "--sitl-mavlink-target-compid"; then
        EXTRA_ARGS+=(--sitl-mavlink-target-compid "${SITL_MAVLINK_TARGET_COMPID}")
    fi
    if ! extra_arg_present "--sitl-mavlink-source-sysid"; then
        EXTRA_ARGS+=(--sitl-mavlink-source-sysid "${SITL_MAVLINK_SOURCE_SYSID}")
    fi
    if ! extra_arg_present "--sitl-mavlink-source-compid"; then
        EXTRA_ARGS+=(--sitl-mavlink-source-compid "${SITL_MAVLINK_SOURCE_COMPID}")
    fi
    if ! extra_arg_present "--sitl-mavlink-endpoint"; then
        EXTRA_ARGS+=(--sitl-mavlink-endpoint "udpin:0.0.0.0:14660")
    fi
    if ! extra_arg_present "--sitl-mavlink-servo-hz"; then
        EXTRA_ARGS+=(--sitl-mavlink-servo-hz 20)
    fi
    if ! extra_arg_present "--sitl-servo-map"; then
        # Direct-thruster mapping aligned with run_urdf_full.py defaults and
        # launch_stack_auto.sh so every launch path uses identical directions.
        EXTRA_ARGS+=(--sitl-servo-map "yaw_rr,yaw_lr,yaw_rf,yaw_lf,ver_lf,ver_rf,ver_lr,ver_rr")
    fi
    if ! extra_arg_present "--sitl-servo-signs"; then
        # Keep all thruster signs positive to avoid launch-path-dependent axis flips.
        EXTRA_ARGS+=(--sitl-servo-signs=1,1,1,1,1,1,1,1)
    fi
    if ! extra_arg_present "--sitl-servo-scale"; then
        EXTRA_ARGS+=(--sitl-servo-scale "0.40")
    fi
    # Lower input latency defaults for SITL command loops.
    : "${ROS2_UUV_CMD_DEADBAND:=0.005}"
    : "${ROS2_UUV_CMD_SLEW_RATE:=45.0}"
    : "${ROS2_UUV_CMD_TIMEOUT_S:=0.45}"
    : "${ROS2_UUV_SITL_MAVLINK_TIMEOUT_S:=1.5}"
    export ROS2_UUV_CMD_DEADBAND ROS2_UUV_CMD_SLEW_RATE ROS2_UUV_CMD_TIMEOUT_S ROS2_UUV_SITL_MAVLINK_TIMEOUT_S
    echo "[launch] SITL mode: cmd filter deadband=${ROS2_UUV_CMD_DEADBAND}, slew=${ROS2_UUV_CMD_SLEW_RATE}/s, timeout=${ROS2_UUV_CMD_TIMEOUT_S}s, mavlink_timeout=${ROS2_UUV_SITL_MAVLINK_TIMEOUT_S}s"
fi

if [ "$ROS2_REQUESTED" = true ] && ! extra_arg_present "--ros2"; then
    EXTRA_ARGS+=(--ros2)
fi

if [[ "$HOVER_STABLE_REQUESTED" == true && -n "$SITL_ARG" && -n "$PROFILE" && "$PROFILE" == "sim_hover" ]]; then
    PROFILE="sim_real"
    for idx in "${!EXTRA_ARGS[@]}"; do
        if [[ "${EXTRA_ARGS[$idx]}" == "--profile" ]]; then
            if ((idx + 1 < ${#EXTRA_ARGS[@]})); then
                unset 'EXTRA_ARGS[idx]'
                unset 'EXTRA_ARGS[idx+1]'
                break
            fi
        fi
    done
    EXTRA_ARGS=("${EXTRA_ARGS[@]}")
    EXTRA_ARGS+=(--profile "$PROFILE")
    echo "[launch] Note: --hover-stable + --sitl combination uses sim_real with no internal stabilization."
fi

if [[ -n "$PROFILE" && "$PROFILE" != "sim_real" ]]; then
    echo "[launch] Simulation profile: $PROFILE"
fi

cleanup_video_bridge() {
    if [[ -n "$VIDEO_BRIDGE_PID" ]] && kill -0 "$VIDEO_BRIDGE_PID" 2>/dev/null; then
        kill "$VIDEO_BRIDGE_PID" 2>/dev/null || true
    fi
}
trap cleanup_video_bridge EXIT INT TERM

if [[ -n "$SITL_ARG" && -n "$IMAGES" && "${AG_DISABLE_QGC_VIDEO_BRIDGE:-0}" != "1" ]]; then
    if [[ -f "$VIDEO_BRIDGE_SCRIPT" ]]; then
        QGC_VIDEO_TOPIC="${QGC_VIDEO_TOPIC:-/stereo/left/image_raw}"
        QGC_VIDEO_HOST="${QGC_VIDEO_HOST:-127.0.0.1}"
        QGC_VIDEO_PORT="${QGC_VIDEO_PORT:-5600}"
        QGC_VIDEO_FPS="${QGC_VIDEO_FPS:-15}"
        QGC_VIDEO_BITRATE_KBPS="${QGC_VIDEO_BITRATE_KBPS:-2600}"
        QGC_VIDEO_WIDTH="${QGC_VIDEO_WIDTH:-640}"
        QGC_VIDEO_HEIGHT="${QGC_VIDEO_HEIGHT:-360}"
        QGC_VIDEO_LOG="${QGC_VIDEO_LOG:-/tmp/uuv_qgc_video_bridge.log}"
        echo "[launch] auto video bridge: ${QGC_VIDEO_TOPIC} -> udp://${QGC_VIDEO_HOST}:${QGC_VIDEO_PORT}"
        (
            python3 "$VIDEO_BRIDGE_SCRIPT" \
                --topic "$QGC_VIDEO_TOPIC" \
                --host "$QGC_VIDEO_HOST" \
                --port "$QGC_VIDEO_PORT" \
                --fps "$QGC_VIDEO_FPS" \
                --bitrate-kbps "$QGC_VIDEO_BITRATE_KBPS" \
                --width "$QGC_VIDEO_WIDTH" \
                --height "$QGC_VIDEO_HEIGHT"
        ) >"$QGC_VIDEO_LOG" 2>&1 &
        VIDEO_BRIDGE_PID=$!
        echo "[launch] auto video bridge pid=${VIDEO_BRIDGE_PID} (log: ${QGC_VIDEO_LOG})"
    else
        echo "[launch] warning: auto video bridge script not found: ${VIDEO_BRIDGE_SCRIPT}"
    fi
fi

python3 run_urdf_full.py \
    --scene competition_scene.xml \
    $IMAGES $SITL_ARG $HEADLESS_ARG \
    ${SITL_ARG:+--sitl-port "$SITL_PORT"} \
    ${SITL_ARG:+--sitl-send-port "$SITL_SEND_PORT"} \
    "${EXTRA_ARGS[@]}"
