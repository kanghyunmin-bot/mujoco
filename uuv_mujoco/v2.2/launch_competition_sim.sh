#!/bin/bash
# Launch MuJoCo competition simulation
# Usage:
#   ./launch_competition_sim.sh [--headless] [--sitl] [--images] [--ros2] [--force-clean]
#   ./launch_competition_sim.sh --sitl --images --calib-left calibration/left.yaml --calib-right calibration/right.yaml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

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
THRUSTER_VOLTAGE=""
HOVER_STABLE_REQUESTED=false
ALLOW_LOCAL_JOYSTICK=false
FORCE_CLEAN=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=true
            shift
            ;;
        --images)
            IMAGES="--ros2-images"
            ROS2_REQUESTED=true
            EXTRA_ARGS+=(--ros2)
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
        --allow-local-joystick)
            ALLOW_LOCAL_JOYSTICK=true
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
            THRUSTER_VOLTAGE="$2"
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

collect_existing_mujoco_pids() {
    ps -eo pid=,args= | awk '
        index($0, "run_urdf_full.py --scene competition_scene.xml") > 0 { print $1 }
    '
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
    if [[ "$ALLOW_LOCAL_JOYSTICK" == false && "$ROS2_REQUESTED" == false ]]; then
        # Prevent accidental local joystick noise from overriding SITL inputs.
        EXTRA_ARGS+=(--disable-joystick)
        echo "[launch] SITL mode: local /dev/input joystick disabled (use --allow-local-joystick to override)."
    fi
fi

if [ "$ROS2_REQUESTED" = true ]; then
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

python3 run_urdf_full.py \
    --scene competition_scene.xml \
    $IMAGES $SITL_ARG $HEADLESS_ARG \
    --ros2-sensor-hz 50 \
    --ros2-image-hz 15 \
    ${SITL_ARG:+--sitl-port "$SITL_PORT"} \
    ${SITL_ARG:+--sitl-send-port "$SITL_SEND_PORT"} \
    "${EXTRA_ARGS[@]}"
