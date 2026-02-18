#!/bin/bash
# Launch MuJoCo competition simulation with ROS2 bridge
# Usage:
#   ./launch_competition_sim.sh [--headless] [--images] [--sitl]
#   ./launch_competition_sim.sh --sitl --images --calib-left calibration/left.yaml --calib-right calibration/right.yaml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

# Parse arguments
HEADLESS=false
IMAGES=""
FORCE_ROS2=false
HEADLESS_ARG=""
SITL_ARG=""
EXTRA_ARGS=()
PROFILE=""
HOVER_STABLE_REQUESTED=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=true
            shift
            ;;
        --images)
            IMAGES="--ros2-images"
            FORCE_ROS2=true
            shift
            ;;
        --sitl)
            SITL_ARG="--sitl"
            FORCE_ROS2=true
            shift
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
            FORCE_ROS2=true
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

# Set display for headless mode
if [ "$HEADLESS" = true ]; then
    HEADLESS_ARG="--headless"
    export DISPLAY=""
    export MUJOCO_GL=egl
    echo "[launch] Running in headless mode (EGL rendering)"
fi

echo "[launch] Starting MuJoCo UUV Competition Simulation"
echo "[launch] Scene: competition_scene.xml"
if [ "$FORCE_ROS2" = true ] && [[ "$HEADLESS" == false ]]; then
    echo "[launch] Note: --sitl or --images requested, automatically enabling --ros2"
fi
echo "[launch] ROS2 Topics:"
echo "  Input:  /cmd_vel, /mavros/rc/override"
echo "  Output: /imu/data, /dvl/velocity, /dvl/odometry, /dvl/altitude"
echo "  Debug:  /mujoco/ground_truth/pose"
if [ -n "$IMAGES" ]; then
    echo "  Images: /stereo/left/image_raw, /stereo/right/image_raw"
fi
echo ""

if [ "$FORCE_ROS2" = true ]; then
    EXTRA_ARGS+=("--ros2")
    if [[ -n "$SITL_ARG" ]]; then
        # Intentionally do not force extra stabilization/depth-hold for SITL.
        # ArduPilot/QGC depth and attitude modes should own these loops.
        if [[ -z "$PROFILE" ]]; then
            PROFILE="sim_real"
            EXTRA_ARGS+=("--profile" "$PROFILE")
        fi
    fi
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
    "${EXTRA_ARGS[@]}"
