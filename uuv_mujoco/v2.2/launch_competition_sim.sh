#!/bin/bash
# Launch MuJoCo competition simulation with ROS2 bridge
# Usage:
#   ./launch_competition_sim.sh [--headless] [--images] [--sitl]
#   ./launch_competition_sim.sh --hover-stable --sitl --images
#   ./launch_competition_sim.sh --sitl --images --calib-left calibration/left.yaml --calib-right calibration/right.yaml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

# Parse arguments
HEADLESS=false
IMAGES=""
HEADLESS_ARG=""
SITL_ARG=""
EXTRA_ARGS=()
HOVER_STABLE=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=true
            shift
            ;;
        --images)
            IMAGES="--ros2-images"
            shift
            ;;
        --sitl)
            SITL_ARG="--sitl"
            shift
            ;;
        --hover-stable)
            HOVER_STABLE=true
            shift
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

if [ "$HOVER_STABLE" = true ]; then
    EXTRA_ARGS+=(
        "--profile" "sim_hover"
        "--imu-stabilize"
        "--depth-hold"
        "--imu-stab-mode" "both"
        "--imu-stab-kp" "6.8"
        "--imu-stab-kd" "2.2"
        "--imu-stab-ki" "0.2"
        "--imu-stab-max" "8.5"
        "--depth-hold-kp" "2.6"
        "--depth-hold-kd" "1.0"
        "--depth-hold-ki" "0.03"
        "--depth-hold-cmd-max" "0.7"
        "--depth-hold-user-deadband" "0.03"
    )
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
echo "[launch] ROS2 Topics:"
echo "  Input:  /cmd_vel, /mavros/rc/override"
echo "  Output: /imu/data, /dvl/velocity, /dvl/odometry, /dvl/altitude"
echo "  Debug:  /mujoco/ground_truth/pose"
if [ -n "$IMAGES" ]; then
    echo "  Images: /stereo/left/image_raw, /stereo/right/image_raw"
fi
if [ "$HOVER_STABLE" = true ]; then
    echo "[launch] Hover-stable preset enabled (profile=sim_hover, imu stabilization=both)"
fi
echo ""

python3 run_urdf_full.py \
    --scene competition_scene.xml \
    --ros2 $IMAGES $SITL_ARG $HEADLESS_ARG \
    --ros2-sensor-hz 50 \
    --ros2-image-hz 15 \
    "${EXTRA_ARGS[@]}"
