#!/usr/bin/env bash
# Run ROS2 stereo camera calibration for MuJoCo stereo topics.
# Usage:
#   ./scripts/run_stereo_calibration.sh --size 8x6 --square 0.025 --out-dir calibration

set -euo pipefail

SIZE="8x6"
SQUARE="0.025"
APPROX="0.1"
LEFT_TOPIC="/stereo/left/image_raw"
RIGHT_TOPIC="/stereo/right/image_raw"
LEFT_NS="/stereo/left"
RIGHT_NS="/stereo/right"
OUT_DIR="calibration"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --size)
            SIZE="$2"
            shift 2
            ;;
        --square)
            SQUARE="$2"
            shift 2
            ;;
        --approx)
            APPROX="$2"
            shift 2
            ;;
        --left-topic)
            LEFT_TOPIC="$2"
            shift 2
            ;;
        --right-topic)
            RIGHT_TOPIC="$2"
            shift 2
            ;;
        --left-ns)
            LEFT_NS="$2"
            shift 2
            ;;
        --right-ns)
            RIGHT_NS="$2"
            shift 2
            ;;
        --out-dir)
            OUT_DIR="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            exit 2
            ;;
    esac
done

source /opt/ros/humble/setup.bash

if ! ros2 pkg list | rg -q '^camera_calibration$'; then
    echo "[error] ros-humble-camera-calibration is not installed."
    echo "Install with: sudo apt-get update && sudo apt-get install -y ros-humble-camera-calibration"
    exit 2
fi

mkdir -p "$OUT_DIR"

echo "[info] 1) Keep simulator running with stereo topics:"
echo "       ./launch_competition_sim.sh --sitl --images"
echo "[info] 2) In calibration GUI, capture enough checkerboard views and click CALIBRATE + SAVE."
echo "[info] 3) Save/rename output YAMLs to:"
echo "       $OUT_DIR/left.yaml"
echo "       $OUT_DIR/right.yaml"
echo "[info] 4) Re-run simulator with calibration:"
echo "       ./launch_competition_sim.sh --sitl --images --calib-left $OUT_DIR/left.yaml --calib-right $OUT_DIR/right.yaml"
echo

ros2 run camera_calibration cameracalibrator \
    --size "$SIZE" \
    --square "$SQUARE" \
    --approximate "$APPROX" \
    left:="$LEFT_TOPIC" \
    right:="$RIGHT_TOPIC" \
    left_camera:="$LEFT_NS" \
    right_camera:="$RIGHT_NS"
