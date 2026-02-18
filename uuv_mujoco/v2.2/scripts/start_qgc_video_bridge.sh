#!/usr/bin/env bash
# Stream ROS2 stereo image topic to QGroundControl UDP video.
#
# Example:
#   ./scripts/start_qgc_video_bridge.sh --topic /stereo/left/image_raw --port 5600

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

python3 "${SCRIPT_DIR}/ros2_to_qgc_video.py" "$@"
