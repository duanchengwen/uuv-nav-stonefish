#!/usr/bin/env bash
# Start the ROS 2 camera publisher and RViz2 for an already-running episode.
#
# Usage:
#   ros2/run_camera_view.sh [episode_dir]
#
# The Stonefish app must be running and writing images/<ego,wrist>/*.png.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EPISODE_DIR="${1:-$ROOT/episodes_raw/live_demo}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found at /opt/ros/humble" >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u

if [[ ! -d "$EPISODE_DIR/images" ]]; then
  echo "warning: $EPISODE_DIR/images does not exist yet; start the simulation first" >&2
fi

/usr/bin/python3 "$ROOT/ros2/camera_publisher.py" --episode-dir "$EPISODE_DIR" &
PUB_PID=$!
trap 'kill $PUB_PID 2>/dev/null || true' EXIT

rviz2 -d "$ROOT/ros2/uuv_cameras.rviz"
