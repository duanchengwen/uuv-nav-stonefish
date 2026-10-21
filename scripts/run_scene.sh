#!/usr/bin/env bash
# Launch one of the generated scenes in a fresh run directory.
#
# Usage:
#   scripts/run_scene.sh scenes/test/episode_000001 [run_dir]
#
# The source scene directory stays untouched; the app writes images/samples to
# the run directory. Use ros2/run_camera_view.sh <run_dir> for the RViz stream.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENE_DIR="${1:-}"

if [[ -z "$SCENE_DIR" || ! -f "$SCENE_DIR/scene.scn" ]]; then
  echo "Usage: $0 <scene-dir> [run-dir]" >&2
  echo "Example: $0 scenes/test/episode_000001" >&2
  echo "Available scenes:" >&2
  find "$ROOT/scenes" -mindepth 2 -maxdepth 2 -type d -name 'episode_*' 2>/dev/null | sort | sed 's#^#  #' >&2
  exit 2
fi
if [[ "$SCENE_DIR" != /* ]]; then
  SCENE_DIR="$ROOT/$SCENE_DIR"
fi
if [[ ! -f "$SCENE_DIR/path.csv" || ! -f "$SCENE_DIR/obstacles.csv" || ! -f "$SCENE_DIR/episode.cfg" ]]; then
  echo "error: $SCENE_DIR must contain scene.scn, path.csv, obstacles.csv, episode.cfg" >&2
  exit 2
fi

SPLIT="$(basename "$(dirname "$SCENE_DIR")")"
NAME="$(basename "$SCENE_DIR")"
RUN_DIR="${2:-$ROOT/episodes_raw/run_${SPLIT}_${NAME}}"

rm -rf "$RUN_DIR"
mkdir -p "$RUN_DIR"
cp "$SCENE_DIR"/{scene.scn,path.csv,obstacles.csv,episode.cfg} "$RUN_DIR"/
if [[ -f "$SCENE_DIR/preview.png" ]]; then
  cp "$SCENE_DIR/preview.png" "$RUN_DIR"/
fi

SF_INSTALL="${SF_INSTALL:-$(cd "$ROOT/.." && pwd)/install_stonefish}"
export LD_LIBRARY_PATH="$SF_INSTALL/lib:${LD_LIBRARY_PATH:-}"

echo "scene:  $SCENE_DIR"
echo "run:    $RUN_DIR"
echo "ROS 2:  $ROOT/ros2/run_camera_view.sh \"$RUN_DIR\""
echo

exec "$ROOT/build/uuv_nav" \
  --scene "$RUN_DIR/scene.scn" \
  --episode-dir "$RUN_DIR"
