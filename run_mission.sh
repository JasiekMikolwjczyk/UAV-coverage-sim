#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f "${ROOT_DIR}/.venv/bin/activate" ]]; then
  echo "Missing .venv; create it and install pymavlink first."
  exit 1
fi

cleanup() {
  if [[ -n "${COV_PID:-}" ]] && kill -0 "${COV_PID}" 2>/dev/null; then
    kill -SIGINT "${COV_PID}"
    wait "${COV_PID}" || true
  fi
}
trap cleanup INT TERM

(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROOT_DIR}/ros2_ws/install/setup.bash"
  set -u
  ros2 run coverage_mapper coverage_node --ros-args -p mode:=ros -p pose_topic:=/uav/pose
) &
COV_PID=$!

sleep 1

(
  source "${ROOT_DIR}/.venv/bin/activate"
  python3 "${ROOT_DIR}/tools/waypoint_runner.py" --auto-arm --auto-takeoff "$@"
)

cleanup
