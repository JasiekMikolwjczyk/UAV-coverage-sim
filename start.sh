#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${ROOT_DIR}/submodules/PX4-Autopilot"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found. Install it or launch the processes manually."
  exit 1
fi

pkill -9 -f gz_pose_to_ros_pose.py || true
pkill -9 -f "gz sim" || true
pkill -9 -f px4 || true
pkill -9 -f mavproxy.py || true

export GZ_SIM_RESOURCE_PATH="${ROOT_DIR}/sim/models"
export GZ_CONFIG_PATH="/usr/share/gz"
LOG_DIR="${ROOT_DIR}/data/logs/start"
mkdir -p "${LOG_DIR}"

if [ ! -d "${PX4_DIR}" ]; then
  echo "PX4 not found at ${PX4_DIR}. Please add PX4-Autopilot there."
  exit 1
fi

echo "Building ROS2 workspace..."
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${ROOT_DIR}/ros2_ws"
colcon build

spawn_terminal() {
  local title="$1"
  local cmd="$2"
  local log_file="${LOG_DIR}/$(echo "${title}" | tr ' /' '__' | tr -cd 'A-Za-z0-9_').log"
  echo "Launching: ${title}"
  : > "${log_file}"
  set +e
  gnome-terminal --title="$title" -- bash -lc "echo \"[start] ${title} $(date)\"; ${cmd} 2>&1 | tee -a '${log_file}'; status=\${PIPESTATUS[0]}; echo \"exit code: \$status\"; read -p 'press enter to close' -r" &
  local rc=$?
  set -e
  if [ "$rc" -ne 0 ]; then
    echo "Failed to launch terminal: ${title} (exit ${rc})"
  fi
  disown
  sleep 0.3
}

spawn_bg() {
  local title="$1"
  local cmd="$2"
  local log_file="${LOG_DIR}/$(echo "${title}" | tr ' /' '__' | tr -cd 'A-Za-z0-9_').log"
  echo "Launching (bg): ${title}"
  : > "${log_file}"
  nohup bash -lc "echo \"[start] ${title} $(date)\"; ${cmd}; echo \"[exit] \$? $(date)\"" >> "${log_file}" 2>&1 &
  local pid=$!
  echo "[pid] ${pid}" >> "${log_file}"
}

spawn_terminal "PX4 SITL + Gazebo" \
  "cd '${PX4_DIR}'; export GZ_SIM_RESOURCE_PATH='${GZ_SIM_RESOURCE_PATH}'; export GZ_CONFIG_PATH='${GZ_CONFIG_PATH}'; export PX4_GZ_HEADLESS=0; export HEADLESS=0; make px4_sitl gz_x500"
spawn_terminal "Gazebo GUI" \
  "sleep 6; export GZ_SIM_RESOURCE_PATH='${GZ_SIM_RESOURCE_PATH}'; export GZ_CONFIG_PATH='${GZ_CONFIG_PATH}'; gz sim -g"
spawn_terminal "MAVProxy GCS" \
  "sleep 6; '${ROOT_DIR}/tools/mavproxy_run.sh'"
spawn_bg "Gazebo Pose Bridge" \
  "sleep 8; set +u; source /opt/ros/jazzy/setup.bash; source '${ROOT_DIR}/ros2_ws/install/setup.bash'; set -u; python3 '${ROOT_DIR}/tools/gz_pose_to_ros_pose.py'"
spawn_bg "Throttle Bridge" \
  "sleep 8; set +u; source /opt/ros/jazzy/setup.bash; source '${ROOT_DIR}/ros2_ws/install/setup.bash'; set -u; python3 '${ROOT_DIR}/tools/throttle_bridge.py'"
spawn_bg "Battery Model" \
  "sleep 10; set +u; source /opt/ros/jazzy/setup.bash; source '${ROOT_DIR}/ros2_ws/install/setup.bash'; set -u; ros2 run battery_model battery_node"
