#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${ROOT_DIR}/submodules/PX4-Autopilot"
WORLD_PATH="${ROOT_DIR}/sim/worlds/multi_quad7.sdf"
GZ_ENV_SH="${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh"
PX4_GZ_PLUGINS="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
PX4_GZ_SERVER_CONFIG="${PX4_DIR}/src/modules/simulation/gz_bridge/server.config"
PX4_GZ_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
PX4_GZ_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found. Install it or launch the processes manually."
  exit 1
fi

pkill -9 -f gz_pose_to_ros_pose.py || true
pkill -9 -f "gz sim" || true
pkill -9 -f px4 || true
pkill -9 -f mavproxy.py || true

for port in 14540 14541 14542; do
  for pid in $(ss -ulpn | awk -F'pid=' "/:${port}/ {print \$2}" | cut -d, -f1 | sort -u); do
    kill "$pid" || true
  done
done

export GZ_SIM_RESOURCE_PATH="${ROOT_DIR}/sim/models:${ROOT_DIR}/sim/worlds:${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_CONFIG_PATH="/usr/share/gz"
export PX4_GZ_WORLDS="${ROOT_DIR}/sim/worlds"
export PX4_GZ_WORLD="multi_quad7"

if [ -f "${GZ_ENV_SH}" ]; then
  set +u
  source "${GZ_ENV_SH}"
  set -u
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${PX4_GZ_PLUGINS}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_GZ_SERVER_CONFIG}"

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
export GZ_CONFIG_PATH="/usr/share/gz"

LOG_DIR="${ROOT_DIR}/data/logs/start_multi"
mkdir -p "${LOG_DIR}"

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
  echo "[pid] $!" >> "${log_file}"
}

spawn_terminal "Gazebo Multi (quad7 x3)" \
  "export GZ_SIM_RESOURCE_PATH='${GZ_SIM_RESOURCE_PATH}'; export GZ_SIM_SYSTEM_PLUGIN_PATH='${GZ_SIM_SYSTEM_PLUGIN_PATH}'; export GZ_SIM_SERVER_CONFIG_PATH='${GZ_SIM_SERVER_CONFIG_PATH}'; export GZ_CONFIG_PATH='/usr/share/gz'; gz sim --force-version 8 -r '${WORLD_PATH}'"

spawn_terminal "PX4 #0 (quad7_0)" \
  "cd '${PX4_DIR}'; export PX4_GZ_STANDALONE=1; export PX4_SYS_AUTOSTART=4001; export PX4_SIM_MODEL=gz_x500; export PX4_GZ_MODEL_NAME=quad7_0; export PX4_GZ_WORLD='${PX4_GZ_WORLD}'; export GZ_IP=127.0.0.1; ./build/px4_sitl_default/bin/px4 -i 0"
spawn_terminal "PX4 #1 (quad7_1)" \
  "cd '${PX4_DIR}'; export PX4_GZ_STANDALONE=1; export PX4_SYS_AUTOSTART=4001; export PX4_SIM_MODEL=gz_x500; export PX4_GZ_MODEL_NAME=quad7_1; export PX4_GZ_WORLD='${PX4_GZ_WORLD}'; export GZ_IP=127.0.0.1; ./build/px4_sitl_default/bin/px4 -i 1"
spawn_terminal "PX4 #2 (quad7_2)" \
  "cd '${PX4_DIR}'; export PX4_GZ_STANDALONE=1; export PX4_SYS_AUTOSTART=4001; export PX4_SIM_MODEL=gz_x500; export PX4_GZ_MODEL_NAME=quad7_2; export PX4_GZ_WORLD='${PX4_GZ_WORLD}'; export GZ_IP=127.0.0.1; ./build/px4_sitl_default/bin/px4 -i 2"

spawn_terminal "MAVProxy #0" \
  "sleep 6; '${ROOT_DIR}/tools/mavproxy_run.sh' udp:127.0.0.1:14540"
spawn_terminal "MAVProxy #1" \
  "sleep 6; '${ROOT_DIR}/tools/mavproxy_run.sh' udp:127.0.0.1:14541"
spawn_terminal "MAVProxy #2" \
  "sleep 6; '${ROOT_DIR}/tools/mavproxy_run.sh' udp:127.0.0.1:14542"

spawn_bg "Pose Bridge #0" \
  "sleep 8; set +u; source /opt/ros/jazzy/setup.bash; source '${ROOT_DIR}/ros2_ws/install/setup.bash'; set -u; python3 '${ROOT_DIR}/tools/gz_pose_to_ros_pose.py' --ros-args -p gz_topic:=/world/multi_quad7/pose/info -p target_entity:=quad7_0 -p out_topic:=/uav_0/pose"
spawn_bg "Pose Bridge #1" \
  "sleep 8; set +u; source /opt/ros/jazzy/setup.bash; source '${ROOT_DIR}/ros2_ws/install/setup.bash'; set -u; python3 '${ROOT_DIR}/tools/gz_pose_to_ros_pose.py' --ros-args -p gz_topic:=/world/multi_quad7/pose/info -p target_entity:=quad7_1 -p out_topic:=/uav_1/pose"
spawn_bg "Pose Bridge #2" \
  "sleep 8; set +u; source /opt/ros/jazzy/setup.bash; source '${ROOT_DIR}/ros2_ws/install/setup.bash'; set -u; python3 '${ROOT_DIR}/tools/gz_pose_to_ros_pose.py' --ros-args -p gz_topic:=/world/multi_quad7/pose/info -p target_entity:=quad7_2 -p out_topic:=/uav_2/pose"

spawn_bg "Ground Vehicle" \
  "sleep 8; python3 '${ROOT_DIR}/tools/ground_vehicle_runner.py' --world multi_quad7 --model ground_vehicle --speed 1.0 --loop --waypoints \"0,0,0.15; 10,0,0.15; 10,10,0.15; 0,10,0.15\""

spawn_bg "Ground Vehicle Pose Bridge" \
  "sleep 8; set +u; source /opt/ros/jazzy/setup.bash; source '${ROOT_DIR}/ros2_ws/install/setup.bash'; set -u; python3 '${ROOT_DIR}/tools/gz_pose_to_ros_pose.py' --ros-args -p gz_topic:=/world/multi_quad7/pose/info -p target_entity:=ground_vehicle -p out_topic:=/ground_vehicle/pose"
