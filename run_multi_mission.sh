#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${ROOT_DIR}/data/coverage/multi"
RUNS_DIR="${ROOT_DIR}/data/coverage/multi_runs"

uav0=""
uav1=""
uav2=""
pass_through=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --uav0)
      uav0="${2:-}"; shift 2 ;;
    --uav1)
      uav1="${2:-}"; shift 2 ;;
    --uav2)
      uav2="${2:-}"; shift 2 ;;
    *)
      pass_through+=("$1"); shift ;;
  esac
done

if [[ -z "${uav0}" || -z "${uav1}" || -z "${uav2}" ]]; then
  echo "Usage: $0 --uav0 \"x,y,z;...\" --uav1 \"x,y,z;...\" --uav2 \"x,y,z;...\" [waypoint_runner args]"
  exit 1
fi

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
  ros2 run coverage_mapper coverage_node --ros-args \
    -p mode:=ros_multi \
    -p pose_topics:="['/uav_0/pose','/uav_1/pose','/uav_2/pose']" \
    -p out_dir:="${OUT_DIR}" \
    -p score_per_tick:=1.0 \
    -p tick_hz:=20.0
) &
COV_PID=$!

sleep 1

run_wp() {
  local udp="$1"
  local waypoints="$2"
  source "${ROOT_DIR}/.venv/bin/activate"
  python3 "${ROOT_DIR}/tools/waypoint_runner.py" \
    --udp "${udp}" \
    --auto-arm \
    --auto-takeoff \
    --waypoints "${waypoints}" \
    "${pass_through[@]}"
}

run_wp "udp:127.0.0.1:14540" "${uav0}" &
P0=$!
run_wp "udp:127.0.0.1:14541" "${uav1}" &
P1=$!
run_wp "udp:127.0.0.1:14542" "${uav2}" &
P2=$!

wait "${P0}" "${P1}" "${P2}"

cleanup

timestamp="$(date +%Y%m%d_%H%M%S)"
run_dir="${RUNS_DIR}/${timestamp}"
mkdir -p "${run_dir}"

{
  echo "timestamp=${timestamp}"
  echo "uav0=${uav0}"
  echo "uav1=${uav1}"
  echo "uav2=${uav2}"
  if [[ ${#pass_through[@]} -gt 0 ]]; then
    echo "extra_args=${pass_through[*]}"
  fi
} > "${run_dir}/waypoints.txt"

if [[ -f "${OUT_DIR}/coverage_score.npy" ]]; then
  cp -f "${OUT_DIR}/coverage_score.npy" "${run_dir}/coverage_score.npy"
  cp -f "${OUT_DIR}/coverage_score.csv" "${run_dir}/coverage_score.csv"
  python3 "${ROOT_DIR}/tools/plot_coverage_score.py" \
    --path "${run_dir}/coverage_score.npy" \
    --title "Multi-UAV coverage score" \
    --out "${run_dir}/coverage_score.png" \
    --no-show || true
  echo "Saved run artifacts to ${run_dir}"
else
  echo "coverage_score.npy not found in ${OUT_DIR}"
fi
