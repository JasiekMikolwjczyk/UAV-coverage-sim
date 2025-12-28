#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${ROOT_DIR}/data/coverage/multi"
RUNS_DIR="${ROOT_DIR}/data/coverage/multi_runs"

uav0=""
uav1=""
uav2=""
uav0_file=""
uav1_file=""
uav2_file=""
pass_through=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --uav0)
      uav0="${2:-}"; shift 2 ;;
    --uav1)
      uav1="${2:-}"; shift 2 ;;
    --uav2)
      uav2="${2:-}"; shift 2 ;;
    --uav0-file)
      uav0_file="${2:-}"; shift 2 ;;
    --uav1-file)
      uav1_file="${2:-}"; shift 2 ;;
    --uav2-file)
      uav2_file="${2:-}"; shift 2 ;;
    *)
      pass_through+=("$1"); shift ;;
  esac
done

if [[ -z "${uav0}" && -z "${uav0_file}" ]] || [[ -z "${uav1}" && -z "${uav1_file}" ]] || [[ -z "${uav2}" && -z "${uav2_file}" ]]; then
  echo "Usage: $0 --uav0 \"x,y,z;...\" --uav1 \"x,y,z;...\" --uav2 \"x,y,z;...\" [waypoint_runner args]"
  echo "   or: $0 --uav0-file path.txt --uav1-file path.txt --uav2-file path.txt [waypoint_runner args]"
  exit 1
fi

if [[ ! -f "${ROOT_DIR}/.venv/bin/activate" ]]; then
  echo "Missing .venv; create it and install pymavlink first."
  exit 1
fi

INTERRUPTED=0

cleanup() {
  if [[ -n "${COV_PID:-}" ]] && kill -0 "${COV_PID}" 2>/dev/null; then
    kill -SIGINT "${COV_PID}"
    wait "${COV_PID}" || true
  fi
}
trap 'INTERRUPTED=1' INT TERM

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
  local waypoints_file="$3"
  source "${ROOT_DIR}/.venv/bin/activate"
  if [[ -n "${waypoints_file}" ]]; then
    python3 "${ROOT_DIR}/tools/waypoint_runner.py" \
      --udp "${udp}" \
      --auto-arm \
      --auto-takeoff \
      --waypoints-file "${waypoints_file}" \
      "${pass_through[@]}"
  else
    python3 "${ROOT_DIR}/tools/waypoint_runner.py" \
      --udp "${udp}" \
      --auto-arm \
      --auto-takeoff \
      --waypoints "${waypoints}" \
      "${pass_through[@]}"
  fi
}

run_wp "udp:127.0.0.1:14540" "${uav0}" "${uav0_file}" &
P0=$!
run_wp "udp:127.0.0.1:14541" "${uav1}" "${uav1_file}" &
P1=$!
run_wp "udp:127.0.0.1:14542" "${uav2}" "${uav2_file}" &
P2=$!

set +e
wait "${P0}" "${P1}" "${P2}"
set -e

cleanup

wait_for_coverage_file() {
  local path="$1"
  local max_wait=20
  local waited=0
  local last_size=-1
  local stable_count=0
  while (( waited < max_wait )); do
    if [[ -f "${path}" ]]; then
      local size
      size=$(stat -c%s "${path}" 2>/dev/null || echo 0)
      if [[ "${size}" -gt 0 ]]; then
        if [[ "${size}" -eq "${last_size}" ]]; then
          stable_count=$((stable_count + 1))
        else
          stable_count=0
        fi
        last_size="${size}"
        if [[ "${stable_count}" -ge 2 ]]; then
          return 0
        fi
      fi
    fi
    sleep 0.5
    waited=$((waited + 1))
  done
  return 1
}

timestamp="$(date +%Y%m%d_%H%M%S)"
run_dir="${RUNS_DIR}/${timestamp}"
mkdir -p "${run_dir}"

{
  echo "timestamp=${timestamp}"
  echo "uav0=${uav0}"
  echo "uav1=${uav1}"
  echo "uav2=${uav2}"
  echo "uav0_file=${uav0_file}"
  echo "uav1_file=${uav1_file}"
  echo "uav2_file=${uav2_file}"
  if [[ ${#pass_through[@]} -gt 0 ]]; then
    echo "extra_args=${pass_through[*]}"
  fi
} > "${run_dir}/waypoints.txt"

if [[ -n "${uav0_file}" && -f "${uav0_file}" ]]; then
  cp -f "${uav0_file}" "${run_dir}/uav0_waypoints.txt"
fi
if [[ -n "${uav1_file}" && -f "${uav1_file}" ]]; then
  cp -f "${uav1_file}" "${run_dir}/uav1_waypoints.txt"
fi
if [[ -n "${uav2_file}" && -f "${uav2_file}" ]]; then
  cp -f "${uav2_file}" "${run_dir}/uav2_waypoints.txt"
fi

if wait_for_coverage_file "${OUT_DIR}/coverage_score.npy"; then
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
