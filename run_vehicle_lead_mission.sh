#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${VEHICLE_LEAD_CONFIG:-${ROOT_DIR}/configs/vehicle_lead_mission.json}"

"${ROOT_DIR}/run_multi_mission.sh" --config "${CONFIG_PATH}"
