#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MASTER="${1:-udp:127.0.0.1:14540}"
ADSB_CMD="module unload adsb"

resolve_mavproxy() {
  if command -v mavproxy.py >/dev/null 2>&1; then
    echo "mavproxy.py"
    return 0
  fi
  if [ -x "${ROOT_DIR}/.venv/bin/mavproxy.py" ]; then
    echo "${ROOT_DIR}/.venv/bin/mavproxy.py"
    return 0
  fi
  return 1
}

while true; do
  for _ in $(seq 1 40); do
    ss -ulpn | grep -q ':14580' && break
    sleep 0.5
  done

  MAVPROXY_BIN="$(resolve_mavproxy || true)"
  if [ -z "${MAVPROXY_BIN:-}" ]; then
    echo "mavproxy.py not found. Install in .venv or system PATH."
    exit 127
  fi

  echo "[mavproxy] starting ${MAVPROXY_BIN} --master=${MASTER}"
  "${MAVPROXY_BIN}" --master="${MASTER}" --force-connected --cmd="${ADSB_CMD}"
  echo "[mavproxy] exited; restarting in 2s"
  sleep 2
done
