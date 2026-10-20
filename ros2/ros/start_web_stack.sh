#!/usr/bin/env bash
# Six-axis MCU session driver + guarded web bridge + rosbridge + Vite.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$(cd "${ROOT}/../../web" && pwd)"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
BAUD="${BAUD:-921600}"
LOG_DIR="${ROOT}/run_logs/$(date +%Y%m%d_%H%M%S)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${ROOT}/install/setup.bash"
mkdir -p "$LOG_DIR"
[[ -e "$SERIAL_PORT" ]] || { echo "Serial device missing: $SERIAL_PORT" >&2; exit 1; }
ros2 launch eng26_arm_driver driver.launch.py device:="$SERIAL_PORT" baud_rate:="$BAUD" >"$LOG_DIR/driver.log" 2>&1 &
PIDS+=($!)
ros2 launch eng26_web_bridge web_bridge.launch.py >"$LOG_DIR/web_bridge.log" 2>&1 &
PIDS+=($!)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml >"$LOG_DIR/rosbridge.log" 2>&1 &
PIDS+=($!)
(cd "$WEB_DIR" && exec node node_modules/vite/bin/vite.js --host 0.0.0.0) >"$LOG_DIR/vite.log" 2>&1 &
PIDS+=($!)
echo "Web: http://localhost:5173/?rosProfile=web_bridge"
echo "Serial: $SERIAL_PORT @ $BAUD; logs: $LOG_DIR"
set +e
wait -n "${PIDS[@]}"
STATUS=$?
exit "$STATUS"
