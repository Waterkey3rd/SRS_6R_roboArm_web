#!/usr/bin/env bash
# ENG26 双臂 web 调试栈一键启动：
#   usb_bridge_node(串口) + web_bridge_node(话题中继) + rosbridge_server(WebSocket) + vite(Web前端)
# Ctrl+C 退出并杀掉全部子进程。
# 前置条件（只需做一次）：
#   sudo apt install -y ros-humble-rosbridge-suite
#   web/node_modules 已安装（缺失时本脚本会用 npmmirror 镜像自动 npm ci）

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$(cd "${ROOT}/../../web" && pwd)"
LOG_DIR="${ROOT}/run_logs/$(date +%Y%m%d_%H%M%S)"
PIDS=()
SERIAL_PORT="/dev/ttyUSB0"
BAUD=921600

mkdir -p "${LOG_DIR}"

cleanup() {
  echo
  echo "[exit] stopping all nodes..."
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  pkill -f "rosbridge_websocket" 2>/dev/null || true
  pkill -f "usb_bridge_node" 2>/dev/null || true
  pkill -f "web_bridge_node" 2>/dev/null || true
  pkill -f "node_modules/.bin/vite" 2>/dev/null || true
  wait 2>/dev/null || true
  echo "[exit] done"
}
trap cleanup EXIT INT TERM

# 防止串口被别的进程（如 eng 的 comm_node）占用
if pgrep -f "comm_pkg.comm_node" >/dev/null 2>&1; then
  echo "[error] eng 的 comm_node 正在运行并占用串口，先停掉 eng-real 服务："
  echo "        sudo systemctl stop eng-real.service"
  exit 1
fi

# 从 PATH 中剔除 miniconda：ROS Humble 的 rclpy 只兼容系统 Python 3.10，
# 否则 rosbridge 等所有 Python 节点会因找不到 _rclpy_pybind11 启动即崩。
# node 在 /usr/bin 下，不受影响。
PATH_NO_CONDA="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:)"
export PATH="${PATH_NO_CONDA:-/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin}"
if [[ "$(command -v python3)" != "/usr/bin/python3" ]]; then
  echo "[error] python3 仍指向 $(command -v python3)，ROS 节点会启动失败"
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ ! -f "${ROOT}/install/setup.bash" ]]; then
  echo "[error] ${ROOT}/install/setup.bash 不存在，先在工作区执行 colcon build"
  exit 1
fi
source "${ROOT}/install/setup.bash"

if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
  echo "[error] rosbridge 未安装，先执行: sudo apt install -y ros-humble-rosbridge-suite"
  exit 1
fi

# 等串口就绪（刚重启时 USB 可能还没枚举完）
echo "[wait] 等待 ${SERIAL_PORT} 出现..."
for i in $(seq 1 30); do
  [[ -e "${SERIAL_PORT}" ]] && break
  sleep 1
done
if [[ ! -e "${SERIAL_PORT}" ]]; then
  echo "[error] 30 秒内 ${SERIAL_PORT} 未出现，检查 USB 线"
  exit 1
fi

echo "[start] usb_bridge_node (${SERIAL_PORT} @ ${BAUD})"
ros2 run eng26_usb_bridge usb_bridge_node \
  --ros-args -p serial_port:="${SERIAL_PORT}" -p baud_rate:="${BAUD}" \
  >"${LOG_DIR}/usb_bridge.log" 2>&1 &
PIDS+=($!)

echo "[start] web_bridge_node"
ros2 run eng26_web_bridge web_bridge_node \
  >"${LOG_DIR}/web_bridge.log" 2>&1 &
PIDS+=($!)

echo "[start] rosbridge_server (ws://localhost:9090)"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  >"${LOG_DIR}/rosbridge.log" 2>&1 &
PIDS+=($!)

echo "[start] web 前端 (vite dev)"
if [[ ! -d "${WEB_DIR}/node_modules" ]]; then
  echo "  node_modules 缺失，自动安装（npmmirror 镜像）..."
  (cd "${WEB_DIR}" && npm ci --registry=https://registry.npmmirror.com) \
    >"${LOG_DIR}/npm_ci.log" 2>&1
fi
(cd "${WEB_DIR}" && exec npm run dev) \
  >"${LOG_DIR}/vite.log" 2>&1 &
PIDS+=($!)

sleep 5
echo
echo "================ 全部就绪 ================"
echo "  Web 调试页面 : http://localhost:5173  (选择 ros2_real 档位)"
echo "  WebSocket    : ws://localhost:9090"
echo "  串口         : ${SERIAL_PORT} @ ${BAUD}"
echo "  日志目录     : ${LOG_DIR}/"
echo "  Ctrl+C 停止全部节点"
echo "=========================================="
echo

set +e
wait -n "${PIDS[@]}"
STATUS=$?
set -e
echo "[warn] 有节点退出 (status=${STATUS})，停止整组" >&2
exit "${STATUS}"
