#!/usr/bin/env bash
# 起 obstacles_world 并核对三根障碍柱是否真的在、坐标是否对。
# 一次性自检脚本：拉起 gz server -> 等 world 就绪 -> 打印模型与位姿 -> 清理退出。
set -u

PX4=/home/hno3/PX4-Autopilot
PKG=/home/hno3/ws_ros2/src/drones_pkg
WORLD_NAME=obstacles_world
WORLD_FILE=$PKG/worlds/$WORLD_NAME.sdf
LOG=$PKG/.tmp_logs/selfcheck_gz.log

export GZ_SIM_RESOURCE_PATH=$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds:$PKG/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4/build/px4_sitl_default/src/modules/simulation/gz_plugins
export GZ_SIM_SERVER_CONFIG_PATH=$PX4/src/modules/simulation/gz_bridge/server.config

mkdir -p "$PKG/.tmp_logs"

cleanup() {
  [ -n "${GZ_PID:-}" ] && kill "$GZ_PID" 2>/dev/null
  wait "$GZ_PID" 2>/dev/null
}
trap cleanup EXIT

echo "[1/4] 起 gz server: $WORLD_FILE"
gz sim -r -s -v 1 "$WORLD_FILE" > "$LOG" 2>&1 &
GZ_PID=$!

echo "[2/4] 等 world 就绪（最多 40s）"
for i in $(seq 1 40); do
  if gz topic -l 2>/dev/null | grep -q "/world/$WORLD_NAME/clock"; then
    echo "      world 就绪（第 ${i}s）"
    break
  fi
  sleep 1
done

if ! gz topic -l 2>/dev/null | grep -q "/world/$WORLD_NAME/clock"; then
  echo "      !! world 未就绪，日志尾部："
  tail -20 "$LOG"
  exit 1
fi

echo "[3/4] 场景里的模型："
gz service -s /world/$WORLD_NAME/scene/info \
  --reqtype gz.msgs.Empty --reptype gz.msgs.Scene \
  --timeout 3000 --req '' 2>/dev/null | grep -E "name:" | head -20

echo
echo "[4/4] 三根柱子的位姿："
gz topic -e -t /world/$WORLD_NAME/pose/info -n 1 2>/dev/null \
  | grep -E 'name:|position|^\s+x:|^\s+y:|^\s+z:' \
  | awk '/name: "obstacle_/{p=1;print;next} p&&/position/{print;next} p&&/^[[:space:]]+[xyz]:/{print;next} /name: "(ground_plane|sunUTC)"/{p=0}' \
  | head -30

echo
echo "完成，清理 gz server。"
