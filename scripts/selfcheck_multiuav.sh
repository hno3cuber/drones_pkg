#!/usr/bin/env bash
# 端到端自检：起 world -> 起四架 PX4 -> 核对是否各自 spawn 到指定位置 -> 清理。
# 用于本地验证 launch 的完整链路（不经过 ros2 launch，直接复刻其行为）。
set -u

PX4=/home/hno3/PX4-Autopilot
PKG=/home/hno3/ws_ros2/src/drones_pkg
WORLD_NAME=obstacles_world
WORLD_FILE=$PKG/worlds/$WORLD_NAME.sdf
LOGDIR=$PKG/.tmp_logs
PX4_BIN=$PX4/build/px4_sitl_default/bin/px4

export GZ_SIM_RESOURCE_PATH=$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds:$PKG/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4/build/px4_sitl_default/src/modules/simulation/gz_plugins
export GZ_SIM_SERVER_CONFIG_PATH=$PX4/src/modules/simulation/gz_bridge/server.config

mkdir -p "$LOGDIR"

PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  sleep 1
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT

echo "[1] 起 gz server"
gz sim -r -s -v 1 "$WORLD_FILE" > "$LOGDIR/e2e_gz.log" 2>&1 &
PIDS+=($!)

for i in $(seq 1 40); do
  gz topic -l 2>/dev/null | grep -q "/world/$WORLD_NAME/clock" && break
  sleep 1
done
gz topic -l 2>/dev/null | grep -q "/world/$WORLD_NAME/clock" \
  || { echo "!! world 未就绪"; tail -15 "$LOGDIR/e2e_gz.log"; exit 1; }
echo "    world 就绪"

echo "[2] 起四架 PX4"
declare -A POSE=( [0]="0,0" [1]="1,0" [2]="-1,0" [3]="2,0" )
for i in 0 1 2 3; do
  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
  PX4_GZ_WORLD=$WORLD_NAME PX4_GZ_MODEL_POSE="${POSE[$i]},0,0,0,0" \
  PX4_GZ_STANDALONE=1 PX4_GZ_NO_FOLLOW=1 \
  "$PX4_BIN" -i "$i" > "$LOGDIR/e2e_px4_$i.log" 2>&1 &
  PIDS+=($!)
  sleep 2
done

echo "[3] 等 12s 让四架完成 spawn / 估计器收敛"
sleep 12

echo "[4] 场景中的模型（应含 x500_0..x500_3）"
gz service -s /world/$WORLD_NAME/scene/info \
  --reqtype gz.msgs.Empty --reptype gz.msgs.Scene \
  --timeout 3000 --req '' 2>/dev/null | grep -E 'name: "' | head -30

echo
echo "[5] 各架 PX4 出生位姿"
gz topic -e -t /world/$WORLD_NAME/pose/info -n 1 2>/dev/null \
  | awk '/name: "x500_/{p=1;print;next} p&&/position/{print;next} p&&/^[[:space:]]+[xyz]:/{print;next} /name: "(ground_plane|sunUTC|obstacle_)/{p=0}' \
  | head -40

echo
echo "[6] 各架 PX4 日志关键行（有无 spawn 成功/报错）"
for i in 0 1 2 3; do
  echo "--- px4 -i $i ---"
  grep -iE "spawning|gz_bridge|error|fail|ready for takeoff" "$LOGDIR/e2e_px4_$i.log" | head -6
done

echo
echo "完成，清理。"
