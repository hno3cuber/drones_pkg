# drones_pkg

PX4 OFFBOARD 参考轨迹跟踪节点。读取 `src/` 下的两行 CSV 平面参考轨迹，
驱动无人机按该轨迹飞行。支持单机与多机：同一可执行文件通过参数区分身份
（`node_namespace` / `target_system` / `trajectory_file`）。

四架无人机的身份与轨迹在 `launch/uav_config.py` 中统一定义，
四个 launch 文件共用这一份配置，改一架无人机只改那一处。

## 依赖

- ROS 2 Humble
- `px4_msgs`（同工作空间内）
- PX4 SITL + Micro XRCE-DDS Agent

## 构建

```bash
cd ~/ws_ros2
colcon build --packages-select drones_pkg
source install/setup.bash
```

## 轨迹文件格式

`src/ref.csv`（零号机默认轨迹）为 **两行 CSV**：

```
x0,x1,x2,...,xN     <- 第一行：x 坐标
y0,y1,y2,...,yN     <- 第二行：y 坐标
```

默认按 **ENU**（x=东, y=北）解释，内部转换为 PX4 的 **NED**。
若文件本身是 NED，把参数 `frame_is_enu` 设为 `false`。

## 运行

单机：

```bash
ros2 launch drones_pkg trajectory_follower.launch.py
```

### 单独运行 uav1

uav1 跟 `src/ref_uav1.csv`，走 PX4 实例 1（命名空间 `/px4_1/fmu`，系统 ID 2）。
身份与轨迹由 `launch/uav_config.py` 自动带出，一条命令即可：

```bash
cd ~/ws_ros2
colcon build --packages-select drones_pkg   # 新增的 ref_uav1.csv 要装进 install
source install/setup.bash
ros2 launch drones_pkg trajectory_follower.launch.py uav:=uav1
```

前提是 PX4 实例 1 已起来（`px4_1`，`MAV_SYS_ID=2`，命名空间 `/px4_1/fmu`），
否则本节点找不到话题、无法 arm/offboard。

只想跑控制、不起可视化：

```bash
ros2 launch drones_pkg trajectory_follower.launch.py uav:=uav1 visualizer:=false
```

多机（四架一次起）：

```bash
ros2 launch drones_pkg trajectory_follower_multi.launch.py
```

多机模式下四架的身份与轨迹对应关系（事实源为 `launch/uav_config.py`）：

| 节点名 | node_namespace | target_system | 轨迹 |
| --- | --- | --- | --- |
| trajectory_follower_uav0 | `/fmu` | 1 | ref.csv |
| trajectory_follower_uav1 | `/px4_1/fmu` | 2 | ref_uav1.csv |
| trajectory_follower_uav2 | `/px4_2/fmu` | 3 | ref_uav2.csv |
| trajectory_follower_uav3 | `/px4_3/fmu` | 4 | ref_uav3.csv |

uav1 / uav2 / uav3 跟的 `ref_uav1.csv` / `ref_uav2.csv` / `ref_uav3.csv` 都是同一套
两步链条串出来的：`ref.csv` 先在原位置避障，结果沿 NED x（North）、y（East）
平移后再避障导出（见「离线导出避障后轨迹」）。第一次用默认 `short`，第二次按障碍
逐个指定弧向（uav1/uav2 用 `["short", "long", "short"]`，uav3 用
`["short", "short", "short"]`，原因见下）。三机只是平移量不同：

| 机 | NED 平移 (x=North, y=East) | ENU 平移 (x=East, y=North) | 终点 ENU | 点数 |
| --- | --- | --- | --- | --- |
| uav1 | x +2.0 | y +2.0 | (25, 8) | 2123 |
| uav2 | x -1.414, y -1.414 | x -1.414, y -1.414 | (23.586, 4.586) | 2015 |
| uav3 | x -1.414, y +1.414 | x +1.414, y -1.414 | (26.414, 4.586) | 1909 |

因平移已烘入文件，`uav_config.py` 中它们的偏移均为 0。

## 实飞航迹可视化

`trajectory_visualizer` 只订阅各架的实飞位置，为每架发一条独立的三维
航迹话题，便于在 RViz 中**各自配色**区分：

| PX4 命名空间 | 航迹话题 |
| --- | --- |
| `/fmu` | `~/fmu/flown_path` |
| `/px4_1/fmu` | `~/px4_1_fmu/flown_path` |
| `/px4_2/fmu` | `~/px4_2_fmu/flown_path` |
| `/px4_3/fmu` | `~/px4_3_fmu/flown_path` |

话题名由命名空间转换而来：去掉前导 `/`，内部 `/` 换成 `_`。

多机 launch 已同时启动可视化节点（订阅四架）：

```bash
ros2 launch drones_pkg trajectory_follower_multi.launch.py
```

在 RViz 中为每个 `Path` 话题各加一个 Path 显示插件，颜色自行指定即可。
单机时可只启动可视化、不启动控制节点：

```bash
ros2 launch drones_pkg trajectory_visualizer.launch.py visualizer_only:=true
```

## 参数

> **注意：参数段的节点名用 `/**` 通配，不要写死成某个节点名。**
> 各 launch 创建的节点名并不统一（单机为 `trajectory_follower_uav0`，
> 多机为 `uav0` / `uav1` ...），ROS 2 按节点名精确匹配参数，
> 一旦写死成其中一个名字，其余节点会静默地读不到任何参数、退回代码默认值。
> `/**` 能让文件加载到的全部 follower / visualizer 节点都生效。

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `node_namespace` | `/fmu` | PX4 话题命名空间，多机为 `/px4_1/fmu` 等 |
| `target_system` | `1` | 目标 PX4 系统 ID（MAV_SYS_ID），多机需与实例对应 |
| `trajectory_file` | `""` | 轨迹文件路径，**必填**，为空则节点启动失败；launch 会按 `uav_config.py` 自动填入 |
| `dt` | `0.1` | 相邻点时间步长 [s]（csv 无时间列时使用） |
| `frame_is_enu` | `true` | 输入是否为 ENU |
| `flight_altitude` | `5.0` | 目标飞行高度 [m] |
| `origin_offset_ned_x` | `0.0` | 轨迹沿 NED x 轴（North）的整体平移 [m]；多机共用同一 csv 时用它错开 |
| `origin_offset_ned_y` | `0.0` | 轨迹沿 NED y 轴（East）的整体平移 [m] |
| `approach_speed` | `2.0` | 飞向轨迹起点的速度 [m/s] |
| `setpoint_rate_hz` | `20.0` | setpoint 发布频率 [Hz]，需 > 2Hz |
| `approach_tolerance` | `0.5` | 判定到达轨迹起点的位置容差 [m] |
| `loop_trajectory` | `false` | 是否循环播放轨迹 |
| `avoidance_enable` | `true` | 避障总开关，`false` 时完全按参考轨迹飞 |
| `avoidance_obstacles` | `[4,4, 7,7, 15,4]` | 障碍圆心扁平数组 `[x0,y0,x1,y1,...]`（ENU） |
| `avoidance_radius` | `1.5` | 安全圆半径 [m]，轨迹进入该半径内即沿圆弧绕行 |
| `avoidance_arc_side` | `short` | 绕行弧走向：`short` 角度差小的一侧 / `long` 另一侧 |
| `avoidance_arc_side_by_obstacle` | `[]` | 按障碍下标覆盖弧向，如 `["short","long","short"]` |
| `export_avoided_csv` | `""` | 避障处理结果导出路径；非空则把处理后的轨迹写成两行 ENU csv |
| `end_action` | `hold` | 轨迹结束后动作：`hold` 悬停 / `rtl` 返航 |

## 避障

`avoidance_enable` 打开时，节点在**加载轨迹后做一次性预处理**：找出所有落入障碍
安全圆的连续轨迹段，用圆外相邻点插值求出与圆的两个交点，把这一段替换成
沿圆周走的弧（等角度铺点，密度与原轨迹一致）。处理后的点序列用于飞行，
运行时不再做几何计算。

因此这种避障是**开环的几何避让**：它保证参考轨迹不穿过安全圆，但不感知
实时位置与突发障碍。障碍物位置来自 `avoidance_obstacles`，与
`obstacle_visualizer` 的 `obstacles` 保持同一组坐标（均按 ENU 解释）。

注意这是 **2D 避让**：只在水平面绕开，高度保持 `flight_altitude` 不变。
若障碍高度与飞行高度接近，需自行评估垂直方向的安全余量。

判据带 `1e-3` 的小容差（以 `d < R - 1e-3` 判定入侵），因为轨迹点是
`float`，绕行弧上的点经过 csv 往返后会有 ~1e-7 的误差；若用严格 `d < R`，
这些贴边点会被判成"仍在圆内"而反复二次绕行。加容差后避障对自身输出**幂等**。

### 绕行弧走向

安全圆的两个交点之间有两条弧，`avoidance_arc_side` 决定走哪条：

- `short`（默认）：两交点方位角差归一化到 (-π, π] 后的那一侧，即角度跨度小的弧
- `long`：另一侧（跨度大的弧）

哪个方向看起来是「从上面/外面绕」并不固定，要看进出的方位角。动手调时
建议配 `export_avoided_csv` 导出成 csv、画图确认，而不是凭直觉预设。

需要只翻某一根障碍的绕行弧时，用 `avoidance_arc_side_by_obstacle` 按障碍
下标逐项覆盖（顺序与 `avoidance_obstacles` 一致），例如
`["short", "long", "short"]`。它的价值在于：同一次避障里不同障碍可能
需要不同的走向，而全局的 `avoidance_arc_side` 一改就全变。

### 离线导出避障后轨迹

参数 `export_avoided_csv` 非空时，节点在"加载 + 平移 + 绕障"完成后，
把当前点序列转回 ENU 并写入该路径（两行 csv，12 位有效精度）。
留空则不产生任何文件，节点仍为纯计算。

用途：把"平移 + 绕行"离线固化成一条新轨迹，供其他无人机直接跟踪。
它也可以串起来用：加载一个 csv、避障、导出，再把导出结果当作下一次的
输入，逐段叠加处理。

例如 `ref_uav1.csv` 就是两步串出来的：先让 `ref.csv` 在原位置避障导出中间
文件，再让中间文件沿 NED x 平移 2m、避障后导出最终结果。

```bash
PKG=~/ws_ros2/src/drones_pkg
SRC=$PKG/src
PARAMS=$PKG/params/trajectory_follower.yaml

# 第 1 步：ref.csv 在原位置避障 -> 中间文件（2004 -> 2035 点，3 段，默认 short）
ros2 run drones_pkg trajectory_follower --ros-args \
  --params-file $PARAMS \
  -p trajectory_file:=$SRC/ref.csv \
  -p export_avoided_csv:=$SRC/ref_avoided.csv

# 第 2 步：中间文件沿 NED x 平移 2m、避障 -> ref_uav1.csv（2123 点，2 段）
# 按障碍指定弧向：障碍2 (7,7) 用 long 从外侧包，障碍3 (15,4) 用 short 从上绕
ros2 run drones_pkg trajectory_follower --ros-args \
  --params-file $PARAMS \
  -p trajectory_file:=$SRC/ref_avoided.csv \
  -p origin_offset_ned_x:=2.0 \
  -p avoidance_arc_side:=short \
  -p avoidance_arc_side_by_obstacle:="[short, long, short]" \
  -p export_avoided_csv:=$SRC/ref_uav1.csv

# 同一套链条生成 ref_uav2.csv：第 2 步换成 NED x/y 各 -1.414
# （本轮再避障会轮到障碍1（两段）/障碍3，因为平移量不同）
ros2 run drones_pkg trajectory_follower --ros-args \
  --params-file $PARAMS \
  -p trajectory_file:=$SRC/ref_avoided.csv \
  -p origin_offset_ned_x:=-1.414 \
  -p origin_offset_ned_y:=-1.414 \
  -p avoidance_arc_side:=short \
  -p avoidance_arc_side_by_obstacle:="[short, long, short]" \
  -p export_avoided_csv:=$SRC/ref_uav2.csv

# 同一套链条生成 ref_uav3.csv：第 2 步换成 NED x -1.414、y +1.414
# （注意 NED x=North、y=East，对应 ENU 平移 x +1.414、y -1.414）
# 弧向用 [short, short, short]：这里的障碍2 进出点只差 29.5°，若用 long 会从另一侧
# 绕 330°（近整整一圈，多飞 28.5s），short 收成 29.5° 小弧
ros2 run drones_pkg trajectory_follower --ros-args \
  --params-file $PARAMS \
  -p trajectory_file:=$SRC/ref_avoided.csv \
  -p origin_offset_ned_x:=-1.414 \
  -p origin_offset_ned_y:=1.414 \
  -p avoidance_arc_side:=short \
  -p avoidance_arc_side_by_obstacle:="[short, short, short]" \
  -p export_avoided_csv:=$SRC/ref_uav3.csv
```

导出的结果已含平移（在加载时烘入），因此 `uav_config.py` 中 uav1/uav2/uav3 指向
各自的 ref_uavN.csv 时偏移为 `0.0`（不能再加，否则平移两次）。

以下参数仅供 `trajectory_visualizer` 使用：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `uav_namespaces` | `["/fmu"]` | 要可视化的 PX4 命名空间列表，每项发一条独立航迹话题 |
| `frame_id` | `map` | RViz 可视坐标系 |
| `publish_rate_hz` | `10.0` | 航迹重发频率 [Hz] |
| `path_max_points` | `5000` | 每条航迹最多保留的点数 |

## 状态机

```
WAIT_SYNC      等待 EKF 位置有效
  -> SEND_SETPOINT  持续发 setpoint，建立 OFFBOARD 心跳
  -> OFFBOARD       发送解锁 + 切换 OFFBOARD，等待确认
  -> APPROACH       从当前位置限速飞向轨迹起点（含爬升到目标高度）
  -> TRACK          按 dt 逐点播放轨迹（线性插值）
  -> HOLD / RTL     轨迹结束后的收尾动作
```

**安全说明**：`WAIT_SYNC` 阶段若 60s 内收不到有效位置估计，节点进入
`FAILSAFE` 并停止控制指令。切 OFFBOARD 阶段若 10s 内未确认，会重发
`arm` + `OFFBOARD` 命令。
