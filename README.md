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

多机（四架，一次启动）：

```bash
ros2 launch drones_pkg trajectory_follower_multi.launch.py
```

多机模式下四架的身份与轨迹对应关系（事实源为 `launch/uav_config.py`）：

| 节点名 | node_namespace | target_system | 轨迹 | NED x 偏移 |
| --- | --- | --- | --- | --- |
| trajectory_follower_uav0 | `/fmu` | 1 | ref.csv | 0 |
| trajectory_follower_uav1 | `/px4_1/fmu` | 2 | ref.csv | +2.0 |
| trajectory_follower_uav2 | `/px4_2/fmu` | 3 | uav2.csv | 0 |
| trajectory_follower_uav3 | `/px4_3/fmu` | 4 | uav3.csv | 0 |

uav1 与 uav0 共用 `ref.csv`，靠 `origin_offset_ned_x = 2.0` 沿 NED 的
x 轴（North）整体平移 2m，避免同轨迹相撞。

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
| `origin_offset_ned_x` | `0.0` | 轨迹沿 NED x 轴（North）的整体平移 [m]；多机共用同一 csv 时用它错开（uav1 为 2.0） |
| `origin_offset_ned_y` | `0.0` | 轨迹沿 NED y 轴（East）的整体平移 [m] |
| `approach_speed` | `2.0` | 飞向轨迹起点的速度 [m/s] |
| `setpoint_rate_hz` | `20.0` | setpoint 发布频率 [Hz]，需 > 2Hz |
| `approach_tolerance` | `0.5` | 判定到达轨迹起点的位置容差 [m] |
| `loop_trajectory` | `false` | 是否循环播放轨迹 |
| `avoidance_enable` | `true` | 避障总开关，`false` 时完全按参考轨迹飞 |
| `avoidance_obstacles` | `[4,4, 7,7, 15,4]` | 障碍圆心扁平数组 `[x0,y0,x1,y1,...]`（ENU） |
| `avoidance_radius` | `1.5` | 安全圆半径 [m]，轨迹进入该半径内即沿圆弧绕行 |
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
