# drones_pkg

PX4 OFFBOARD 参考轨迹跟踪节点。读取 `src/uav0.csv` 中的平面参考轨迹，
驱动无人机按该轨迹飞行。

## 依赖

- ROS 2 Humble
- `px4_msgs`（同工作空间内）
- PX4 SITL + Micro XRCE-DDS Agent

## 构建

本包的构建产物默认输出到包内 `_colcon/` 目录（见 `.gitignore`），
如果直接在工作空间根目录构建，用标准方式即可：

```bash
cd ~/ws_ros2
colcon build --packages-select drones_pkg
source install/setup.bash
```

## 轨迹文件格式

`src/uav0.csv` 为 **两行 CSV**：

```
x0,x1,x2,...,xN     <- 第一行：x 坐标
y0,y1,y2,...,yN     <- 第二行：y 坐标
```

默认按 **ENU**（x=东, y=北）解释，内部转换为 PX4 的 **NED**。
若文件本身是 NED，把参数 `frame_is_enu` 设为 `false`。

## 运行

```bash
ros2 launch drones_pkg trajectory_follower.launch.py
```

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `trajectory_file` | `""` | 轨迹文件路径，空则用包内 `src/uav0.csv` |
| `dt` | `0.1` | 相邻点时间步长 [s]（csv 无时间列时使用） |
| `frame_is_enu` | `true` | 输入是否为 ENU |
| `flight_altitude` | `5.0` | 目标飞行高度 [m] |
| `origin_offset_ned_x` | `0.0` | 轨迹原点相对起飞点的 NED x 偏移 [m] |
| `origin_offset_ned_y` | `0.0` | 轨迹原点相对起飞点的 NED y 偏移 [m] |
| `approach_speed` | `2.0` | 飞向轨迹起点的速度 [m/s] |
| `setpoint_rate_hz` | `20.0` | setpoint 发布频率 [Hz]，需 > 2Hz |
| `approach_tolerance` | `0.5` | 判定到达轨迹起点的位置容差 [m] |
| `loop_trajectory` | `false` | 是否循环播放轨迹 |
| `end_action` | `hold` | 轨迹结束后动作：`hold` 悬停 / `rtl` 返航 |

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
