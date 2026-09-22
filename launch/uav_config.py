"""四架无人机的身份与参考轨迹：全包唯一的事实源。

单独放在 launch/ 下，四个 launch 文件通过同目录 import 共用这一份配置，
避免同一个事实在多处重复维护。改一架无人机只改这里。

字段（四元组）：
    uav0: (node_namespace, target_system, trajectory_csv, ned_x_offset)
      - node_namespace : PX4 实例命名空间（/fmu, /px4_1/fmu ...）
      - target_system  : PX4 系统 ID（MAV_SYS_ID），arm/offboard/rtl 的目标
      - trajectory_csv : share/drones_pkg/src 下的参考轨迹文件名
      - ned_x_offset   : 该机相对轨迹的 NED x（North）方向平移 [m]

多架可以共用同一份 csv，用 ned_x_offset 错开。uav1 走的是另一条路：
它的平移已被烘入 ref_uav1.csv（由 ref.csv 先避障、再沿 NED x 平移 2m、
再避障后导出），所以此处偏移为 0。生成方式见 README「离线导出避障后轨迹」。
"""

UAVS = {
    'uav0': ('/fmu', 1, 'ref.csv', 0.0),
    'uav1': ('/px4_1/fmu', 2, 'ref_uav1.csv', 0.0),
    'uav2': ('/px4_2/fmu', 3, 'uav2.csv', 0.0),
    'uav3': ('/px4_3/fmu', 4, 'uav3.csv', 0.0),
}

# 多机按固定顺序起节点，保证日志与话题顺序稳定
UAV_ORDER = ['uav0', 'uav1', 'uav2', 'uav3']
