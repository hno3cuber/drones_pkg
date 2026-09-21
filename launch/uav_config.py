"""四架无人机的身份与参考轨迹：全包唯一的事实源。

单独放在 launch/ 下，四个 launch 文件通过同目录 import 共用这一份配置，
避免同一个事实在多处重复维护。改一架无人机只改这里。

字段：
    uav0: (node_namespace, target_system, trajectory_csv)
      - node_namespace : PX4 实例命名空间（/fmu, /px4_1/fmu ...）
      - target_system  : PX4 系统 ID（MAV_SYS_ID），arm/offboard/rtl 的目标
      - trajectory_csv : share/drones_pkg/src 下的参考轨迹文件名
"""

UAVS = {
    'uav0': ('/fmu', 1, 'ref.csv'),
    'uav1': ('/px4_1/fmu', 2, 'uav1.csv'),
    'uav2': ('/px4_2/fmu', 3, 'uav2.csv'),
    'uav3': ('/px4_3/fmu', 4, 'uav3.csv'),
}

# 多机按固定顺序起节点，保证日志与话题顺序稳定
UAV_ORDER = ['uav0', 'uav1', 'uav2', 'uav3']
