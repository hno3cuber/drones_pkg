"""Gazebo 多机仿真的身份配置：与 launch/uav_config.py 同源。

与 uav_config.py 分开维护的原因：那个文件描述的是「控制侧」身份
（PX4 命名空间、系统 ID、参考轨迹 csv），本文件描述的是「仿真侧」身份
（Gazebo 世界里的出生位姿）。二者一一对应，改一架要同时看两个文件。

字段（与 uav_config.UAVS 同键，便于交叉核对）：
    uav0: (px4_instance, pose_xy)
      - px4_instance : PX4 实例号（决定 MAV_SYS_ID 与 UDP 端口）
      - pose_xy      : Gazebo 世界坐标下的出生点 (x, y)，ENU，z 由模型自身
                       的 <pose>0 0 .24</pose> 决定，yaw 一律 0
"""

# 出生点沿 x 轴排开，y 全为 0，与用户给定的布局一致。
UAV_POSES = {
    'uav0': (0, (0.0, 0.0)),
    'uav1': (1, (1.0, 0.0)),
    'uav2': (2, (-1.0, 0.0)),
    'uav3': (3, (2.0, 0.0)),
}

# 起进程的顺序：主实例（instance 0，负责起 gz server + world）必须排第一，
# 其余实例以 PX4_GZ_STANDALONE=1 连上去。
UAV_ORDER = ['uav0', 'uav1', 'uav2', 'uav3']

# 仿真侧公共参数
AIRFRAME = '4001'          # 4001_gz_x500
SIM_MODEL = 'gz_x500'      # PX4_SIM_MODEL
WORLD_NAME = 'obstacles_world'
