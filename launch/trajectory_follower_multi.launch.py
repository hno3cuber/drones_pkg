from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# 四架无人机的身份：命名空间、系统 ID、参考轨迹
UAVS = [
    ('uav0', '/fmu', 1, 'uav0.csv'),
    ('uav1', '/px4_1/fmu', 2, 'uav1.csv'),
    ('uav2', '/px4_2/fmu', 3, 'uav2.csv'),
    ('uav3', '/px4_3/fmu', 4, 'uav3.csv'),
]


def generate_launch_description():
    """四架无人机的 OFFBOARD 轨迹跟踪 + 实飞航迹可视化。

    每个 follower 实例用 parameters 覆盖三项：
      - node_namespace : PX4 实例命名空间（/fmu, /px4_1/fmu, /px4_2/fmu, /px4_3/fmu）
      - target_system  : PX4 系统 ID（1, 2, 3, 4）
      - trajectory_file: 各自的参考轨迹 csv

    visualizer 订阅全部四架的实飞位置，为每架发一条独立 topic
    （~/uav0/flown_path ... ~/uav3/flown_path），便于在 RViz 中单独配色。
    """
    pkg_share = get_package_share_directory('drones_pkg')
    params_file = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')
    src_dir = os.path.join(pkg_share, 'src')

    def follower(name, namespace, sys_id, csv_name):
        return Node(
            package='drones_pkg',
            executable='trajectory_follower',
            name=name,
            output='screen',
            parameters=[
                params_file,
                {
                    'node_namespace': namespace,
                    'target_system': sys_id,
                    'trajectory_file': os.path.join(src_dir, csv_name),
                },
            ],
        )

    nodes = [
        follower('trajectory_follower_' + name, ns, sys_id, csv)
        for name, ns, sys_id, csv in UAVS
    ]

    # 四条实飞航迹：一个 visualizer 订阅全部命名空间
    namespaces = [ns for _, ns, _, _ in UAVS]
    nodes.append(Node(
        package='drones_pkg',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        output='screen',
        parameters=[params_file, {'uav_namespaces': namespaces}],
    ))

    return LaunchDescription(nodes)
