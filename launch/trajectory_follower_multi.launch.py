from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """四架无人机的 OFFBOARD 轨迹跟踪。

    同一份 params 文件作为基线，每个实例用 parameters 覆盖三项：
      - node_namespace : PX4 实例命名空间（/fmu, /px4_1/fmu, /px4_2/fmu, /px4_3/fmu）
      - target_system  : PX4 系统 ID（1, 2, 3, 4）
      - trajectory_file: 各自的参考轨迹 csv

    对应关系:
      uav0 -> /fmu       sys=1 -> uav0.csv
      uav1 -> /px4_1/fmu sys=2 -> uav1.csv
      uav2 -> /px4_2/fmu sys=3 -> uav2.csv
      uav3 -> /px4_3/fmu sys=4 -> uav3.csv
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

    return LaunchDescription([
        follower('trajectory_follower_uav0', '/fmu', 1, 'uav0.csv'),
        follower('trajectory_follower_uav1', '/px4_1/fmu', 2, 'uav1.csv'),
        follower('trajectory_follower_uav2', '/px4_2/fmu', 3, 'uav2.csv'),
        follower('trajectory_follower_uav3', '/px4_3/fmu', 4, 'uav3.csv'),
    ])
