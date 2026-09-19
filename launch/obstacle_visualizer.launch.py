from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """只启动静态圆柱障碍物可视化节点，参数全部来自 params 文件。

    障碍物位置、半径、高度等都在 params/trajectory_follower.yaml 的
    obstacle_visualizer 段里改。这个 launch 只负责把那份 yaml 喂给节点。

    用法:
        ros2 launch drones_pkg obstacle_visualizer.launch.py
        ros2 launch drones_pkg obstacle_visualizer.launch.py params_file:=/path/to/other.yaml
    """
    pkg_share = get_package_share_directory('drones_pkg')
    default_params = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')
    default_path = os.path.join(pkg_share, 'src', 'ref.csv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件路径'),
        DeclareLaunchArgument(
            'path_file', default_value=default_path,
            description='参考轨迹 csv 路径（两行 x/y）'),

        Node(
            package='drones_pkg',
            executable='obstacle_visualizer',
            name='obstacle_visualizer',
            output='screen',
            # 唯一来源就是 yaml，launch 不再塞任何默认值去覆盖它。
            parameters=[
                LaunchConfiguration('params_file'),
                {'path_file': LaunchConfiguration('path_file')},
            ],
        ),
    ])
