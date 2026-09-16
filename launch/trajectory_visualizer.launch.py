from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """轨迹跟随 + RViz 可视化的组合启动。

    两个节点共用同一份 params 文件，因此 RViz 里画出的参考轨迹与无人机实际
    跟踪的目标严格一致。

    用法:
        ros2 launch drones_pkg trajectory_visualizer.launch.py
        ros2 launch drones_pkg trajectory_visualizer.launch.py visualizer_only:=true
    """
    pkg_share = get_package_share_directory('drones_pkg')
    default_params = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')

    params_file = LaunchConfiguration('params_file')
    visualizer_only = LaunchConfiguration('visualizer_only')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件（两个节点共用）'),
        DeclareLaunchArgument(
            'visualizer_only', default_value='false',
            description='true 时只启动可视化节点，不启动控制节点'),

        Node(
            package='drones_pkg',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(PythonExpression(['not ', visualizer_only])),
        ),

        Node(
            package='drones_pkg',
            executable='trajectory_visualizer',
            name='trajectory_visualizer',
            output='screen',
            parameters=[params_file],
        ),
    ])
