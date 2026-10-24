from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    """启动时求值参数：yaml 是基底，显式传入的 path_file 覆盖它。"""
    pkg_share = get_package_share_directory('drones_pkg')

    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')

    path_file = LaunchConfiguration('path_file').perform(context)

    # path_file 留空时什么都不传，交给节点自己回退到包内 src/ref.csv。
    # 这样避免了 yaml 里的空值与 launch 传参互相覆盖。
    params = [params_file]
    if path_file:
        params.append({'path_file': path_file})

    return [
        Node(
            package='drones_pkg',
            executable='obstacle_visualizer',
            name='obstacle_visualizer',
            output='screen',
            parameters=params,
        ),
    ]


def generate_launch_description():
    """只启动静态圆柱障碍物可视化节点。

    障碍物位置、半径、高度、轨迹线样式等都来自 params 文件
    （params/trajectory_follower.yaml 的 obstacle_visualizer 段）。
    轨迹 csv 默认使用包内 src/ref.csv（节点自己按 share 目录查找），
    也可以用 path_file:= 指定别的文件。

    用法:
        ros2 launch drones_pkg obstacle_visualizer.launch.py
        ros2 launch drones_pkg obstacle_visualizer.launch.py path_file:=/abs/path.csv
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='参数文件路径（留空则用包内默认 yaml）'),
        DeclareLaunchArgument(
            'path_file', default_value='',
            description='参考轨迹 csv 路径（留空则用包内 src/ref.csv）'),

        OpaqueFunction(function=_launch_setup),
    ])
