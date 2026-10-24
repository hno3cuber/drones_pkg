from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uav_config import UAVS, UAV_ORDER  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    """启动时求值参数：yaml 是基底，显式传入的 params_file 覆盖它。"""
    pkg_share = get_package_share_directory('drones_pkg')

    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')

    # 命名空间列表以 uav_config.py 为唯一事实源，避免与 launch 各写一份
    uav_namespaces = [UAVS[name][0] for name in UAV_ORDER]

    params = [params_file, {'uav_namespaces': uav_namespaces}]

    return [
        Node(
            package='drones_pkg',
            executable='uav_link_visualizer',
            name='uav_link_visualizer',
            output='screen',
            parameters=params,
        ),
    ]


def generate_launch_description():
    """可视化：订阅四机实时位置，各打一个点，再把点两两连起来。

    每架只取启动后第一帧有效位置作快照，之后点固定不动。命名空间列表
    取自 launch/uav_config.py（与 follower / visualizer 同源）。
    点大小、线宽、连接方式、颜色等来自 params 文件
    （params/trajectory_follower.yaml 的 uav_link_visualizer 段）。

    用法:
        ros2 launch drones_pkg uav_link_visualizer.launch.py
        # 需先启动仿真 / 至少让各机位置话题在发布，否则节点会等到超时
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='参数文件路径（留空则用包内默认 yaml）'),

        OpaqueFunction(function=_launch_setup),
    ])
