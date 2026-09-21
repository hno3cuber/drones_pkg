from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

# 同目录的公共配置：四架无人机的身份与参考轨迹的唯一事实源
sys.path.insert(0, os.path.dirname(__file__))
from uav_config import UAVS  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    """按 uav 短名解析身份：follower 控制这一架，visualizer 只订阅这一架。"""
    pkg_share = get_package_share_directory('drones_pkg')
    params_file = LaunchConfiguration('params_file').perform(context)
    uav = LaunchConfiguration('uav').perform(context)
    visualizer = LaunchConfiguration('visualizer').perform(context)

    if uav not in UAVS:
        raise RuntimeError(
            f"未知的 uav='{uav}'，可选：{', '.join(sorted(UAVS))}")

    default_ns, default_sys, default_csv = UAVS[uav]

    # 三个覆盖参数留空时回退到 uav 表中的默认值
    ns = LaunchConfiguration('node_namespace').perform(context) or default_ns
    sys_str = LaunchConfiguration('target_system').perform(context)
    sys_id = int(sys_str) if sys_str else default_sys

    trajectory = LaunchConfiguration('trajectory_file').perform(context) or \
        os.path.join(pkg_share, 'src', default_csv)

    nodes = [
        Node(
            package='drones_pkg',
            executable='trajectory_follower',
            name='trajectory_follower_' + uav,
            output='screen',
            parameters=[
                params_file,
                {
                    'node_namespace': ns,
                    'target_system': sys_id,
                    'trajectory_file': trajectory,
                },
            ],
        ),
        Node(
            package='drones_pkg',
            executable='trajectory_visualizer',
            name='trajectory_visualizer',
            output='screen',
            # 只可视化所选这一架，话题即为 ~/<ns 转写>/flown_path
            parameters=[params_file, {'uav_namespaces': [ns]}],
            condition=IfCondition(visualizer),
        ),
    ]
    return nodes


def generate_launch_description():
    """单机轨迹跟随（默认带可视化），任选一架。

    一条命令同时起控制节点与可视化节点，二者共用同一份 params 文件，
    且都只针对 uav 指定的那一架。真机飞行等场景可用 visualizer:=false 只跑控制。

    用法:
        # 默认 uav0，控制 + 可视化一起起
        ros2 launch drones_pkg trajectory_follower.launch.py

        # 只跑 uav2 的控制 + 可视化
        ros2 launch drones_pkg trajectory_follower.launch.py uav:=uav2

        # 只跑控制，不起可视化节点
        ros2 launch drones_pkg trajectory_follower.launch.py \\
            uav:=uav2 visualizer:=false

        # 单独覆盖身份（不常用）
        ros2 launch drones_pkg trajectory_follower.launch.py \\
            uav:=uav1 node_namespace:=/fmu target_system:=1
    """
    pkg_share = get_package_share_directory('drones_pkg')
    default_params = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'uav', default_value='uav0',
            description='选哪一架：uav0 / uav1 / uav2 / uav3'),
        DeclareLaunchArgument(
            'visualizer', default_value='true',
            description='true 时同时启动可视化节点，false 时只跑控制'),
        DeclareLaunchArgument(
            'node_namespace', default_value='',
            description='覆盖 PX4 命名空间（留空则按 uav 自动推断）'),
        DeclareLaunchArgument(
            'target_system', default_value='',
            description='覆盖 PX4 系统 ID（留空则按 uav 自动推断）'),
        DeclareLaunchArgument(
            'trajectory_file', default_value='',
            description='覆盖轨迹文件绝对路径（留空则用 src/<uav>.csv）'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件路径（两个节点共用）'),

        OpaqueFunction(function=_launch_setup),
    ])
