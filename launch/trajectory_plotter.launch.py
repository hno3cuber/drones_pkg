from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

# 同目录的公共配置：四架无人机的身份与参考轨迹的唯一事实源
sys.path.insert(0, os.path.dirname(__file__))
from uav_config import UAVS  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    """按 uav 短名解析身份：follower 控制这一架，plotter 订阅这一架的到点信号。"""
    pkg_share = get_package_share_directory('drones_pkg')
    params_file = LaunchConfiguration('params_file').perform(context)
    uav = LaunchConfiguration('uav').perform(context)
    plot_only = LaunchConfiguration('plot_only').perform(context)
    output_png = LaunchConfiguration('output_png').perform(context)

    if uav not in UAVS:
        raise RuntimeError(
            f"未知的 uav='{uav}'，可选：{', '.join(sorted(UAVS))}")

    default_ns, default_sys, default_csv, default_off_x = UAVS[uav]

    ns = LaunchConfiguration('node_namespace').perform(context) or default_ns
    sys_str = LaunchConfiguration('target_system').perform(context)
    sys_id = int(sys_str) if sys_str else default_sys

    off_str = LaunchConfiguration('origin_offset_ned_x').perform(context)
    off_x = float(off_str) if off_str else default_off_x

    # 在 Python 里把字符串转成布尔，别交给 PythonExpression 去求值
    # （'false' 在 Python 里是个未定义的名字，会直接抛 NameError）
    skip_follower = plot_only.strip().lower() in ('true', '1', 'yes')

    trajectory = LaunchConfiguration('trajectory_file').perform(context) or \
        os.path.join(pkg_share, 'src', default_csv)

    nodes = []
    if not skip_follower:
        nodes.append(
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
                        # NED x（North）方向整体平移，用于多架共用同一 csv 时错开
                        'origin_offset_ned_x': off_x,
                    },
                ],
            )
        )

    nodes.append(
        Node(
            package='drones_pkg',
            executable='trajectory_plotter',
            name='trajectory_plotter',
            output='screen',
            # 三条距离曲线画在一张图上，y=safe_distance 处一条红色虚线
            parameters=[
                params_file,
                {
                    'node_namespace': ns,
                    'output_png': output_png,
                },
            ],
        )
    )
    return nodes


def generate_launch_description():
    """单机轨迹跟随 + 到点后绘制距离曲线。

    控制节点与绘图节点共用同一份 params 文件。绘图节点在收到
    <node_namespace>/trajectory_finished 之后弹窗，因此要在起飞前把它启动好，
    才能记下完整的距离序列。

    用法:
        # 默认 uav0：跟随 + 到点后弹窗
        ros2 launch drones_pkg trajectory_plotter.launch.py

        # 已单独在跑 follower，只启动绘图节点
        ros2 launch drones_pkg trajectory_plotter.launch.py uav:=uav1 plot_only:=true

        # 除了弹窗，同时存一份 png
        ros2 launch drones_pkg trajectory_plotter.launch.py \\
            output_png:=/home/hno3/flight.png
    """
    pkg_share = get_package_share_directory('drones_pkg')
    default_params = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'uav', default_value='uav0',
            description='选哪一架：uav0 / uav1 / uav2 / uav3'),
        DeclareLaunchArgument(
            'plot_only', default_value='false',
            description='true 时只启动绘图节点，不启动控制节点'),
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
            'output_png', default_value='',
            description='非空时把图同时保存到该路径（如 ~/flight.png）'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件路径（两个节点共用）'),

        OpaqueFunction(function=_launch_setup),
    ])
