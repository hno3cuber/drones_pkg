from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

# 同目录的公共配置：四架无人机的身份与参考轨迹的唯一事实源
sys.path.insert(0, os.path.dirname(__file__))
from uav_config import UAVS, UAV_ORDER  # noqa: E402


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

    # 入参顺序与 UAVS 表中的字段一一对应（namespace, sys_id, csv, ned_x_offset）。
    # 改动 uav_config.py 的字段时，这里必须同步，否则 * 解包会静默错位。
    def follower(name, namespace, sys_id, csv_name, ned_x_offset):
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
                    # NED x（North）方向整体平移，用于多架共用同一 csv 时错开
                    'origin_offset_ned_x': ned_x_offset,
                },
            ],
        )

    nodes = [
        follower(name, *UAVS[name])
        for name in UAV_ORDER
    ]
    # 四条实飞航迹：一个 visualizer 订阅全部命名空间
    namespaces = [UAVS[name][0] for name in UAV_ORDER]
    nodes.append(Node(
        package='drones_pkg',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        output='screen',
        parameters=[params_file, {'uav_namespaces': namespaces}],
    ))

    return LaunchDescription(nodes)
