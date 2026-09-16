from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('drones_pkg')
    params_file = os.path.join(pkg_share, 'params', 'trajectory_follower.yaml')

    return LaunchDescription([
        Node(
            package='drones_pkg',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[params_file],
        ),
    ])
