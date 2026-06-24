import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros_traj_gen_utils'),
        'config',
        'perch_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='ros_traj_gen_utils',
            executable='traj_exe',
            name='traj_exe',
            output='screen',
            parameters=[config],
            prefix=['xterm -e gdb -ex run --args']
        ),
    ])
