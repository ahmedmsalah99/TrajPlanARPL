from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traj_gen_test',
            executable='offboard_bridge',
            name='px4_offboard_bridge',
            output='screen',
            parameters=[{
                'device': '/quadrotor',
                'offboard_rate_hz': 50.0,
                'command_timeout_s': 0.5,
            }],
        ),
    ])
