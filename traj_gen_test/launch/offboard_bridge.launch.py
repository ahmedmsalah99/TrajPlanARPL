from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_traj_gen_utils',
            executable='so3_control_node',
            name='so3_control_node',
            output='screen',
            parameters=[{
                'device': '/quadrotor',
                'mass': 1.0,
                'gravity': 9.81,
                # Normalized-throttle fraction PX4 uses to hover -- calibrate
                # this per vehicle (e.g. QGC's "Hover Throttle" / MPC_THR_HOVER)
                # before flight; the default is a placeholder.
                'hover_thrust': 0.5,
                'kR': [8.0, 8.0, 3.0],
                'max_body_rate': 6.0,
                'control_rate_hz': 100.0,
                'command_timeout_s': 0.25,
            }],
        ),
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
