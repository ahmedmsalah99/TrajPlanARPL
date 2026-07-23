from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # so3_control_node owns the whole PX4-facing offboard sequence now
    # (heartbeat, DO_SET_MODE, start_replan, rate setpoints) -- there is no
    # separate offboard_bridge node anymore; running one alongside this would
    # mean two nodes both able to declare /fmu/in/offboard_control_mode,
    # which PX4 only accepts one consistent version of at a time.
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
                'offboard_prime_s': 1.0,
                'offboard_confirm_timeout_s': 5.0,
            }],
        ),
    ])
