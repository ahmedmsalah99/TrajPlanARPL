from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traj_gen_test',
            executable='dummy_publisher',
            name='traj_gen_dummy_publisher',
            output='screen',
            parameters=[{
                'device': '/quadrotor',
                'frame_id': 'odom',
                'odom_rate_hz': 50.0,
                'publish_tag': True,
                'waypoint_period_s': 0.1,
                'waypoints': [1.0, 0.0, 1.0, 0.0,
                              2.0, 1.0, 1.0, 0.0,
                              3.0, 0.0, 1.0, 0.0],
                'tag_pose' : [3.0, 0.0, 1.0, 0.0,3.14,0.0],
                'odom_ned':[-5.0,0,0],
                'publish_tf': True,      # publish TF so RViz can resolve the frames
                'fixed_frame': 'map',    # set RViz Fixed Frame to this
            }],
        ),
    ])
