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
                'max_segment_len': 0.1,  # densify waypoints so legs are <= 0.1 m
                # NED waypoints: z is Down, so negative z = altitude above origin
                'waypoints': [
                    # 0.0, 0.0, 0.0, 0.0,
                            #   2.0, 0.0, -1.0, 0.0,
                              3.0, 0.0, -1.0, 0.0],
                'tag_pose' : [3.0, 0.0, -1.0, 0.0,0.3,0.0],
                'odom_ned':[0.0, 0.0, 0.0,0],
                # 'static' reports odom_ned forever (no real motion feedback);
                # 'advance' echoes the planner's own position_cmd back as odom,
                # so replans see genuine progress toward the goal.
                'odom_mode': 'advance',
                'publish_tf': True,      # publish TF so RViz can resolve the frames
                'fixed_frame': 'map',    # set RViz Fixed Frame to this
                'rviz_enu_flip': True,   # flip fixed_frame->planner so NED shows upright
            }],
        ),
    ])
