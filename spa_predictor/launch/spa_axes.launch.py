# Launches three spa_axis_node instances (see spa_axis_node.cpp) covering
# x/North, y/East, and heave/Down -- the axis parameter each needs is the
# only thing that differs between them; output_topic is left at its
# axis-derived default (/pad/spa/x_prediction, .../y_prediction,
# .../heave_prediction), so all three publish without colliding.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/pad/fmu/out/vehicle_odometry',
        description=(
            'px4_msgs/VehicleOdometry source shared by all three axis '
            'predictors -- pad_motion_gazebo/PadMotionPlugin in simulation, '
            'or a real vision-derived source.'
        ),
    )
    input_topic = LaunchConfiguration('input_topic')

    def axis_node(axis, name):
        return Node(
            package='spa_predictor',
            executable='spa_axis_node',
            # Distinct names -- ROS2 requires unique node names in the same
            # namespace, and three spa_axis_node processes with no override
            # would otherwise all default to "spa_axis_node" and collide.
            name=name,
            output='screen',
            parameters=[{
                'axis': axis,
                'input_topic': input_topic,
            }],
        )

    return LaunchDescription([
        input_topic_arg,
        axis_node(0, 'spa_axis_node_x'),
        axis_node(1, 'spa_axis_node_y'),
        axis_node(2, 'spa_axis_node_heave'),
    ])
