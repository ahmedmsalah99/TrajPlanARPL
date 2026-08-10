# Launches three spa_axis_node instances (see spa_axis_node.cpp) covering
# x/North, y/East, and heave/Down -- output_topic is left at each one's
# axis-derived default (/pad/spa/x_prediction, .../y_prediction,
# .../heave_prediction), so all three publish without colliding.
#
# Mode-detection tuning (t_fft_s, peak_sensitivity, max_modes, f_min_hz,
# f_max_hz) is exposed as two GROUPS of launch arguments -- "horizontal"
# (shared by x and y) and "heave" -- rather than per-axis, since x/y and
# heave are physically different dynamics that field data showed needing
# different tuning (heave: an intermittently-detected secondary mode;
# horizontal: a real harmonic sitting right at the peak_sensitivity
# threshold, and a drifting dominant frequency between windows), while x
# and y themselves have shown no evidence yet of needing separate tuning
# from each other.
#
# Each of these arguments defaults to an EMPTY string, meaning "don't
# override -- use spa_axis_node's own Config default" (see
# spa_predictor.h), not a copy of that default baked in here -- avoids a
# second, driftable copy of those numbers living in this launch file.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (param name, is this an integer parameter -- only max_modes is; the rest
# are doubles)
TUNABLE_PARAMS = [
    ('t_fft_s', False),
    ('peak_sensitivity', False),
    ('max_modes', True),
    ('f_min_hz', False),
    ('f_max_hz', False),
]


def _group_overrides(context, prefix):
    overrides = {}
    for name, is_int in TUNABLE_PARAMS:
        value = LaunchConfiguration(f'{prefix}_{name}').perform(context)
        if value != '':
            overrides[name] = int(value) if is_int else float(value)
    return overrides


def _launch_setup(context, *args, **kwargs):
    input_topic = LaunchConfiguration('input_topic')
    horizontal_overrides = _group_overrides(context, 'horizontal')
    heave_overrides = _group_overrides(context, 'heave')

    def axis_node(axis, name, overrides):
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
                **overrides,
            }],
        )

    return [
        axis_node(0, 'spa_axis_node_x', horizontal_overrides),
        axis_node(1, 'spa_axis_node_y', horizontal_overrides),
        axis_node(2, 'spa_axis_node_heave', heave_overrides),
    ]


def generate_launch_description():
    declares = [
        DeclareLaunchArgument(
            'input_topic',
            default_value='/pad/fmu/out/vehicle_odometry',
            description=(
                'px4_msgs/VehicleOdometry source shared by all three axis '
                'predictors -- pad_motion_gazebo/PadMotionPlugin in '
                'simulation, or a real vision-derived source.'
            ),
        ),
    ]
    for prefix, label in (('horizontal', 'x and y'), ('heave', 'heave/z')):
        for name, _ in TUNABLE_PARAMS:
            declares.append(DeclareLaunchArgument(
                f'{prefix}_{name}',
                default_value='',
                description=(
                    f"Override {name} for the {label} predictor(s). Empty "
                    f"(default) leaves spa_axis_node's own Config default "
                    f"in effect -- see spa_predictor.h."
                ),
            ))
    return LaunchDescription(declares + [OpaqueFunction(function=_launch_setup)])
