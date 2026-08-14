# Launches all five SPA predictors covering the pad's state: three
# spa_axis_node instances (x/North, y/East, heave/Down -- see
# spa_axis_node.cpp) and two spa_angle_node instances (roll, pitch -- see
# spa_angle_node.cpp). output_topic is left at each one's own derived
# default (/pad/spa/x_prediction, .../y_prediction, .../heave_prediction,
# .../roll_prediction, .../pitch_prediction), so all five publish without
# colliding. Also launches spa_eval_logger.py (the CSV logger, see its own
# docstring) so a ground-truth + predictions dataset is always being
# recorded (to spa_eval_logger.py's own default /tmp/spa_eval, overwritten
# every run -- see spa_eval_analyze.py for the matching offline analysis,
# run separately/afterward, straight from the source tree).
#
# Mode-detection tuning (t_fft_s, peak_sensitivity, max_modes, f_min_hz,
# f_max_hz) is exposed as three GROUPS of launch arguments -- "horizontal"
# (shared by x and y), "heave", and "angular" (shared by roll and pitch)
# -- rather than per-signal, grouped by physically-similar dynamics: x/y
# and heave are known to need different tuning from field data (heave: an
# intermittently-detected secondary mode; horizontal: a real harmonic
# sitting right at the peak_sensitivity threshold, and a drifting dominant
# frequency between windows); roll/pitch get their own group since tilt
# dynamics have no reason to share tuning with translation, though no
# field data exists yet to say whether roll and pitch need to differ from
# EACH OTHER -- start them shared, split if evidence says otherwise (same
# reasoning x/y were kept shared under).
#
# Each of these arguments defaults to an EMPTY string, meaning "don't
# override -- use the node's own Config default" (see spa_predictor.h),
# not a copy of that default baked in here -- avoids a second, driftable
# copy of those numbers living in this launch file.
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
    angular_overrides = _group_overrides(context, 'angular')

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

    def angle_node(angle, name, overrides):
        return Node(
            package='spa_predictor',
            executable='spa_angle_node',
            name=name,
            output='screen',
            parameters=[{
                'angle': angle,
                'input_topic': input_topic,
                **overrides,
            }],
        )

    logger_node = Node(
        package='spa_predictor',
        executable='spa_eval_logger.py',
        name='spa_eval_logger',
        output='screen',
        parameters=[{
            # Wired to the SAME input_topic the axis predictors read, so an
            # override here can't silently leave the logger recording
            # ground truth from the old default while the predictors
            # switched to a different source. x_topic/y_topic/heave_topic
            # and output_dir are left at spa_eval_logger.py's own defaults,
            # which already match this launch file's own defaults for the
            # axis predictors' output_topic.
            'truth_topic': input_topic,
        }],
    )

    return [
        axis_node(0, 'spa_axis_node_x', horizontal_overrides),
        axis_node(1, 'spa_axis_node_y', horizontal_overrides),
        axis_node(2, 'spa_axis_node_heave', heave_overrides),
        angle_node(0, 'spa_angle_node_roll', angular_overrides),
        angle_node(1, 'spa_angle_node_pitch', angular_overrides),
        logger_node,
    ]


def generate_launch_description():
    declares = [
        DeclareLaunchArgument(
            'input_topic',
            default_value='/pad/fmu/out/vehicle_odometry',
            description=(
                'px4_msgs/VehicleOdometry source shared by all five '
                'predictors -- pad_motion_gazebo/PadMotionPlugin in '
                'simulation, or a real vision-derived source.'
            ),
        ),
    ]
    for prefix, label in (('horizontal', 'x and y'), ('heave', 'heave/z'), ('angular', 'roll and pitch')):
        for name, _ in TUNABLE_PARAMS:
            declares.append(DeclareLaunchArgument(
                f'{prefix}_{name}',
                default_value='',
                description=(
                    f"Override {name} for the {label} predictor(s). Empty "
                    f"(default) leaves that node's own Config default in "
                    f"effect -- see spa_predictor.h."
                ),
            ))
    return LaunchDescription(declares + [OpaqueFunction(function=_launch_setup)])
