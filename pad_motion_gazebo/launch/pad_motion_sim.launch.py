"""Launch the pad-motion Gazebo test world (see worlds/pad_motion_test.sdf).

    ros2 launch pad_motion_gazebo pad_motion_sim.launch.py

Spawns gz-sim with a single kinematically-heaving pad (PadMotionPlugin,
embedded in the pad's own <model> SDF -- see worlds/pad_motion_test.sdf)
publishing ground-truth px4_msgs/VehicleOdometry on
/pad/fmu/out/vehicle_odometry. Feed that into ros_traj_gen_utils'
spa_heave_node to validate the SPA predictor against a known signal before
pointing it at a real (vision-derived) source:

    ros2 run ros_traj_gen_utils spa_heave_node

No drone/PX4 SITL is started here -- this is deliberately isolated to just
the pad, for testing the predictor and this plugin on their own.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('pad_motion_gazebo')
    world_path = os.path.join(pkg_share, 'worlds', 'pad_motion_test.sdf')

    # ament installs this package's CMake library target to
    # <install_prefix>/lib/pad_motion_gazebo (a sibling of share/, matching
    # every other package in this workspace -- see e.g.
    # ros_traj_gen_utils/CMakeLists.txt's own install(TARGETS ...) rules),
    # NOT under share/. Derive it from pkg_share rather than hardcoding, so
    # this keeps working under any install prefix.
    install_prefix = os.path.dirname(os.path.dirname(pkg_share))  # strips /share/<pkg>
    plugin_path = os.path.join(install_prefix, 'lib', 'pad_motion_gazebo')
    model_path = os.path.join(pkg_share, 'models')

    # gz-sim's plugin loader searches GZ_SIM_SYSTEM_PLUGIN_PATH for
    # filename="PadMotionPlugin" (-> libPadMotionPlugin.so); resource path
    # lets `<include><uri>model://wave_pad</uri></include>` resolve from
    # OTHER worlds too, not just this package's own test world.
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_path + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_path + ' -r'}.items())

    return LaunchDescription([set_plugin_path, set_resource_path, gz_sim])
