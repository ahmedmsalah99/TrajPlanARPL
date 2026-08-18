cmake_minimum_required(VERSION 3.8)
project(spa_predictor)

# Compile as C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate this package's interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/SpaPrediction.msg
  msg/LandingGate.msg
  DEPENDENCIES std_msgs
)

# Typesupport target so this package can use its own messages from C++
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

set(dependencies
  rclcpp
  std_msgs
  px4_msgs
)

# Library
# NOTE: rosidl_generate_interfaces creates a target named ${PROJECT_NAME}, so the
# C++ library must use a different target name. Keep the output file name unchanged.
add_library(spa_predictor_lib
  src/spa_predictor.cpp
)
set_target_properties(spa_predictor_lib PROPERTIES OUTPUT_NAME spa_predictor)

target_include_directories(spa_predictor_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
)

# spa_predictor.h/.cpp are pure Eigen (no rclcpp dependency) -- but the
# library still needs px4_msgs's generated headers to build cleanly as part
# of this package's dependency graph, and rclcpp for consistency with the
# rest of this workspace's packages.
ament_target_dependencies(spa_predictor_lib ${dependencies})

# Executable: standalone SPA per-axis predictor node (see spa_axis_node.cpp)
# -- run one instance per axis (x/y/heave) via its `axis` parameter. Not
# linked into ros_traj_gen_utils/traj_exe -- this is Phase 2 of the SPA
# rollout (validate against a known/simulated signal before anything
# consumes its output for real).
add_executable(spa_axis_node src/spa_axis_node.cpp)
ament_target_dependencies(spa_axis_node ${dependencies})
target_link_libraries(spa_axis_node
  spa_predictor_lib
  ${cpp_typesupport_target}
)

# Executable: standalone SPA per-angle predictor node (see
# spa_angle_node.cpp) -- run one instance per angle (roll/pitch) via its
# `angle` parameter. Sibling to spa_axis_node above.
add_executable(spa_angle_node src/spa_angle_node.cpp)
ament_target_dependencies(spa_angle_node ${dependencies})
target_link_libraries(spa_angle_node
  spa_predictor_lib
  ${cpp_typesupport_target}
)

# Executable: Go/NoGo landing gate (see spa_landing_gate_node.cpp) -- built
# from the five predictor nodes' outputs, not a signal predictor itself
# (doesn't link spa_predictor_lib), but does need this package's own
# generated message type, hence the typesupport link below.
add_executable(spa_landing_gate_node src/spa_landing_gate_node.cpp)
ament_target_dependencies(spa_landing_gate_node ${dependencies})
target_link_libraries(spa_landing_gate_node
  ${cpp_typesupport_target}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS spa_predictor_lib
  EXPORT export_spa_predictor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS spa_axis_node spa_angle_node spa_landing_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Diagnostic-only CSV logger (see its own docstring). Installed as a
# PROGRAM rather than converting this ament_cmake package to ament_python,
# so it's runnable via `ros2 run`/launchable via spa_axes.launch.py like
# any other node. spa_eval_analyze.py (the offline plotting/stats half)
# is DELIBERATELY NOT installed here -- it's not a ROS node, has no need
# to be found via the ament index, and is meant to be run straight from
# the source tree (scripts/spa_eval_analyze.py), not a colcon install
# path.
install(
  PROGRAMS
    scripts/spa_eval_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(spa_predictor_lib)
ament_export_dependencies(rosidl_default_runtime ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
