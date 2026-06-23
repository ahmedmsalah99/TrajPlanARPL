cmake_minimum_required(VERSION 3.8)
project(ros_traj_gen_utils)

# Compile as C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(traj_gen REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(trackers_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
# traj_gen is a static library whose exported interface links the Boost::thread
# and Threads::Threads imported targets, so consumers must define them too.
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Threads REQUIRED)

# Generate this package's interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Polynomial.msg
  msg/StdTraj.msg
  msg/CuboidMap.msg
  msg/AprilTagDetection.msg
  msg/AprilTagDetectionArray.msg
  DEPENDENCIES std_msgs geometry_msgs
)

# Typesupport target so this package can use its own messages from C++
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

set(dependencies
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  traj_gen
  quadrotor_msgs
  trackers_msgs
  px4_msgs
)

# Library
# NOTE: rosidl_generate_interfaces creates a target named ${PROJECT_NAME}, so the
# C++ library must use a different target name. Keep the output file name unchanged.
add_library(ros_traj_gen_utils_lib
  src/pub_sub/ros_waypoint_utils.cpp
  src/pub_sub/ros_traj_utils.cpp
  src/pub_sub/apriltag_utils.cpp #Move to Apriltag package later
  src/pub_sub/odom_utils.cpp
  src/pub_sub/poscmd_publisher.cpp
  src/cuboid_map/rect_node.cpp
  src/cuboid_map/ros_cuboid_utils.cpp
  src/ros_replanner_utils.cpp
)
set_target_properties(ros_traj_gen_utils_lib PROPERTIES OUTPUT_NAME ros_traj_gen_utils)

target_include_directories(ros_traj_gen_utils_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(ros_traj_gen_utils_lib ${dependencies})
target_link_libraries(ros_traj_gen_utils_lib ${cpp_typesupport_target})

# Executable
add_executable(traj_exe src/traj_manager.cpp)
ament_target_dependencies(traj_exe ${dependencies})
target_link_libraries(traj_exe
  ros_traj_gen_utils_lib
  ${cpp_typesupport_target}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ros_traj_gen_utils_lib
  EXPORT export_ros_traj_gen_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS traj_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ros_traj_gen_utils_lib)
ament_export_dependencies(rosidl_default_runtime ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
