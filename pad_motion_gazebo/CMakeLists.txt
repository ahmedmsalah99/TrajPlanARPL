cmake_minimum_required(VERSION 3.8)
project(pad_motion_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# gz-simN is version-dependent (Harmonic=8, Fortress=7); try the newer one
# first and fall back, so this builds against whichever is installed
# without the caller needing to pin a version.
find_package(gz-sim8 QUIET)
if(gz-sim8_FOUND)
  set(GZ_SIM_VER 8)
else()
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER 7)
endif()
message(STATUS "pad_motion_gazebo: building PadMotionPlugin against gz-sim${GZ_SIM_VER}")

# Plugin library, loaded by gz-sim's plugin loader (via a
# <plugin filename="PadMotionPlugin" name="pad_motion::PadMotionPlugin"/>
# element in whatever world/model SDF embeds it -- that SDF lives outside
# this package) -- NOT a ROS node executable, even though it links
# rclcpp/px4_msgs directly to publish ground truth from inside the .so with
# no separate bridge process.
add_library(PadMotionPlugin SHARED src/PadMotionPlugin.cc)
# Plain (non-keyword) signature: ament_target_dependencies() below also
# calls target_link_libraries() on this target in plain form, and CMake
# forbids mixing keyword (PUBLIC/PRIVATE/INTERFACE) and plain signatures on
# the same target.
target_link_libraries(PadMotionPlugin gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
ament_target_dependencies(PadMotionPlugin rclcpp px4_msgs)

install(
  TARGETS PadMotionPlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
