cmake_minimum_required(VERSION 3.8)
project(traj_gen)

# Compile as C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# ament / ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)

# Third-party / system dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Threads REQUIRED)
find_package(ooqp REQUIRED ooqpbase ooqpdense ooqpgensparse ooqpmehrotra ooqpbound
  ooqpgendense ooqpgondzio ooqpsparse)
# nlopt is installed system-wide (/usr/local) by install_trajgen_depend.sh and is
# linked directly below; no find_package module is required.

add_library(traj_gen
  src/traj_src/Waypoint.cpp
  src/traj_src/TrajBase.cpp
  src/traj_src/QPpolyTraj.cpp
  src/traj_src/FOV.cpp
  src/ros_utils_src/polynomial.cc
  src/ros_utils_src/EigenQP.cpp
  src/ros_utils_src/quaternion.cpp
  src/ooqp_interface/OoqpEigenInterface.cpp
  src/ooqp_interface/QuadraticProblemFormulation.cpp
)

target_include_directories(traj_gen PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${NLOPT_INCLUDE_DIRS}
  ${OOQP_INCLUDE_DIRS}
)

ament_target_dependencies(traj_gen nav_msgs)

# OOQP, ma27 (HSL Fortran) and blas are static archives with circular
# interdependencies, and OOQP_LIBRARY embeds libma27.a after gfortran, so wrap
# them in a linker group to make resolution order-independent. boost::thread is
# used in QPpolyTraj.cpp and must be linked explicitly.
target_link_libraries(traj_gen
  nlopt
  ${NLOPT_LIBRARIES} ${NLOPT_LIBRARY}
  -Wl,--start-group
  ${OOQP_LIBRARIES} ${OOQP_LIBRARY}
  blas ma27 gfortran
  -Wl,--end-group
  Boost::thread
  Threads::Threads
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS traj_gen
  EXPORT export_traj_gen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export so downstream ament packages (ros_traj_gen_utils) can link traj_gen
ament_export_include_directories(include)
ament_export_libraries(traj_gen)
ament_export_targets(export_traj_gen HAS_LIBRARY_TARGET)
ament_export_dependencies(nav_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
