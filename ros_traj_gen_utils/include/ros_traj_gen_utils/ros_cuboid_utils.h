#ifndef _ros_cuboid_utils_h
#define _ros_cuboid_utils_h
#include <vector>
#include <Eigen/Eigen>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <traj_gen/traj_utils/quaternion.h>
#include <traj_gen/trajectory/Waypoint.h>

#include <ros_traj_gen_utils/msg/cuboid_map.hpp> // message
#include <visualization_msgs/msg/marker.hpp> // message
#include <visualization_msgs/msg/marker_array.hpp> // message
#include <ros_traj_gen_utils/rect_node.h>




class ros_cuboid_utils {
private:
bool exist_reading =false;
ros_traj_gen_utils::msg::CuboidMap graph;
std::vector<rect_node> heap;
//Helper function ensure that there exists a path between objects
visualization_msgs::msg::MarkerArray marker_array_;
Eigen::MatrixXf solvedPath;
bool first_msg;
public:
ros_cuboid_utils();
void setListiner(const ros_traj_gen_utils::msg::CuboidMap &msg);
bool existReading() const; //checks if there is a map
ros_traj_gen_utils::msg::CuboidMap getGraph() const; //returns the cuboid map msg
//Returns a 6nx2 matrix. Matrix always comes in multiples of 2 the first and last
//Shrink is an optional parameter which shrinks the cubes a set a mount after being based
//A slight modification is made where the cubes are forced to overlap therefore if two cubes merely touch
//An additional cube will be inserted ensuring overlap to ensure safe shrinking.
std::vector<rect_node> astar(Eigen::Vector3f start_pt, Eigen::Vector3f end_pt);
std::vector<rect_node> astar(Eigen::Vector3f start_pt, Eigen::Vector3f end_pt, float shrink);

//Shrinks the boxes a set amount to give additional clearance
std::vector<rect_node> genOverlap(std::vector<rect_node> path);
std::vector<rect_node> shrink(std::vector<rect_node> path, float shrink);
visualization_msgs::msg::MarkerArray visulizeSafeFlight(std::vector<rect_node> path);
};

#endif
