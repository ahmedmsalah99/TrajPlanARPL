#ifndef _ros_waypoint_utils_h
#define _ros_waypoint_utils_h
#include <vector>
#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <traj_gen/trajectory/Waypoint.h>
#include <ros_traj_gen_utils/ros_traj_utils.h>
#include <traj_gen/traj_utils/quaternion.h>
#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features

class ros_waypoint_utils {
private:
std::vector<waypoint> vertices;
std::string frame_id = "simulator";
rclcpp::Node::SharedPtr node_;
public:

int flag = 0;
void setNode(rclcpp::Node::SharedPtr node); // node used for reading optional waypoint parameters
void waypointListiner(const nav_msgs::msg::Path &msg); //Takes a stdTrajectory MSG and stores it in the private trajectory variable
std::vector<waypoint>* getTrajectory(); //returns null if nothing is there
std::string getFrameId();

};

#endif
