#ifndef _ros_odom_utils_h
#define _ros_odom_utils_h
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features

class odom_utils {
private:
bool read = false;
nav_msgs::msg::Odometry current_heading;

public:
void outputListiner(const nav_msgs::msg::Odometry &msg);
bool enable_write = true;
bool enable_time_samp = false;
double now;
bool getCurrOdom(nav_msgs::msg::Odometry * curr_heading);
};

#endif
