#ifndef _ros_apriltag_utils_h
#define _ros_apriltag_utils_h
#include <vector>
#include <Eigen/Eigen>
#include "odom_utils.h"
#include <rclcpp/rclcpp.hpp>
//#include <ros_traj_gen_utils/msg/april_tag_detection.hpp> // message
//#include <ros_traj_gen_utils/msg/april_tag_detection_array.hpp> // message
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traj_gen/traj_utils/quaternion.h>
#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features
const int BUFFER_SIZE = 60;

typedef struct {
    nav_msgs::msg::Odometry quad;
    geometry_msgs::msg::Pose target;
} joint_pose;

class apriltag_utils {


private:
//Creating the circulur buffer for odom
int circle_start = 0;
int circle_end = BUFFER_SIZE-1;
nav_msgs::msg::Odometry odom_buffer[BUFFER_SIZE];
rclcpp::TimerBase::SharedPtr timer;
odom_utils odom_l;
rclcpp::Node::SharedPtr node_;
nav_msgs::msg::Odometry current_heading;
//PREVIOUS POSE ARRAY
geometry_msgs::msg::PoseStamped current_target;
Eigen::Matrix4d H_RC;
Eigen::Matrix4d H_TAG;
//Timer Function to stuff the cricle buffer

public:
apriltag_utils();
int flag = 0;

void setNode(rclcpp::Node::SharedPtr node);
//Configure camera/tag extrinsics: camera offset in body frame, camera tilt about
//its x-axis (rad), and the tag->target translation offset.
void setExtrinsics(const Eigen::Vector3d& camTranslation, double camTilt, const Eigen::Vector3d& tagTranslation);
//Takes a stdTrajectory MSG and stores it in the private trajectory variable
void aprilListen(const geometry_msgs::msg::PoseStamped &msg);
void syncCallback(const geometry_msgs::msg::PoseArray &msg1,const nav_msgs::msg::Odometry &msg2);
bool getLanding(joint_pose * pointer_in); //returns false if failed to detect perch
bool getLanding(Eigen::Matrix4d * pointer_in); //returns false if failed to detect perch
bool getLanding(Eigen::Matrix4d * pointer_in, nav_msgs::msg::Odometry * msg2); //returns false if failed to detect perch
//Feed the latest odometry (from the main odom callback) into the buffer pipeline
void updateOdom(const nav_msgs::msg::Odometry &msg);
Eigen::Matrix4d WorldRot(joint_pose pose);
//Convert Homogenous transform to odometry for publishing
static nav_msgs::msg::Odometry convertMsg(Eigen::Matrix4d H, nav_msgs::msg::Odometry header);
void timerCallback();
};

#endif
