#include <ros_traj_gen_utils/ros_waypoint_utils.h>
#include <iostream>
#include <cmath>
using namespace std;

static const std::string low_title[4] = { "lowX", "lowY", "lowZ", "lowW" };
static const std::string up_title[4] = { "upX", "upY", "upZ", "upW" };

namespace {
constexpr double kPoseEqEps = 1e-6;
bool poseApproxEqual(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b){
	return std::fabs(a.position.x - b.position.x) < kPoseEqEps &&
	       std::fabs(a.position.y - b.position.y) < kPoseEqEps &&
	       std::fabs(a.position.z - b.position.z) < kPoseEqEps &&
	       std::fabs(a.orientation.x - b.orientation.x) < kPoseEqEps &&
	       std::fabs(a.orientation.y - b.orientation.y) < kPoseEqEps &&
	       std::fabs(a.orientation.z - b.orientation.z) < kPoseEqEps &&
	       std::fabs(a.orientation.w - b.orientation.w) < kPoseEqEps;
}
} // namespace

bool ros_waypoint_utils::isSameAsLast(const std::vector<geometry_msgs::msg::PoseStamped>& points) const {
	if(!haveLastPoints_){
		return false;
	}
	if(points.size() != lastPoints_.size()){
		return false;
	}
	for(size_t i = 0; i < points.size(); i++){
		if(!poseApproxEqual(points[i].pose, lastPoints_[i].pose)){
			return false;
		}
	}
	return true;
}

void ros_waypoint_utils::setNode(rclcpp::Node::SharedPtr node){
	node_ = node;
}

std::vector<waypoint>*  ros_waypoint_utils::getTrajectory(){
	if (flag){
		return &vertices;
	}
	return NULL;
}

// Helper that reads a double-array parameter from the node if it exists.
static bool getDoubleArrayParam(rclcpp::Node::SharedPtr node, const std::string& name,
                                std::vector<double>* out){
	if(!node){
		return false;
	}
	if(!node->has_parameter(name)){
		node->declare_parameter<std::vector<double>>(name, std::vector<double>{});
	}
	std::vector<double> val = node->get_parameter(name).as_double_array();
	if(val.empty()){
		return false;
	}
	*out = val;
	return true;
}

// NOTE (ROS 2 port): the original ROS 1 implementation parsed per-waypoint inequality
// constraints from nested XmlRpc parameter dictionaries (e.g. /trajectory_gen_demo/ineqN
// as a list of dicts). ROS 2 has no XmlRpc parameter type, and the bundled launch configs
// do not set these parameters, so the nested-inequality parsing is intentionally not
// ported. The simpler per-waypoint vel/acc/jerk/snap constraints below are ported using
// the ROS 2 parameter API.
waypoint additionalConstraint(rclcpp::Node::SharedPtr node, waypoint w, int pointNum){
	std::vector<double> constraint;
	std::string idx = std::to_string(pointNum);
	if (getDoubleArrayParam(node, "vel" + idx, &constraint) && constraint.size() >= 4){
		Eigen::Vector4d vel(constraint.data());
		w.setVel(vel);
	}
	if (getDoubleArrayParam(node, "acc" + idx, &constraint) && constraint.size() >= 4){
		Eigen::Vector4d acc(constraint.data());
		w.setAccel(acc);
	}
	if (getDoubleArrayParam(node, "jer" + idx, &constraint) && constraint.size() >= 4){
		Eigen::Vector4d jer(constraint.data());
		w.setJerk(jer);
	}
	if (getDoubleArrayParam(node, "sna" + idx, &constraint) && constraint.size() >= 4){
		Eigen::Vector4d sna(constraint.data());
		w.setSnap(sna);
	}
	return w;
}

void ros_waypoint_utils::waypointListiner(const nav_msgs::msg::Path &msg){
	std::vector<geometry_msgs::msg::PoseStamped> points= msg.poses;
	// [DEDUP] A republish of the exact same waypoint list (e.g. a heartbeat/
	// dummy trigger that resends unconditionally, regardless of flight
	// state) must not re-trigger a whole new flight -- only content that
	// actually changed should. Without this, executeOneShotTraj/
	// executeReplanTraj restart from scratch every time the previous flight
	// naturally finishes and the still-running heartbeat is seen again.
	if(isSameAsLast(points)){
		return;
	}
	lastPoints_ = points;
	haveLastPoints_ = true;

	frame_id = msg.header.frame_id;
	vertices.clear();
	double prevYaw = 0;
	for(size_t i =0; i <points.size();i++){
		Eigen::Vector4d tempPoint(4);
		geometry_msgs::msg::Pose pose = points[i].pose;
		tempPoint(0) = pose.position.x;
		tempPoint(1) = pose.position.y;
		tempPoint(2) = pose.position.z;
		Quaternion q;
		q.x = pose.orientation.x;
		q.y = pose.orientation.y;
		q.z = pose.orientation.z;
		q.w = pose.orientation.w;
		EulerAngles ang = ToEulerAngles(q);
		//Reversing the angles
		while((ang.yaw - prevYaw) > 3.1415926){
			ang.yaw = ang.yaw - 2*3.1415926;
		}
		while((ang.yaw - prevYaw) < -3.1415926) {
			ang.yaw = ang.yaw + 2*3.1415926;
		}
		tempPoint(3) = ang.yaw;
		prevYaw = ang.yaw;
		waypoint w(tempPoint);
		w = additionalConstraint(node_, w, i);
		vertices.push_back(w);
	}
	flag = 1;
}

std::string ros_waypoint_utils::getFrameId(){
	return frame_id;
}
