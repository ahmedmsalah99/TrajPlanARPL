#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
#include <memory>
#include <traj_gen/trajectory/Waypoint.h>
#include <traj_gen/trajectory/QPpolyTraj.h>
#include <traj_gen/traj_utils/polynomial.h>
#include <ros_traj_gen_utils/apriltag_utils.h>
#include <traj_gen/trajectory/TrajBase.h>
#include <ros_traj_gen_utils/ros_traj_utils.h>
#include <ros_traj_gen_utils/ros_waypoint_utils.h>
#include <ros_traj_gen_utils/poscmd_publisher.h>
#include <ros_traj_gen_utils/ros_cuboid_utils.h>
#include <ros_traj_gen_utils/ros_replanner_utils.h>
#include <trackers_msgs/srv/transition.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std;
using namespace std::chrono_literals;

rclcpp::Node::SharedPtr node;
ros_waypoint_utils listener;
//Publishers
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubQP;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visual_vel_pub_;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visual_acc_pub_;

rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subWaypoint;
rclcpp::Subscription<ros_traj_gen_utils::msg::CuboidMap>::SharedPtr subMap;
apriltag_utils aprilListen;

rclcpp::Client<trackers_msgs::srv::Transition>::SharedPtr srv_transition_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hover_;
rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subOdomMsg;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subApril;
// world -> odom: a fixed correction published once, from the FIRST
// /fmu/out/vehicle_odometry reading received (i.e. wherever the vehicle is
// when this node starts listening -- effectively home/launch). Without this,
// nothing relates the planner's "odom"-referenced trajectory to a "world"
// frame at all, which is what made RViz (referenced to "world"/ground) show
// the trajectory offset by exactly the vehicle's starting height/position.
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> worldOdomTfBroadcaster;
bool worldOdomTfPublished = false;

odom_utils odomListiner;
static const std::string line_tracker_min_jerk("std_trackers/LineTrackerMinJerkAction");
static const std::string null_tracker_str("std_trackers/NullTracker");
bool useRVIZ = false;
//Replanning hyperparameters
bool replan = false;
std::string vehicle_name;
std::string odom;
ros_cuboid_utils cube_map;
Eigen::Matrix4d target;
bool usePerch = false;
bool useVisual = false;
//Replanner timing/retry tuning (read from config in init_params)
double g_replan_time = 0.04;
double g_replan_t_off = 0.05;
double g_replan_retry_step = 0.2;
int g_replan_retry_max = 10;
double g_replan_min_seg = 0.5;
bool g_fov_enable = true;
double g_fov_coverage_fraction = 0.5;

// Helper to declare (once) and fetch a parameter with a default.
template <typename T>
static T getParamOr(const std::string& name, const T& def){
	if(!node->has_parameter(name)){
		node->declare_parameter<T>(name, def);
	}
	return node->get_parameter(name).get_value<T>();
}
nav_msgs::msg::Odometry vehicleOdometryToRosOdometry(
    const px4_msgs::msg::VehicleOdometry &px4_msg)
{
    nav_msgs::msg::Odometry odom;

    // Timestamp
    odom.header.stamp = rclcpp::Time(px4_msg.timestamp * 1000ULL);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // NED/FRD passthrough: the library now works natively in NED (z-down) / FRD,
    // so the PX4 odometry is copied as-is (no NED->ENU swap, no orientation rotation).
    odom.pose.pose.position.x = px4_msg.position[0];   // North
    odom.pose.pose.position.y = px4_msg.position[1];   // East
    odom.pose.pose.position.z = px4_msg.position[2];   // Down

    // PX4 quaternion is [w, x, y, z], body-FRD expressed in NED -- copied as-is.
    odom.pose.pose.orientation.w = px4_msg.q[0];
    odom.pose.pose.orientation.x = px4_msg.q[1];
    odom.pose.pose.orientation.y = px4_msg.q[2];
    odom.pose.pose.orientation.z = px4_msg.q[3];

    // Linear velocity (NED) as-is
    odom.twist.twist.linear.x = px4_msg.velocity[0];
    odom.twist.twist.linear.y = px4_msg.velocity[1];
    odom.twist.twist.linear.z = px4_msg.velocity[2];

    // Angular velocity (body FRD) as-is
    odom.twist.twist.angular.x = px4_msg.angular_velocity[0];
    odom.twist.twist.angular.y = px4_msg.angular_velocity[1];
    odom.twist.twist.angular.z = px4_msg.angular_velocity[2];

    return odom;
}

void init_params(){
	listener.setNode(node);
	aprilListen.setNode(node);
	worldOdomTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
	auto best_effort_qos = rclcpp::QoS(1).best_effort();
	//Camera/tag extrinsics (calibration); defaults reproduce the original values.
	std::vector<double> cam_t = getParamOr<std::vector<double>>("cam_translation", std::vector<double>{0.3, 0.0, 0.0});
	std::vector<double> tag_t = getParamOr<std::vector<double>>("tag_translation", std::vector<double>{0.0, 0.0, 0.0});
	Eigen::Vector3d camTrans = (cam_t.size() >= 3) ? Eigen::Vector3d(cam_t[0], cam_t[1], cam_t[2]) : Eigen::Vector3d(0.3, 0.0, 0.0);
	Eigen::Vector3d tagTrans = (tag_t.size() >= 3) ? Eigen::Vector3d(tag_t[0], tag_t[1], tag_t[2]) : Eigen::Vector3d(0.0, 0.0, 0.0);
	// Full camera-frame -> body-frame rotation, composed from:
	//  - body_r_cam: the fixed mount convention (row-major 3x3, e.g. a nadir-facing
	//    camera whose axes don't line up 1:1 with the body's -- default identity).
	//  - theta_from_nadir_deg: an additional tilt UP from nadir (deg), applied about
	//    the body y-axis. theta_from_nadir=90 (default) means the tilt term is the
	//    identity (no additional tilt beyond body_r_cam).
	// This replaces the old single-axis cam_tilt-based rotation (which could only
	// express a tilt about the camera's own x-axis with no base mount rotation at
	// all); cam_tilt is now used only by the FOV optical-axis model (setFovCamTilt).
	std::vector<double> body_r_cam_v = getParamOr<std::vector<double>>("body_r_cam",
		std::vector<double>{1,0,0, 0,1,0, 0,0,1});
	double theta_from_nadir_deg = getParamOr<double>("theta_from_nadir_deg", 90.0);
	Eigen::Matrix3d body_R_cam = Eigen::Matrix3d::Identity();
	if(body_r_cam_v.size() == 9){
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				body_R_cam(i,j) = body_r_cam_v[i*3+j];
			}
		}
	}
	double theta_from_hor = (90.0 - theta_from_nadir_deg) * (M_PI/180.0);
	Eigen::Matrix3d Rtilt_y;
	Rtilt_y << std::cos(theta_from_hor), 0.0, std::sin(theta_from_hor),
	           0.0,                      1.0, 0.0,
	           -std::sin(theta_from_hor),0.0, std::cos(theta_from_hor);
	Eigen::Matrix3d camToBodyRot = Rtilt_y * body_R_cam;
	aprilListen.setExtrinsics(camTrans, camToBodyRot, tagTrans);

	vehicle_name = getParamOr<std::string>("device", std::string(""));
	useVisual = getParamOr<bool>("visual", false);

	std::cout << " VEHICLE NAME " << vehicle_name <<std::endl;
	// setting up the publishers and subscribers
	pubQP = node->create_publisher<nav_msgs::msg::Path>("/"+vehicle_name+"/trackers_manager/qp_tracker/qp_trajectory_pos", 10);
	visual_vel_pub_ = node->create_publisher<nav_msgs::msg::Path>("/"+vehicle_name+"/trackers_manager/qp_tracker/qp_trajectory_vel", 10);
	visual_acc_pub_ = node->create_publisher<nav_msgs::msg::Path>("/"+vehicle_name+"/trackers_manager/qp_tracker/qp_trajectory_acc", 10);
	subApril = node->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/tags_features_extractor/tag_pose", best_effort_qos,
		[](const geometry_msgs::msg::PoseStamped &msg){ aprilListen.aprilListen(msg); });

	subWaypoint = node->create_subscription<nav_msgs::msg::Path>(
		vehicle_name+"/waypoints", 10,
		[](const nav_msgs::msg::Path &msg){ listener.waypointListiner(msg); });
	srv_transition_ = node->create_client<trackers_msgs::srv::Transition>(vehicle_name+"/trackers_manager/transition");
	hover_	= node->create_client<std_srvs::srv::Trigger>(vehicle_name+"/mav_services/hover");
	std::string odom_frame = getParamOr<std::string>("odom_frame", std::string("/odom"));
	target.setIdentity();
	//load a preselcted target
	std::vector<double> select_target = getParamOr<std::vector<double>>("target_pose", std::vector<double>{});
	if(select_target.size() == 16){
		int count =0;
		for (int i = 0;i<4;i++){
			for (int j = 0;j<4;j++){
				target(i,j) = select_target[count];
				count +=1;
			}
		}
		usePerch = true;
		std::cout << " WE ARE USING A TARGET " << target <<std::endl;
	}
	std::string odom_topic = "/fmu/out/vehicle_odometry";
	
	subOdomMsg = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
		odom_topic, best_effort_qos,
		[](const px4_msgs::msg::VehicleOdometry &msg){
			nav_msgs::msg::Odometry odom =
        	vehicleOdometryToRosOdometry(msg);

			if(!worldOdomTfPublished){
				// world's origin = wherever the vehicle is on this first
				// reading (translation only; no rotation correction).
				geometry_msgs::msg::TransformStamped t;
				t.header.stamp = node->now();
				t.header.frame_id = "world";
				t.child_frame_id = "odom";
				t.transform.translation.x = -odom.pose.pose.position.x;
				t.transform.translation.y = -odom.pose.pose.position.y;
				t.transform.translation.z = -odom.pose.pose.position.z;
				t.transform.rotation.w = 1.0;
				worldOdomTfBroadcaster->sendTransform(t);
				worldOdomTfPublished = true;
			}

			odomListiner.outputListiner(odom);
			aprilListen.updateOdom(odom);
			});
	subMap = node->create_subscription<ros_traj_gen_utils::msg::CuboidMap>(
		"/vox_blox_map/graph", 10,
		[](const ros_traj_gen_utils::msg::CuboidMap &msg){ cube_map.setListiner(msg); });
	//Replanner timing/retry tuning (defaults preserve prior behavior)
	g_replan_time = getParamOr<double>("replan_time", 0.04);
	g_replan_t_off = getParamOr<double>("replan_t_off", 0.05);
	g_replan_retry_step = getParamOr<double>("replan_retry_step", 0.2);
	g_replan_retry_max = getParamOr<int>("replan_retry_max", 10);
	g_replan_min_seg = getParamOr<double>("replan_min_seg", 0.5);
	g_fov_enable = getParamOr<bool>("fov_enable", true);
	g_fov_coverage_fraction = getParamOr<double>("fov_coverage_fraction", 0.5);
}


//Visualization
void visualize_paths(TrajBase * traj ){
	//encode and publish the msg path to see that way it should be following also velocity and acceleration
	nav_msgs::msg::Path msgQP = ros_traj_utils::encodePath(0, traj, listener.getFrameId()) ;
	pubQP->publish(msgQP);
	//Do 2D Visualization
	bool display2D = getParamOr<bool>("display_2D", false);
	if(display2D){
		msgQP = ros_traj_utils::encodePath(1,traj,"world");
		visual_vel_pub_->publish(msgQP);
		msgQP = ros_traj_utils::encodePath(2,traj,"world");
		visual_acc_pub_->publish(msgQP);
	}
}

void executeOneShotTraj(std::vector<waypoint>  vertices, poscmd_publisher * controller, TrajBase * traj){
	ros_replan_utils replanner(traj, &odomListiner, &vertices, false);
	replanner.setReplanParams(g_replan_retry_step, g_replan_retry_max, g_replan_min_seg);
	if(usePerch){
		std::cout << target <<std::endl;
		replanner.initialPlan(3, target);
	}
	else{
		replanner.initialPlan(4);
	}
	visualize_paths(traj);
	//Nulltracker transition
	auto transition_cmd = std::make_shared<trackers_msgs::srv::Transition::Request>();
	transition_cmd->tracker = null_tracker_str;
	srv_transition_->async_send_request(transition_cmd);
	//poscmd transition
	controller->startFlight( traj);
	while(controller->getState() != HOVER && rclcpp::ok()){
		rclcpp::spin_some(node);
	}
	controller->setEND();
	auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
	hover_->async_send_request(trigger);

}

void executeReplanTraj(std::vector<waypoint>  vertices, poscmd_publisher * controller, TrajBase * traj){
	//std::cout << "number of vertices" <<vertices->size() <<std::endl;
	std::cout << "preparation initial plan " <<std::endl;
	ros_replan_utils replanner(traj, &odomListiner, &vertices, useVisual);
	replanner.setReplanParams(g_replan_retry_step, g_replan_retry_max, g_replan_min_seg);
	replanner.setFOVEnable(g_fov_enable);
	replanner.setFOVCoverageFraction(g_fov_coverage_fraction);
	if(usePerch){
		replanner.initialPlan(3, target);
	}
	else{
		replanner.initialPlan(4);
	}
	std::cout << "preparation initial plan solved " <<std::endl;
	TrajBase * traj_use = replanner.getTraj();
	visualize_paths(traj_use);
	auto transition_cmd = std::make_shared<trackers_msgs::srv::Transition::Request>();
	transition_cmd->tracker = null_tracker_str;
	srv_transition_->async_send_request(transition_cmd);
	controller->startFlight(traj_use);
	double t0 = node->now().seconds() ;
	double replan_time = g_replan_time;
	double time_plan = 0;
	std::cout << "TIME Start Flight " << node->now().seconds() <<std::endl;
	while(controller->getState() != HOVER && rclcpp::ok()){
		rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(replan_time*0.1)));
		//pubTarget.publish(target);
		rclcpp::spin_some(node);
		bool replan_success = false;
		double tend =  node->now().seconds() ;
		double t_elap = tend - t0;
		t0 = tend;
		time_plan+=t_elap;
		//Publish apriltag Detection
		if (time_plan >=replan_time){
			//std::cout << "replan start" <<std::endl;
			double replan_timer = node->now().seconds() ;
			if(useVisual){
				Eigen::Matrix4d H;
				if(aprilListen.getLanding(&H)){
					// std::cout << "[DIAG] getLanding OK, target H=\n" << H << std::endl;
					replan_success = replanner.replan(4,time_plan,g_replan_t_off,H);
				}
				else{
					std::cout << "[DIAG] getLanding FAILED (tag not consumed; "
					          << "using stale/initial plan)" << std::endl;
				}
			}
			else{
				replan_success = replanner.replan(4, time_plan, g_replan_t_off);
			}
			//std::cout << "replan end" <<std::endl;
			if (replan_success){
				traj_use = replanner.getTraj();
				controller->startFlight(traj_use);
				//refresh RViz so it shows the live replanned trajectory, not the
				//stale initial plan (the endpoint tracks the moving target)
				visualize_paths(traj_use);
			}
			double replan_timer_end =  node->now().seconds() ;
			// std::cout << "Time ELAPSED " <<replan_timer_end-replan_timer <<std::endl;
			time_plan = 0.0;
		}
	}
	std::cout << "replanning time done, take a hover" << std::endl;
	controller->setEND();
	auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
	hover_->async_send_request(trigger);
}




int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	node = std::make_shared<rclcpp::Node>("traj_exe");
	init_params();
	std::vector<waypoint> * vertices;
	rclcpp::sleep_for(1s);
	QPpolyTraj qp_traj(4);
	//Dynamic limits used for time allocation (from config; defaults preserve prior behavior)
	qp_traj.limits[1] = getParamOr<double>("v_max", 5.0);
	qp_traj.limits[2] = getParamOr<double>("a_max", 10.0);
	//Perching parameters (from config; defaults reproduce the original hard-coded values)
	qp_traj.setPerchParams(
		getParamOr<double>("max_inclination_accel", 4.0),
		getParamOr<double>("impact_normal_vel", 1.0),
		getParamOr<double>("impact_slide_vel", -3.0),
		getParamOr<double>("min_pitch", 0.5));
	//eq.(14) approach band
	qp_traj.setPerchBand(
		getParamOr<double>("perch_band_q", 0.5),
		getParamOr<double>("perch_window", 0.5),
		getParamOr<double>("perch_band_eps", 0.2));
	//FOV optical-axis model's camera tilt (rad) -- NOT the same rotation as the
	//apriltag extrinsics above (that's now body_r_cam/theta_from_nadir_deg); this
	//is a separate, still-under-investigation knob specific to the FOV model.
	qp_traj.setFovCamTilt(getParamOr<double>("cam_tilt", 0.25));
	qp_traj.setFovMargin(getParamOr<double>("fov_margin", 0.0));
	//FOV cone ratio r/h = tan(horizontal_FOV / 2), eq.(7). Configure via the
	//camera's actual horizontal field of view (rad); default reproduces the
	//previous hard-coded r_h=0.76732 for cameras that don't set this.
	qp_traj.setFovRh(std::tan(getParamOr<double>("fov_horizontal_fov", 2.0*std::atan(0.76732)) / 2.0));
	//eq.(9) trust region: bounds how far the trajectory may sample from each
	//FOV row's linearization point, so the Taylor expansion stays valid.
	qp_traj.setFovTrustRegion(
		getParamOr<double>("fov_trust_pos", 0.05),
		getParamOr<double>("fov_trust_acc", 0.2),
		getParamOr<double>("fov_trust_yaw", 0.1));
	//Minimum altitude (metres above the world origin) enforced across the whole
	//trajectory (NED: an upper bound on z). Disabled by default.
	qp_traj.setMinAltitude(
		getParamOr<bool>("min_altitude_enable", false),
		getParamOr<double>("min_altitude", 0.3));
	//These values of 5 means that for a 1.7m distance gives around 5/3.4 or 1.5 ish time allocated.
	TrajBase * traj;
	double dt =0.01; //Handles the timer speed
	std::string cmd_topic = vehicle_name+"/position_cmd";
	poscmd_publisher controller(node, cmd_topic, dt);
	bool useBern = false;
	while(rclcpp::ok()) {
		rclcpp::spin_some(node);
		//wait till we publish waypoints with RVIZ
		if(listener.flag==1){
			vertices = listener.getTrajectory();
			listener.flag =0; //Allow another trajectory to be queud
			traj = &qp_traj;
			replan = getParamOr<bool>("replan", false);
			std::cout << " Start Execution" <<std::endl;
			if(replan){
				executeReplanTraj(*vertices, &controller, traj);
			}
			else{
				executeOneShotTraj(*vertices, &controller, traj);
			}
		}
	}
	rclcpp::shutdown();
	return 0;
}
