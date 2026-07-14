#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
#include <memory>
#include <atomic>
#include <fstream>
#include <iomanip>
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
#include <tf2_ros/transform_broadcaster.h>
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
// world -> odom: republished every /fmu/out/vehicle_odometry callback, since
// the vehicle (and therefore this offset) keeps moving. Without this, nothing
// relates the planner's "odom"-referenced trajectory to a "world" frame at
// all, which is what made RViz (referenced to "world"/ground) show the
// trajectory offset by the vehicle's position.
std::shared_ptr<tf2_ros::TransformBroadcaster> worldOdomTfBroadcaster;

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
// Gates executeReplanTraj's replan() loop: the initial plan is always solved
// and continuously published (via poscmd_publisher's own timer) regardless
// of this flag, but replan() is only called once this is true. Off by
// default -- start_replan/stop_replan (Trigger services) are the only way to
// flip it, so replanning begins only once whatever's driving the vehicle
// (e.g. offboard_bridge, after confirming PX4 actually entered OFFBOARD) has
// deliberately asked for it, not the instant the initial plan finishes.
std::atomic<bool> g_replanEnabled{false};
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_replan_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_replan_;

// --- DEBUG (this branch only): plan-vs-actual comparison ---
// Goal: capture what the trajectory ASSUMES will happen over time (a static
// CSV dump of the frozen plan) alongside what ACTUALLY happens (a live CSV
// of real vehicle state), so the two can be plotted against each other.
// Replanning is deliberately left disabled (see start_replan below) -- this
// is meant to isolate "does the vehicle track a single static plan" from
// any replanning-related effects.
//
// Freeze: while replanning is off, the existing visual-target refresh loop
// (see executeReplanTraj's disabled branch) keeps re-solving the initial
// plan toward wherever the visual target currently is. Once that target
// stops moving (holds steady within kFreezeStableTolM for
// kFreezeStableTicks consecutive checks), stop re-solving and dump the
// resulting plan's whole time profile to CSV.
bool g_planFrozen = false;
bool g_haveLastVisualTarget = false;
Eigen::Matrix4d g_lastVisualTarget = Eigen::Matrix4d::Identity();
int g_visualStableCount = 0;
const int kFreezeStableTicks = 10;      // consecutive stable cadence ticks before freezing
const double kFreezeStableTolM = 0.02;  // metres; position-only "unchanged" tolerance

// Start/stop hooked onto the EXISTING start_replan/stop_replan services:
// start_replan already fires exactly "when offboard is asked for" (that's
// what offboard_bridge calls once it confirms OFFBOARD via VehicleStatus),
// even with g_replanEnabled's assignment commented out -- the service call
// itself still happens, so it's a convenient, already-wired trigger for
// "start recording the real vehicle state now."
bool g_recordingActual = false;
double g_recordingStartTime = 0.0;
std::ofstream g_actualCsv;

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
	worldOdomTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
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
	srv_start_replan_ = node->create_service<std_srvs::srv::Trigger>(
		vehicle_name+"/start_replan",
		[](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
		   std::shared_ptr<std_srvs::srv::Trigger::Response> response){
			// g_replanEnabled = true;  // replanning deliberately disabled for this debug capture
			// DEBUG: this still fires exactly "when offboard is asked for" (offboard_bridge
			// calls start_replan right after confirming OFFBOARD via VehicleStatus), so it's
			// reused here as the trigger to start recording real vehicle state.
			if(!g_recordingActual){
				std::string path = getParamOr<std::string>("actual_csv_path", std::string("/tmp/actual_trajectory.csv"));
				g_actualCsv.open(path, std::ios::out | std::ios::trunc);
				if(g_actualCsv.is_open()){
					g_recordingStartTime = node->now().seconds();
					// t0_abs lets plot_plan_vs_actual.py compute the exact offset between
					// this recording's start and the planned CSV's freeze instant, instead
					// of requiring a manually-guessed --shift.
					g_actualCsv << "# t0_abs," << std::setprecision(17) << g_recordingStartTime << "\n";
					g_actualCsv << "t,x,y,z,vx,vy,vz\n";
					g_recordingActual = true;
					std::cout << "[PLAN_VS_ACTUAL] start_replan called -- recording actual "
					          << "vehicle state to " << path << std::endl;
				}
				else{
					std::cout << "[PLAN_VS_ACTUAL] FAILED to open " << path << " for writing" << std::endl;
				}
			}
			response->success = true;
			response->message = "Replanning enabled.";
			std::cout << "[REPLAN_GATE] " << response->message << std::endl;
		});
	srv_stop_replan_ = node->create_service<std_srvs::srv::Trigger>(
		vehicle_name+"/stop_replan",
		[](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
		   std::shared_ptr<std_srvs::srv::Trigger::Response> response){
			g_replanEnabled = false;
			if(g_recordingActual){
				g_actualCsv.close();
				g_recordingActual = false;
				std::cout << "[PLAN_VS_ACTUAL] stop_replan called -- stopped recording "
				          << "actual vehicle state." << std::endl;
			}
			response->success = true;
			response->message = "Replanning disabled -- the last successfully "
			                     "replanned trajectory keeps being published as-is.";
			std::cout << "[REPLAN_GATE] " << response->message << std::endl;
		});
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

			{
				// Republished every callback -- this tracks the live vehicle
				// position, not a one-time offset. (Translation only, no
				// rotation correction; sign/convention to be confirmed against
				// the rest of your TF tree.)
				geometry_msgs::msg::TransformStamped t;
				t.header.stamp = node->now();
				t.header.frame_id = "world";
				t.child_frame_id = "odom";
				t.transform.translation.x = odom.pose.pose.position.x;
				t.transform.translation.y = odom.pose.pose.position.y;
				t.transform.translation.z = odom.pose.pose.position.z;
				t.transform.rotation.w = 1.0;
				worldOdomTfBroadcaster->sendTransform(t);
			}

			odomListiner.outputListiner(odom, node);
			aprilListen.updateOdom(odom);

			// DEBUG: append real vehicle state while a recording session is active
			// (see start_replan/stop_replan above), timestamped relative to when
			// recording started so it lines up with the frozen plan's own t=0.
			if(g_recordingActual && g_actualCsv.is_open()){
				double t = node->now().seconds() - g_recordingStartTime;
				g_actualCsv << t << "," << odom.pose.pose.position.x << ","
				            << odom.pose.pose.position.y << "," << odom.pose.pose.position.z << ","
				            << odom.twist.twist.linear.x << "," << odom.twist.twist.linear.y << ","
				            << odom.twist.twist.linear.z << "\n";
				g_actualCsv.flush();
			}
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

// DEBUG (this branch only): dump the frozen plan's whole time profile
// (position + velocity, sampled at kPlanCsvDt from t=0 to the plan's total
// duration) to CSV, so it can be plotted against the recorded actual
// vehicle state.
void dumpPlannedTrajectoryCsv(TrajBase * traj_use, double freezeAbsTime){
	std::string path = getParamOr<std::string>("planned_csv_path", std::string("/tmp/planned_trajectory.csv"));
	std::ofstream f(path, std::ios::out | std::ios::trunc);
	if(!f.is_open()){
		std::cout << "[PLAN_VS_ACTUAL] FAILED to open " << path << " for writing" << std::endl;
		return;
	}
	// t0_abs lets plot_plan_vs_actual.py compute the exact offset between this
	// freeze instant and the actual recording's start, instead of requiring a
	// manually-guessed --shift.
	f << "# t0_abs," << std::setprecision(17) << freezeAbsTime << "\n";
	f << "t,x,y,z,vx,vy,vz\n";
	double totalTime = 0.0;
	for(size_t i = 0; i < traj_use->segmentTimes.size(); i++){ totalTime += traj_use->segmentTimes[i]; }
	const double kPlanCsvDt = 0.02;
	for(double t = 0.0; t <= totalTime + 1e-9; t += kPlanCsvDt){
		Eigen::MatrixXd pt = traj_use->evalTraj(t);
		f << t << "," << pt(0,0) << "," << pt(0,1) << "," << pt(0,2) << ","
		  << pt(1,0) << "," << pt(1,1) << "," << pt(1,2) << "\n";
	}
	f.close();
	std::cout << "[PLAN_VS_ACTUAL] wrote planned trajectory (" << totalTime
	          << "s, dt=" << kPlanCsvDt << ") to " << path << std::endl;
}

void executeOneShotTraj(std::vector<waypoint>  vertices, poscmd_publisher * controller, TrajBase * traj){
	ros_replan_utils replanner(traj, &odomListiner, &vertices, false);
	replanner.setReplanParams(g_replan_retry_step, g_replan_retry_max, g_replan_min_seg);
	bool initial_ok;
	if(usePerch){
		std::cout << target <<std::endl;
		initial_ok = replanner.initialPlan(3, target);
	}
	else{
		initial_ok = replanner.initialPlan(4);
	}
	if(!initial_ok){
		std::cout << "[INITIAL_PLAN] FAILED -- not publishing/commanding this trajectory." << std::endl;
		controller->setEND();
		auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
		hover_->async_send_request(trigger);
		return;
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
	bool initial_ok;
	if(usePerch){
		initial_ok = replanner.initialPlan(3, target);
	}
	else{
		initial_ok = replanner.initialPlan(4);
	}
	if(!initial_ok){
		std::cout << "[INITIAL_PLAN] FAILED -- not publishing/commanding this trajectory." << std::endl;
		controller->setEND();
		auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
		hover_->async_send_request(trigger);
		return;
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
	// Gates replan()'s call cadence ONLY. Pinned at 0 while replanning isn't
	// enabled (see below) and reset to 0 the instant it becomes enabled, so
	// replan()'s incremental "continue from the predicted future point" model
	// (which needs real elapsed FLIGHT time) never sees time accumulated while
	// the vehicle wasn't actually being driven by the plan yet -- this is the
	// entire fix from the earlier "commanded acceleration far from the
	// vehicle's real acceleration" bug and must stay untouched by anything else.
	double time_plan = 0;
	// Fully independent of time_plan: gates how often the initial plan gets
	// re-solved toward the current visual target while replanning isn't
	// enabled yet. Only ever read/written in the `else` branch below; replan()
	// never sees this variable at all.
	double visual_refresh_time = 0;
	bool replanWasEnabled = false;
	std::cout << "TIME Start Flight " << node->now().seconds() <<std::endl;
	while(controller->getState() != HOVER && rclcpp::ok()){
		rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(replan_time*0.1)));
		//pubTarget.publish(target);
		rclcpp::spin_some(node);

		bool replan_success = false;
		bool replanEnabledNow = g_replanEnabled.load();
		double tend =  node->now().seconds() ;
		if(replanEnabledNow && !replanWasEnabled){
			std::cout << "[REPLAN_GATE] replanning just enabled -- resetting elapsed-time "
			          << "bookkeeping so the idle/waiting period isn't counted" << std::endl;
			t0 = tend;
			time_plan = 0.0;
		}
		replanWasEnabled = replanEnabledNow;
		double t_elap = tend - t0;
		t0 = tend;

		if(replanEnabledNow){
			time_plan+=t_elap;
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
		else{
			// Pinned at 0 while disabled -- see the member comment above.
			time_plan = 0.0;
			if(useVisual && !g_planFrozen){
				// Not flying for real yet (still waiting on start_replan, e.g. for
				// offboard to be confirmed): keep the INITIAL plan aimed at wherever
				// the visual target currently is by re-solving it fresh (from the
				// vehicle's current position, not a predicted one) at this same
				// cadence -- so a moving/updating target keeps being tracked
				// continuously, not just on the first-ever sighting. Uses its own
				// independent timer (visual_refresh_time), never time_plan.
				visual_refresh_time += t_elap;
				if(visual_refresh_time >= replan_time){
					Eigen::Matrix4d H;
					if(aprilListen.getLanding(&H)){
						// DEBUG: has the visual target held steady (within
						// kFreezeStableTolM) since the last check? Once it has for
						// kFreezeStableTicks consecutive checks, freeze -- stop
						// re-solving and dump the current (now-stable) plan to CSV --
						// instead of re-solving toward this same, unchanged target
						// again.
						if(g_haveLastVisualTarget){
							double posDelta = (H.block<3,1>(0,3) - g_lastVisualTarget.block<3,1>(0,3)).norm();
							if(posDelta < kFreezeStableTolM){
								g_visualStableCount++;
							}
							else{
								g_visualStableCount = 0;
							}
						}
						g_lastVisualTarget = H;
						g_haveLastVisualTarget = true;

						if(g_visualStableCount >= kFreezeStableTicks){
							std::cout << "[PLAN_VS_ACTUAL] visual target held steady for "
							          << kFreezeStableTicks << " checks -- freezing the plan."
							          << std::endl;
							g_planFrozen = true;
							dumpPlannedTrajectoryCsv(traj_use, node->now().seconds());
						}
						else{
							bool redo_ok = replanner.initialPlan(3, H);
							if(redo_ok){
								traj_use = replanner.getTraj();
								controller->startFlight(traj_use);
								visualize_paths(traj_use);
							}
							else{
								std::cout << "[VISUAL_TARGET] initial plan toward the current "
								          << "target FAILED -- keeping the previous plan running."
								          << std::endl;
							}
						}
					}
					visual_refresh_time = 0.0;
				}
			}
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
	//Horizontal (x,y) vel/accel/jerk limits, sampled across every segment's
	//interior (not just at waypoints). Each is independently optional: <= 0
	//(the default) disables that derivative order's limit.
	qp_traj.setHorizontalLimits(
		getParamOr<double>("horiz_vel_limit", 0.0),
		getParamOr<double>("horiz_accel_limit", 0.0),
		getParamOr<double>("horiz_jerk_limit", 0.0));
	//Sampling step (s) for every per-vertex sampled inequality (perch eq.(14)
	//band, min-altitude, horizontal limits). Default coarsened from the
	//library's old hardcoded 0.01 -- see TrajBase's ineqSampleDt member
	//comment for why (row-count blowup from stacking multiple sampled boxes
	//made OOQP/MA27 fail to factor or fail to converge).
	qp_traj.setIneqSampleDt(getParamOr<double>("ineq_sample_dt", 0.05));
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
