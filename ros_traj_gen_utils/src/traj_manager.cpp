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
#include <traj_gen/traj_utils/quaternion.h>
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
#include <px4_msgs/msg/vehicle_status.hpp>
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
rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subVehicleStatus;
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
// What replan() anchors each new plan's start state to. True (default) keeps
// the existing behaviour of re-pinning it to the measured odometry every
// cycle; false continues from the previous plan's predicted state instead.
// See ros_replan_utils::setAnchorOdom() for why that distinction matters.
bool g_replan_anchor_odom = true;
// Fraction of the measured tracking error each replan folds into the new
// plan's start state. 1.0 reproduces the original full re-pin onto odometry,
// 0.0 is pure predictive continuation, in between keeps the setpoint's lead
// while bounding drift. See ros_replan_utils::setOdomBlend().
double g_replan_odom_blend = 1.0;
// Leave jerk and snap free at each plan's start waypoint so acceleration can
// ramp immediately instead of as O(t^3). See setFreeStartJerkSnap().
bool g_free_start_jerk_snap = false;
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

// [DEBUG] Optional mode: once true, the visual target replan() is aimed at
// gets captured once and then locked -- never refreshed from aprilListen
// again for as long as replanning stays enabled. Isolates "is the CONTROL
// loop the problem" from "is the visual target itself jumping around"
// during tracking-divergence debugging. Off by default; only takes effect
// while replanEnabledNow (i.e. after start_replan/offboard-enable) --
// visual tracking before that (the initial-plan re-solve loop) is
// untouched. g_haveFrozenTarget resets on stop_replan, so the next
// offboard-enable captures a fresh target rather than reusing a stale one.
bool g_freezeTargetOnOffboard = false;

// Is PX4 actually in OFFBOARD right now? Read straight from the flight
// controller's own VehicleStatus, NOT inferred from g_replanEnabled: that flag
// means "replanning is on", and is only related to offboard at all because
// offboard_bridge happens to call start_replan once it has confirmed the mode
// switch. Tying the flight start to it would mean a replan:false run never
// takes off, and would silently depend on which bridge is driving the vehicle.
std::atomic<bool> g_offboardActive{false};

// Hold the solved plan without starting its clock until offboard is actually
// enabled. poscmd_publisher's trajectory clock starts at startFlight(), so
// starting it at solve time means the plan is consumed while the vehicle is
// still under RC/position hold -- and a short plan runs out entirely before
// the operator flips to offboard. What is then being published is the
// trajectory's TERMINAL setpoint, held, so offboard engages against a large
// position step rather than a trajectory, and the vehicle step-responds to it
// (overshooting, and moving faster than the plan ever asked for). Only the
// visual replan path escaped this, because its idle re-solve loop kept
// calling startFlight() and so kept resetting the clock. Set false for
// sim/bench runs with no flight controller publishing VehicleStatus,
// otherwise nothing would ever fly.
bool g_waitForOffboard = true;
bool g_haveFrozenTarget = false;
Eigen::Matrix4d g_frozenTarget = Eigen::Matrix4d::Identity();

// [TRAJ_LOG] Planned-trajectory snapshots: every time a new solve becomes the
// active trajectory, sample it across its own duration and append it as one
// block to this file (throttled by g_trajSavePeriodS -- a fast replan cadence
// would otherwise flood the file with near-identical snapshots). Both this
// and g_actualCsv timestamp relative to g_loggingT0 -- t=0 the moment offboard
// is enabled (start_replan is called) -- so a planned sample's relative time
// is just gen_time_rel + t_local, no separate alignment step needed.
std::ofstream g_plannedCsv;
// The plan currently being flown, recorded even while logging is still off.
// With replanning off there is exactly one plan and it is solved as soon as
// waypoints arrive -- always before start_replan opens the logs -- so without
// keeping it here, maybeLogPlannedTrajectory's gate drops the only plan there
// will ever be and the planned log ends up containing nothing but its header.
TrajBase * g_activeTraj = nullptr;
double g_lastPlannedSaveTime = -1e18;
int g_plannedTrajId = 0;
double g_trajSavePeriodS = 1.0;
double g_trajSampleDt = 0.05;
std::string g_plannedTrajLogPath;
std::string g_actualTrajLogPath;

// [TRAJ_LOG] Continuous actual vehicle position/velocity, timestamped --
// written from the existing vehicle_odometry callback in init_params().
std::ofstream g_actualCsv;

// [TRAJ_LOG] Both logs only actually write once offboard is enabled --
// start_replan's handler opens (truncating) both files and sets this t=0.
// Off/0 until then; maybeLogPlannedTrajectory and the actual-state write
// below both check this before writing anything.
bool g_loggingEnabled = false;
double g_loggingT0 = 0.0;

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

void maybeLogPlannedTrajectory(TrajBase * traj_use);

// [TRAJ_LOG] Opens (truncating) both log files, writes their headers, and
// sets g_loggingT0 = now -- called from start_replan's handler, so t=0 is
// the moment offboard is actually enabled. Re-arms (fresh files, fresh t=0)
// every time start_replan fires, treating each offboard-enable as a new
// logging session.
void startTrajLogging(){
	g_loggingT0 = node->now().seconds();
	g_loggingEnabled = true;

	g_plannedCsv.open(g_plannedTrajLogPath, std::ios::out | std::ios::trunc);
	if(g_plannedCsv.is_open()){
		g_plannedCsv << "traj_id,gen_time_rel,t_local,x,y,z,vx,vy,vz\n";
	} else {
		std::cout << "[TRAJ_LOG] FAILED to open " << g_plannedTrajLogPath << " for writing" << std::endl;
	}
	g_lastPlannedSaveTime = -1e18;
	g_plannedTrajId = 0;

	g_actualCsv.open(g_actualTrajLogPath, std::ios::out | std::ios::trunc);
	if(g_actualCsv.is_open()){
		g_actualCsv << "t_rel,x,y,z,vx,vy,vz,roll,pitch,yaw\n";
	} else {
		std::cout << "[TRAJ_LOG] FAILED to open " << g_actualTrajLogPath << " for writing" << std::endl;
	}

	// Write out the plan that is already active. It was solved before this
	// point (the initial plan runs as soon as waypoints arrive, offboard is
	// enabled later), so it never made it past the logging gate. With
	// replanning on, later replans would eventually populate the file anyway;
	// with replanning off this is the only plan there is, which is why the
	// planned log came out empty.
	if(g_activeTraj != nullptr){
		maybeLogPlannedTrajectory(g_activeTraj);
	}
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
		[](const geometry_msgs::msg::PoseStamped &msg){ 
			std::cout << "updating vision" << std::endl;
			aprilListen.aprilListen(msg); });

	subWaypoint = node->create_subscription<nav_msgs::msg::Path>(
		vehicle_name+"/waypoints", 10,
		[](const nav_msgs::msg::Path &msg){ listener.waypointListiner(msg); });
	srv_transition_ = node->create_client<trackers_msgs::srv::Transition>(vehicle_name+"/trackers_manager/transition");
	hover_	= node->create_client<std_srvs::srv::Trigger>(vehicle_name+"/mav_services/hover");
	srv_start_replan_ = node->create_service<std_srvs::srv::Trigger>(
		vehicle_name+"/start_replan",
		[](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
		   std::shared_ptr<std_srvs::srv::Trigger::Response> response){
			g_replanEnabled = true;
			startTrajLogging();
			response->success = true;
			response->message = "Replanning enabled.";
			std::cout << "[REPLAN_GATE] " << response->message << std::endl;
		});
	srv_stop_replan_ = node->create_service<std_srvs::srv::Trigger>(
		vehicle_name+"/stop_replan",
		[](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
		   std::shared_ptr<std_srvs::srv::Trigger::Response> response){
			g_replanEnabled = false;
			g_haveFrozenTarget = false; // next offboard-enable captures a fresh target
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
	// Authoritative offboard state, straight from PX4 -- see g_offboardActive.
	subVehicleStatus = node->create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status_v1", best_effort_qos,
		[](const px4_msgs::msg::VehicleStatus &msg){
			g_offboardActive.store(
				msg.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
		});

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

			// [TRAJ_LOG] continuous actual position/velocity/attitude, relative
			// to g_loggingT0 (the moment offboard was enabled) -- no-op until
			// then. Roll/pitch logged to check whether a position/velocity
			// tracking deviation is actually attitude-tilt coupling (e.g. a
			// Z-axis correction leaning the thrust vector and dragging the
			// vehicle horizontally) rather than a separate horizontal-loop or
			// replanning issue.
			if(g_loggingEnabled && g_actualCsv.is_open()){
				Quaternion q_actual;
				q_actual.w = odom.pose.pose.orientation.w;
				q_actual.x = odom.pose.pose.orientation.x;
				q_actual.y = odom.pose.pose.orientation.y;
				q_actual.z = odom.pose.pose.orientation.z;
				EulerAngles rpy_actual = ToEulerAngles(q_actual);
				g_actualCsv << std::setprecision(17) << (node->now().seconds() - g_loggingT0) << ","
				            << std::setprecision(9)
				            << odom.pose.pose.position.x << "," << odom.pose.pose.position.y << ","
				            << odom.pose.pose.position.z << ","
				            << odom.twist.twist.linear.x << "," << odom.twist.twist.linear.y << ","
				            << odom.twist.twist.linear.z << ","
				            << rpy_actual.roll << "," << rpy_actual.pitch << "," << rpy_actual.yaw << "\n";
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
	// See the g_replan_anchor_odom declaration comment above.
	g_replan_anchor_odom = getParamOr<bool>("replan_anchor_odom", true);
	// See the g_replan_odom_blend declaration comment above.
	g_replan_odom_blend = getParamOr<double>("replan_odom_blend", 1.0);
	// See the g_free_start_jerk_snap declaration comment above.
	g_free_start_jerk_snap = getParamOr<bool>("free_start_jerk_snap", false);
	g_fov_enable = getParamOr<bool>("fov_enable", true);
	g_fov_coverage_fraction = getParamOr<double>("fov_coverage_fraction", 0.5);

	// [DEBUG] See g_freezeTargetOnOffboard declaration comment above.
	g_freezeTargetOnOffboard = getParamOr<bool>("freeze_target_on_offboard", false);
	// See the g_waitForOffboard declaration comment above.
	g_waitForOffboard = getParamOr<bool>("wait_for_offboard", true);

	// [TRAJ_LOG] Config only -- the files themselves are opened by
	// startTrajLogging(), called from start_replan's handler once offboard is
	// actually enabled (see g_loggingEnabled above).
	g_trajSavePeriodS = getParamOr<double>("traj_save_period_s", 1.0);
	g_trajSampleDt = getParamOr<double>("traj_sample_dt", 0.05);
	g_plannedTrajLogPath = getParamOr<std::string>(
		"planned_traj_log_path", std::string("/tmp/planned_trajectories.csv"));
	g_actualTrajLogPath = getParamOr<std::string>(
		"actual_traj_log_path", std::string("/tmp/actual_trajectory.csv"));
}

// [TRAJ_LOG] Samples traj_use across its own duration and appends it to
// g_plannedCsv as one traj_id-tagged block, throttled by g_trajSavePeriodS.
// No-op until offboard is enabled (g_loggingEnabled) -- checked before the
// throttle timer too, so the first trajectory generated right after offboard
// enables logs immediately instead of being blocked by a stale timestamp
// from while logging was off.
void maybeLogPlannedTrajectory(TrajBase * traj_use){
	// Recorded before the gate, so a plan solved while logging is still off is
	// still available for startTrajLogging() to write out.
	g_activeTraj = traj_use;
	if(!g_loggingEnabled || !g_plannedCsv.is_open()){
		return;
	}
	double now_s = node->now().seconds();
	if(now_s - g_lastPlannedSaveTime < g_trajSavePeriodS){
		return;
	}
	g_lastPlannedSaveTime = now_s;

	double gen_time_rel = now_s - g_loggingT0;
	double totalTime = 0.0;
	for(size_t i = 0; i < traj_use->segmentTimes.size(); i++){ totalTime += traj_use->segmentTimes[i]; }
	int traj_id = g_plannedTrajId++;
	for(double t = 0.0; t <= totalTime + 1e-9; t += g_trajSampleDt){
		Eigen::MatrixXd pt = traj_use->evalTraj(t);
		g_plannedCsv << traj_id << "," << std::setprecision(17) << gen_time_rel << "," << t << ","
		             << std::setprecision(9)
		             << pt(0,0) << "," << pt(0,1) << "," << pt(0,2) << ","
		             << pt(1,0) << "," << pt(1,1) << "," << pt(1,2) << "\n";
	}
	g_plannedCsv.flush();
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

// When useVisual is true, the initial plan must not run on a stale/default
// target -- it needs to be aimed at wherever the visual target actually is,
// same as every later re-solve. Blocks (spinning) until aprilListen has a
// detection to hand back, or the node starts shutting down. Returns false
// only in the shutdown case (H is left untouched then).
bool waitForVisualTarget(Eigen::Matrix4d * H){
	if(aprilListen.getLanding(H)){
		return true; // already have one -- no need to wait or log
	}
	std::cout << "[VISUAL_TARGET] useVisual is true -- waiting for a visual "
	          << "target before the initial plan can run..." << std::endl;
	while(rclcpp::ok()){
		rclcpp::spin_some(node);
		if(aprilListen.getLanding(H)){
			std::cout << "[VISUAL_TARGET] visual target received -- "
			          << "proceeding with initial plan." << std::endl;
			return true;
		}
		rclcpp::sleep_for(50ms);
	}
	return false;
}

// Blocks (spinning) until PX4 reports it is in OFFBOARD. Returns false only if
// the node starts shutting down first, or immediately true when
// g_waitForOffboard is off. Independent of replanning: see the
// g_offboardActive and g_waitForOffboard declaration comments.
bool waitForOffboardEnabled(){
	if(!g_waitForOffboard || g_offboardActive.load()){
		return true;
	}
	std::cout << "[OFFBOARD_GATE] plan solved -- holding its start setpoint until "
	          << "PX4 reports OFFBOARD before starting the trajectory clock..." << std::endl;
	while(rclcpp::ok()){
		rclcpp::spin_some(node);
		if(g_offboardActive.load()){
			std::cout << "[OFFBOARD_GATE] offboard active -- starting the flight." << std::endl;
			return true;
		}
		rclcpp::sleep_for(20ms);
	}
	return false;
}

// Solve the initial plan for whichever targeting mode is configured. Always
// anchored to the vehicle's CURRENT odometry, so calling it again later
// re-anchors the plan to wherever the vehicle actually is by then.
// Returns false on a failed solve, or on shutdown while waiting for a target.
bool solveInitialPlan(ros_replan_utils * replanner){
	if(useVisual){
		Eigen::Matrix4d H;
		if(!waitForVisualTarget(&H)){
			return false; // shutting down while still waiting
		}
		return replanner->initialPlan(3, H);
	}
	if(usePerch){
		std::cout << target <<std::endl;
		return replanner->initialPlan(3, target);
	}
	return replanner->initialPlan(4);
}

void executeOneShotTraj(std::vector<waypoint>  vertices, poscmd_publisher * controller, TrajBase * traj){
	ros_replan_utils replanner(traj, &odomListiner, &vertices, false);
	replanner.setReplanParams(g_replan_retry_step, g_replan_retry_max, g_replan_min_seg);
	replanner.setFreeStartJerkSnap(g_free_start_jerk_snap);
	bool initial_ok = solveInitialPlan(&replanner);
	if(!initial_ok){
		std::cout << "[INITIAL_PLAN] FAILED -- not publishing/commanding this trajectory." << std::endl;
		controller->setEND();
		auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
		hover_->async_send_request(trigger);
		return;
	}
	visualize_paths(traj);
	maybeLogPlannedTrajectory(traj);
	//Nulltracker transition
	auto transition_cmd = std::make_shared<trackers_msgs::srv::Transition::Request>();
	transition_cmd->tracker = null_tracker_str;
	srv_transition_->async_send_request(transition_cmd);
	//poscmd transition -- not until offboard is live, so the trajectory isn't
	//consumed (or finished outright) before the vehicle is being driven by it.
	//Hold the plan's START setpoint meanwhile: that keeps the stream PX4 (and
	//the bridge's own staleness check) needs in order to grant offboard, and
	//it holds the vehicle where the plan begins instead of where it ends.
	controller->holdTrajectoryStart(traj);
	if(!waitForOffboardEnabled()){
		return; // shutting down while still waiting
	}
	// Re-anchor before flying. The plan above was solved from a snapshot of the
	// vehicle's state, and the wait for offboard can be arbitrarily long -- the
	// vehicle is under RC/position hold throughout it and drifts, or is still
	// settling from a climb. Starting a stale plan means beginning with a
	// position AND velocity error the controller has to burn the first seconds
	// absorbing instead of tracking (measured: 1.3 m and 1.35 m/s of initial
	// mismatch, with the plan opening in a 1.1 m/s climb the vehicle was no
	// longer doing). Re-solving here costs one QP and starts the trajectory
	// from where the vehicle actually is.
	if(!solveInitialPlan(&replanner)){
		std::cout << "[OFFBOARD_GATE] re-solve at offboard FAILED -- flying the "
		          << "plan solved earlier instead." << std::endl;
	}
	else{
		visualize_paths(traj);
		maybeLogPlannedTrajectory(traj);
	}
	controller->startFlight( traj);
	while(controller->getState() != HOVER && rclcpp::ok()){
		rclcpp::spin_some(node);
	}
	// Deliberately NOT setEND() here: END makes poscmd_publisher's timer return
	// without publishing at all, so <device>/position_cmd goes dead the instant
	// the flight finishes. Whatever is relaying it to PX4 (offboard_bridge)
	// needs a continuously-fresh stream both to enter and to stay in OFFBOARD.
	// HOVER keeps republishing the trajectory's final setpoint, which is the
	// correct thing to hold at anyway. setEND() is still used on the failure
	// paths above, where there is no valid final setpoint to hold.
	auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
	hover_->async_send_request(trigger);

}

void executeReplanTraj(std::vector<waypoint>  vertices, poscmd_publisher * controller, TrajBase * traj){
	//std::cout << "number of vertices" <<vertices->size() <<std::endl;
	std::cout << "preparation initial plan " <<std::endl;
	ros_replan_utils replanner(traj, &odomListiner, &vertices, useVisual);
	replanner.setReplanParams(g_replan_retry_step, g_replan_retry_max, g_replan_min_seg);
	replanner.setFreeStartJerkSnap(g_free_start_jerk_snap);
	replanner.setAnchorOdom(g_replan_anchor_odom);
	replanner.setOdomBlend(g_replan_odom_blend);
	replanner.setFOVEnable(g_fov_enable);
	replanner.setFOVCoverageFraction(g_fov_coverage_fraction);
	bool initial_ok;
	if(useVisual){
		Eigen::Matrix4d H;
		if(!waitForVisualTarget(&H)){
			return; // shutting down while still waiting
		}
		initial_ok = replanner.initialPlan(3, H);
	}
	else if(usePerch){
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
	maybeLogPlannedTrajectory(traj_use);
	auto transition_cmd = std::make_shared<trackers_msgs::srv::Transition::Request>();
	transition_cmd->tracker = null_tracker_str;
	srv_transition_->async_send_request(transition_cmd);
	// Only gate the non-visual path. The visual path must fall through so its
	// idle re-solve loop below can keep the initial plan aimed at the moving
	// target while waiting for offboard -- and since every one of those
	// re-solves calls startFlight() again, its clock is already being reset
	// continuously, so it never runs the trajectory down before offboard.
	if(!useVisual){
		controller->holdTrajectoryStart(traj_use);
		if(!waitForOffboardEnabled()){
			return; // shutting down while still waiting
		}
	}
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
		std::cout << "replanEnabledNow " << replanEnabledNow << std::endl;
		if(replanEnabledNow){
			time_plan+=t_elap;
			if (time_plan >=replan_time){
				//std::cout << "replan start" <<std::endl;
				double replan_timer = node->now().seconds() ;
				std::cout << "useVisual " << useVisual << std::endl;
				if(useVisual){
					Eigen::Matrix4d H;
					bool haveTarget;
					if(g_freezeTargetOnOffboard){
						// [DEBUG] Capture once, then never touch aprilListen again --
						// keep retrying each cycle only until the first capture
						// succeeds (in practice this fires on the very next cycle,
						// since visual tracking was already running before offboard
						// enabled), then lock permanently until stop_replan resets it.
						if(!g_haveFrozenTarget){
							g_haveFrozenTarget = aprilListen.getLanding(&g_frozenTarget);
							if(g_haveFrozenTarget){
								std::cout << "[DEBUG] freeze_target_on_offboard: target "
								          << "captured and frozen." << std::endl;
							}
						}
						haveTarget = g_haveFrozenTarget;
						H = g_frozenTarget;
					}
					else{
						haveTarget = aprilListen.getLanding(&H);
					}
					if(haveTarget){
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
					maybeLogPlannedTrajectory(traj_use);
				}
				double replan_timer_end =  node->now().seconds() ;
				// std::cout << "Time ELAPSED " <<replan_timer_end-replan_timer <<std::endl;
				time_plan = 0.0;
			}
		}
		else{
			// Pinned at 0 while disabled -- see the member comment above.
			time_plan = 0.0;
			if(useVisual){
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
						bool redo_ok = replanner.initialPlan(3, H);
						if(redo_ok){
							traj_use = replanner.getTraj();
							controller->startFlight(traj_use);
							visualize_paths(traj_use);
							maybeLogPlannedTrajectory(traj_use);
						}
						else{
							std::cout << "[VISUAL_TARGET] initial plan toward the current "
							          << "target FAILED -- keeping the previous plan running."
							          << std::endl;
						}
					}
					visual_refresh_time = 0.0;
				}
			}
		}
	}
	std::cout << "replanning time done, take a hover" << std::endl;
	// Deliberately NOT setEND() here -- see the matching comment in
	// executeOneShotTraj: HOVER keeps the position_cmd stream alive on the
	// trajectory's final setpoint, END would kill it entirely.
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
	double dt =0.02; //Handles the timer speed
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
