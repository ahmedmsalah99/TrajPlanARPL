// Geometric (flatness-based) trajectory-tracking controller node.
//
// Subscribes to this repo's own quadrotor_msgs::msg::PositionCommand (the
// planner's differentially-flat output: position/velocity/acceleration/
// jerk/yaw/yaw_dot/kx/kv) and PX4's vehicle_odometry, runs
// geometric_control::GeometricController every tick, and publishes the
// result directly to PX4 as a body-rate + thrust command
// (VehicleRatesSetpoint) -- bypassing PX4's own outer position/attitude
// loops entirely, since those weren't built to hold the tight terminal
// tracking a perch approach needs.
//
// This node owns the WHOLE PX4-facing offboard sequence -- not just the
// setpoint stream:
//   1. enable_offboard starts the OffboardControlMode(body_rate=true)
//      heartbeat (and the dummy <device>/waypoints trigger traj_exe's
//      planner needs to start planning at all).
//   2. Once the heartbeat has been flowing for offboard_prime_s (PX4's own
//      precondition for entering offboard), sends a single DO_SET_MODE.
//   3. Once VehicleStatus confirms OFFBOARD is actually active, calls
//      traj_exe's start_replan service.
//   4. Once nav_state is OFFBOARD, starts publishing the actual
//      VehicleRatesSetpoint computed from PositionCommand + odometry.
// disable_offboard reverses all of it (stops the heartbeat, calls
// stop_replan). There is deliberately no other node also touching
// /fmu/in/offboard_control_mode -- PX4 only accepts one consistent
// declaration at a time.
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <ros_traj_gen_utils/geometric_controller.h>

#include <chrono>
#include <memory>
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

geometric_control::VehicleState vehicleOdometryToState(const px4_msgs::msg::VehicleOdometry& msg){
    geometric_control::VehicleState s;
    s.position = Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
    s.velocity = Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    // PX4 quaternion is [w, x, y, z], body-FRD expressed in NED -- same
    // passthrough convention as vehicleOdometryToRosOdometry in
    // traj_manager.cpp (no ENU/FLU flip anywhere in this pipeline).
    s.orientation = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    return s;
}

geometric_control::FlatState positionCommandToFlatState(const quadrotor_msgs::msg::PositionCommand& msg){
    geometric_control::FlatState f;
    f.position = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    f.velocity = Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    f.acceleration = Eigen::Vector3d(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
    f.jerk = Eigen::Vector3d(msg.jerk.x, msg.jerk.y, msg.jerk.z);
    f.yaw = msg.yaw;
    f.yaw_dot = msg.yaw_dot;
    return f;
}

geometric_control::PositionGains positionCommandToGains(const quadrotor_msgs::msg::PositionCommand& msg){
    geometric_control::PositionGains g;
    g.kx = Eigen::Vector3d(msg.kx[0], msg.kx[1], msg.kx[2]);
    g.kv = Eigen::Vector3d(msg.kv[0], msg.kv[1], msg.kv[2]);
    return g;
}

// Linear thrust-curve approximation: calibrated so commanding exactly
// `gravity` (specific thrust at hover) yields `hover_thrust_normalized` --
// the same linear-throttle assumption every PX4-native reference we looked
// at uses (e.g. px4-mpc's own thrust_rates[0]*0.07 calibration).
double specificThrustToNormalized(double thrust_specific, double gravity, double hover_thrust_normalized){
    double coeff = hover_thrust_normalized / gravity;
    return thrust_specific * coeff;
}

} // namespace

class So3ControlNode : public rclcpp::Node {
public:
    So3ControlNode() : rclcpp::Node("so3_control_node") {
        device_ = declare_parameter<std::string>("device", "/quadrotor");
        mass_ = declare_parameter<double>("mass", 1.0);
        gravity_ = declare_parameter<double>("gravity", 9.81);
        hover_thrust_ = declare_parameter<double>("hover_thrust", 0.5);
        max_body_rate_ = declare_parameter<double>("max_body_rate", 6.0);
        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 100.0);
        command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.25);
        // How long to stream the heartbeat after enable_offboard before
        // sending DO_SET_MODE -- gives PX4 time to see the stream as
        // established (its own precondition for offboard).
        offboard_prime_s_ = declare_parameter<double>("offboard_prime_s", 1.0);
        // After DO_SET_MODE, how long to wait for VehicleStatus to confirm
        // OFFBOARD before logging a one-time non-fatal "taking a while" warning.
        offboard_confirm_timeout_s_ = declare_parameter<double>("offboard_confirm_timeout_s", 5.0);
        std::vector<double> kR_vec = declare_parameter<std::vector<double>>(
            "kR", std::vector<double>{8.0, 8.0, 3.0});
        Eigen::Vector3d kR(kR_vec.size() > 0 ? kR_vec[0] : 8.0,
                           kR_vec.size() > 1 ? kR_vec[1] : 8.0,
                           kR_vec.size() > 2 ? kR_vec[2] : 3.0);

        controller_ = std::make_unique<geometric_control::GeometricController>(mass_, gravity_, kR);

        auto best_effort_qos = rclcpp::QoS(1).best_effort();

        cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
            device_ + "/position_cmd", 10,
            [this](const quadrotor_msgs::msg::PositionCommand& msg){ onPositionCommand(msg); });
        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", best_effort_qos,
            [this](const px4_msgs::msg::VehicleOdometry& msg){ onVehicleOdometry(msg); });
        status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", best_effort_qos,
            [this](const px4_msgs::msg::VehicleStatus& msg){ nav_state_ = msg.nav_state; });

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        rates_pub_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
            "/fmu/in/vehicle_rates_setpoint", 10);
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        // Dummy waypoint trigger: traj_manager.cpp's main loop only ever
        // calls initialPlan() once <device>/waypoints gets a message at all
        // (see ros_waypoint_utils::waypointListiner) -- the actual perch
        // target comes from AprilTag/config, this is purely the "go plan
        // something" kick. Published unconditionally (not gated on
        // streaming_) so traj_exe can start planning immediately, before
        // offboard is ever requested.
        wp_pub_ = create_publisher<nav_msgs::msg::Path>(device_ + "/waypoints", 10);

        start_replan_client_ = create_client<std_srvs::srv::Trigger>(device_ + "/start_replan");
        stop_replan_client_ = create_client<std_srvs::srv::Trigger>(device_ + "/stop_replan");

        enable_srv_ = create_service<std_srvs::srv::Trigger>(
            device_ + "/enable_offboard",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response){
                onEnableOffboard(response);
            });
        disable_srv_ = create_service<std_srvs::srv::Trigger>(
            device_ + "/disable_offboard",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response){
                onDisableOffboard(response);
            });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_hz_),
            [this](){ tick(); });
    }

private:
    void onPositionCommand(const quadrotor_msgs::msg::PositionCommand& msg){
        last_cmd_ = msg;
        have_cmd_ = true;
        last_cmd_time_ = now();
    }

    void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry& msg){
        last_state_ = vehicleOdometryToState(msg);
        have_state_ = true;
    }

    void onEnableOffboard(std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        if(streaming_){
            response->success = false;
            response->message = "Already streaming (call disable_offboard first to restart).";
            return;
        }
        if(!have_cmd_){
            response->success = false;
            response->message = "No PositionCommand received yet -- refusing to enable.";
            return;
        }
        double age_s = (now() - last_cmd_time_).seconds();
        if(age_s > command_timeout_s_){
            response->success = false;
            response->message = "Last PositionCommand is " + std::to_string(age_s) +
                "s old -- refusing to enable.";
            return;
        }

        streaming_ = true;
        streaming_since_ = now();
        mode_cmd_sent_ = false;
        replan_started_ = false;
        warned_confirm_slow_ = false;
        response->success = true;
        response->message = "Streaming started, requesting OFFBOARD in " +
            std::to_string(offboard_prime_s_) + "s.";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    void onDisableOffboard(std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        streaming_ = false;
        mode_cmd_sent_ = false;
        bool was_replanning = replan_started_;
        replan_started_ = false;
        warned_confirm_slow_ = false;
        if(was_replanning){
            if(stop_replan_client_->service_is_ready()){
                stop_replan_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            } else {
                RCLCPP_WARN(get_logger(),
                    "stop_replan service not available -- traj_exe may not be running.");
            }
        }
        response->success = true;
        response->message = "Streaming stopped." + std::string(was_replanning ? " Replanning stopped too." : "");
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    void sendOffboardModeCommand(){
        // DO_SET_MODE(base_mode=CUSTOM, custom_main_mode=OFFBOARD) -- the
        // same single command PX4's own ROS2 offboard example sends after
        // priming the stream.
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = nowUs();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2 = 6.0;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void checkOffboardConfirmedAndStartReplan(){
        if(nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
            replan_started_ = true;
            warned_confirm_slow_ = false;
            RCLCPP_INFO(get_logger(), "VehicleStatus confirms OFFBOARD is active -- calling start_replan.");
            if(start_replan_client_->service_is_ready()){
                start_replan_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            } else {
                RCLCPP_WARN(get_logger(),
                    "start_replan service not available -- traj_exe may not be running.");
            }
            return;
        }
        double wait_s = (now() - mode_cmd_sent_at_).seconds();
        if(wait_s > offboard_confirm_timeout_s_ && !warned_confirm_slow_){
            RCLCPP_WARN(get_logger(),
                "DO_SET_MODE sent %.1fs ago but VehicleStatus still reports nav_state=%d, "
                "not OFFBOARD (%d) -- check the RC switch/QGC mode selector. Still watching.",
                wait_s, nav_state_, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
            warned_confirm_slow_ = true;
        }
    }

    void tick(){
        // Unconditional: feeds traj_manager.cpp's waypoint listener --
        // unrelated to offboard mode, so traj_exe can start planning before
        // enable_offboard is ever called.
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = "world";
        path.poses.push_back(geometry_msgs::msg::PoseStamped());
        wp_pub_->publish(path);

        if(!streaming_){
            return; // stream is off until enable_offboard is called
        }

        // Heartbeat must keep flowing regardless of command freshness --
        // it's PX4's own precondition for entering/staying in offboard.
        px4_msgs::msg::OffboardControlMode heartbeat{};
        heartbeat.timestamp = nowUs();
        heartbeat.position = false;
        heartbeat.velocity = false;
        heartbeat.acceleration = false;
        heartbeat.attitude = false;
        heartbeat.body_rate = true;
        offboard_pub_->publish(heartbeat);

        // Mode-switch/replan-start sequencing -- independent of whether a
        // fresh PositionCommand exists this exact tick.
        if(!mode_cmd_sent_){
            double elapsed_s = (now() - streaming_since_).seconds();
            if(elapsed_s >= offboard_prime_s_){
                sendOffboardModeCommand();
                mode_cmd_sent_ = true;
                mode_cmd_sent_at_ = now();
                RCLCPP_INFO(get_logger(),
                    "Streamed for %.2fs, requesting OFFBOARD mode switch.", elapsed_s);
            }
        } else if(!replan_started_){
            checkOffboardConfirmedAndStartReplan();
        }

        if(!have_cmd_ || !have_state_){
            return; // nothing to compute a rate setpoint from yet
        }
        double age_s = (now() - last_cmd_time_).seconds();
        if(age_s > command_timeout_s_){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No PositionCommand in %.2fs (timeout=%.2fs) -- holding off on a "
                "rate setpoint this tick.", age_s, command_timeout_s_);
            return;
        }

        geometric_control::FlatState flat = positionCommandToFlatState(last_cmd_);
        geometric_control::PositionGains gains = positionCommandToGains(last_cmd_);
        geometric_control::RateThrustCommand cmd = controller_->compute(flat, last_state_, gains);

        // Safety clamp: guard against a transient reference/estimate glitch
        // commanding an extreme rate.
        Eigen::Vector3d rate = cmd.body_rate;
        double rate_norm = rate.norm();
        if(rate_norm > max_body_rate_){
            rate *= (max_body_rate_ / rate_norm);
        }
        double thrust_norm = std::clamp(
            specificThrustToNormalized(cmd.thrust_specific, gravity_, hover_thrust_), 0.0, 1.0);

        // Only actually send the vehicle a rate setpoint once PX4 confirms
        // offboard is active (same gating as px4-mpc's own cmdloop) -- the
        // heartbeat above still needs to flow beforehand to make that
        // confirmation possible in the first place.
        if(nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
            return;
        }

        px4_msgs::msg::VehicleRatesSetpoint setpoint{};
        setpoint.timestamp = nowUs();
        // Body-FRD rates, no sign flip -- this pipeline works natively in
        // NED/FRD throughout, unlike ENU/FLU-internal references.
        setpoint.roll = static_cast<float>(rate.x());
        setpoint.pitch = static_cast<float>(rate.y());
        setpoint.yaw = static_cast<float>(rate.z());
        setpoint.thrust_body[0] = 0.0f;
        setpoint.thrust_body[1] = 0.0f;
        // Thrust acts along -body_z (FRD: z points down through the
        // vehicle) -- PX4's own fixed message-level convention for this
        // field, independent of any sending node's internal frame choice.
        setpoint.thrust_body[2] = -static_cast<float>(thrust_norm);
        rates_pub_->publish(setpoint);
    }

    uint64_t nowUs(){
        return static_cast<uint64_t>(now().nanoseconds() / 1000);
    }

    std::string device_;
    double mass_, gravity_, hover_thrust_, max_body_rate_, control_rate_hz_, command_timeout_s_;
    double offboard_prime_s_, offboard_confirm_timeout_s_;

    std::unique_ptr<geometric_control::GeometricController> controller_;

    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wp_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_replan_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_replan_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool streaming_ = false;
    rclcpp::Time streaming_since_;
    bool mode_cmd_sent_ = false;
    rclcpp::Time mode_cmd_sent_at_;
    bool replan_started_ = false;
    bool warned_confirm_slow_ = false;

    bool have_cmd_ = false;
    bool have_state_ = false;
    quadrotor_msgs::msg::PositionCommand last_cmd_;
    geometric_control::VehicleState last_state_;
    rclcpp::Time last_cmd_time_;
    uint8_t nav_state_ = 0;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<So3ControlNode>());
    rclcpp::shutdown();
    return 0;
}
