// Wraps SpaPredictor around one axis of the pad's ground-truth position, as
// published by pad_motion_gazebo's PadMotionPlugin on
// px4_msgs/VehicleOdometry, and publishes predictions + self-assessment as
// spa_predictor/SpaPrediction.
//
// Generic over WHICH axis via the `axis` parameter (0/1/2 -> NED
// position[0]/[1]/[2], i.e. x/North, y/East, heave/Down) -- run one
// instance per axis you want predicted (e.g. three instances, one each for
// x, y, heave) rather than three separate node implementations, matching
// SpaPredictor's own design: one scalar-signal instance per degree of
// freedom (see spa_predictor.h's class comment). Each instance gets its
// own independent mode detection/estimation -- there is no assumption that
// different axes share frequencies, which is correct (e.g. heave and sway
// can genuinely be driven at different rates).
//
// Also feeds SpaPredictor a per-sample ACCELERATION measurement (one
// finite difference of VehicleOdometry.velocity[axis_], via velToAccel_)
// alongside position -- a genuine second Kalman correction channel (see
// spa_predictor.h's class comment), not a prediction-time output. NaN
// (velToAccel_'s sentinel) until the first velocity sample has something
// to difference against.
//
// Deliberately standalone -- not wired into ros_traj_gen_utils' traj_exe
// yet, and deliberately in its own package (not ros_traj_gen_utils) so the
// predictor stays reusable/testable independent of the planner. This is
// Phase 2 of the SPA rollout (see the planning discussion this node came
// out of): validate the predictor's mode detection and accuracy-vs-
// horizon (sigma) against a known/simulated signal before anything
// consumes its output for real. Point input_topic at a real (vision-
// derived) position source later without changing this node's logic.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <spa_predictor/msg/spa_prediction.hpp>
#include <spa_predictor/spa_predictor.h>
#include <spa_predictor/finite_difference.h>
#include <limits>
#include <memory>
#include <vector>
#include <string>

namespace
{
// Friendly name for the default topic/log text only -- axis_ itself (the
// actual position[] index used) is what matters functionally.
std::string axisName(int axis)
{
	switch(axis){
		case 0: return "x";
		case 1: return "y";
		default: return "heave"; // axis 2 (NED z/Down) -- this repo's original/primary use case
	}
}
} // namespace

class SpaAxisNode : public rclcpp::Node
{
public:
	SpaAxisNode() : Node("spa_axis_node")
	{
		SpaPredictor::Config cfg; // defaults, overridden by parameters below

		// Declared and read FIRST so output_topic's own default can be
		// derived from it below (a fresh instance per axis, run with no
		// other overrides, then publishes to distinct topics instead of
		// colliding on one).
		declare_parameter("axis", 2);
		axis_ = static_cast<int>(get_parameter("axis").as_int());
		if(axis_ < 0 || axis_ > 2){
			RCLCPP_WARN(get_logger(), "axis=%d out of range [0,2] -- clamping to 2 (heave).", axis_);
			axis_ = 2;
		}

		declare_parameter("input_topic", std::string("/pad/fmu/out/vehicle_odometry"));
		declare_parameter("output_topic", std::string("/pad/spa/" + axisName(axis_) + "_prediction"));
		declare_parameter("publish_rate_hz", 10.0);
		declare_parameter("horizons_s", std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0});

		declare_parameter("t_fft_s", cfg.t_fft);
		declare_parameter("fft_grid_dt_s", cfg.fft_grid_dt);
		declare_parameter("f_min_hz", cfg.f_min_hz);
		declare_parameter("f_max_hz", cfg.f_max_hz);
		declare_parameter("peak_sensitivity", cfg.peak_sensitivity);
		declare_parameter("max_modes", cfg.max_modes);
		declare_parameter("process_noise_osc", cfg.process_noise_osc);
		declare_parameter("process_noise_offset", cfg.process_noise_offset);
		declare_parameter("measurement_noise_pos", cfg.measurement_noise_pos);
		declare_parameter("measurement_noise_accel", cfg.measurement_noise_accel);
		declare_parameter("nominal_dt_s", cfg.nominal_dt);
		declare_parameter("sigma_max_horizon_s", cfg.sigma_max_horizon);
		declare_parameter("sigma_bin_s", cfg.sigma_bin_s);

		cfg.t_fft = get_parameter("t_fft_s").as_double();
		cfg.fft_grid_dt = get_parameter("fft_grid_dt_s").as_double();
		cfg.f_min_hz = get_parameter("f_min_hz").as_double();
		cfg.f_max_hz = get_parameter("f_max_hz").as_double();
		cfg.peak_sensitivity = get_parameter("peak_sensitivity").as_double();
		cfg.max_modes = static_cast<int>(get_parameter("max_modes").as_int());
		cfg.process_noise_osc = get_parameter("process_noise_osc").as_double();
		cfg.process_noise_offset = get_parameter("process_noise_offset").as_double();
		cfg.measurement_noise_pos = get_parameter("measurement_noise_pos").as_double();
		cfg.measurement_noise_accel = get_parameter("measurement_noise_accel").as_double();
		cfg.nominal_dt = get_parameter("nominal_dt_s").as_double();
		cfg.sigma_max_horizon = get_parameter("sigma_max_horizon_s").as_double();
		cfg.sigma_bin_s = get_parameter("sigma_bin_s").as_double();
		predictor_ = std::make_unique<SpaPredictor>(cfg);

		horizons_ = get_parameter("horizons_s").as_double_array();

		// Matches the best-effort/depth-1 QoS every other PX4 topic in this
		// repo uses (traj_manager.cpp, dummy_publisher.py, offboard_bridge.py).
		rclcpp::QoS px4_qos(1);
		px4_qos.best_effort();
		px4_qos.durability_volatile();

		std::string input_topic = get_parameter("input_topic").as_string();
		sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
			input_topic, px4_qos,
			std::bind(&SpaAxisNode::onOdom, this, std::placeholders::_1));

		std::string output_topic = get_parameter("output_topic").as_string();
		pub_ = create_publisher<spa_predictor::msg::SpaPrediction>(output_topic, 10);

		double rate = get_parameter("publish_rate_hz").as_double();
		timer_ = create_wall_timer(
			std::chrono::duration<double>(1.0 / rate),
			std::bind(&SpaAxisNode::onTimer, this));

		RCLCPP_INFO(get_logger(),
			"spa_axis_node up: axis=%d (%s, NED position[%d]) %s -> %s @ %.1f Hz, "
			"t_fft=%.1fs, %zu horizons",
			axis_, axisName(axis_).c_str(), axis_, input_topic.c_str(),
			output_topic.c_str(), rate, cfg.t_fft, horizons_.size());
	}

private:
	void onOdom(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
	{
		// PX4 timestamps are microseconds since boot/epoch (matches the rest
		// of this repo's px4_msgs handling, e.g. poscmd_publisher/dummy_publisher).
		double t = static_cast<double>(msg->timestamp) * 1e-6;
		// NED: position[0]/[1]/[2] = North/East/Down. For axis=2 (heave),
		// "heave up" is therefore a DECREASE in this value -- same
		// convention this whole repo uses throughout (TrajBase/QPpolyTraj/
		// apriltag_utils are all NED-native).
		double value = static_cast<double>(msg->position[axis_]);

		// Acceleration for the ESTIMATOR (not exposed in predictions -- see
		// spa_predictor.h's class comment): one finite difference of the
		// ALREADY-MEASURED velocity (VehicleOdometry.velocity[axis_]), not a
		// second difference of position -- avoids compounding two
		// differentiation noise amplifications. NaN (velToAccel_'s pre-set
		// sentinel) on the first sample, when no previous velocity exists
		// yet to difference against.
		double accel = std::numeric_limits<double>::quiet_NaN();
		velToAccel_.Update(t, static_cast<double>(msg->velocity[axis_]), &accel);
		lastAccel_ = accel; // published as-is in onTimer() -- see msg.measured_accel

		predictor_->addMeasurement(t, value, accel);
	}

	void onTimer()
	{
		if(!predictor_->initialized()){
			return;
		}
		spa_predictor::msg::SpaPrediction msg;
		msg.header.stamp = now();
		msg.header.frame_id = "odom"; // NED, matches this repo's odom_frame convention
		msg.initialized = true;
		// See made_at_t's .msg comment: the actual time base horizon_s is
		// relative to, NOT header.stamp (ROS time) -- these are only the
		// same clock by coincidence, since addMeasurement() is fed PX4
		// timestamp-derived seconds (see onOdom()).
		msg.made_at_t = predictor_->lastMeasurementTime();

		double filtered = 0.0;
		predictor_->predict(0.0, &filtered);
		msg.filtered_value = filtered;
		// See msg.measured_accel's .msg comment -- the RAW input the
		// estimator was corrected against (or NaN), not a predicted value.
		msg.measured_accel = lastAccel_;

		for(const auto& m : predictor_->currentModes()){
			msg.mode_freq_hz.push_back(m.freq_hz);
			msg.mode_amplitude.push_back(m.amplitude);
			msg.mode_phase_rad.push_back(m.phase_rad);
		}
		msg.offset = predictor_->currentOffset();

		std::vector<double> velocity;
		std::vector<double> predicted = predictor_->predictAndAssess(horizons_, &velocity);
		msg.horizon_s = horizons_;
		msg.predicted_value = predicted;
		msg.predicted_velocity = velocity;
		msg.sigma_s.reserve(horizons_.size());
		for(double h : horizons_){
			msg.sigma_s.push_back(predictor_->sigmaAtHorizon(h));
		}

		msg.dropped_samples = predictor_->droppedSamples();
		pub_->publish(msg);
	}

	int axis_ = 2;
	std::unique_ptr<SpaPredictor> predictor_;
	FiniteDifference velToAccel_;
	double lastAccel_ = std::numeric_limits<double>::quiet_NaN();
	std::vector<double> horizons_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
	rclcpp::Publisher<spa_predictor::msg::SpaPrediction>::SharedPtr pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpaAxisNode>());
	rclcpp::shutdown();
	return 0;
}
