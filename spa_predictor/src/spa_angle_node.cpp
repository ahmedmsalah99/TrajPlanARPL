// Wraps SpaPredictor around the pad's roll or pitch angle, extracted from
// the attitude quaternion published by pad_motion_gazebo's PadMotionPlugin
// on px4_msgs/VehicleOdometry, and publishes predictions + self-assessment
// as spa_predictor/SpaPrediction. Sibling to spa_axis_node.cpp (which
// covers x/y/heave via VehicleOdometry.position[]) -- this one covers the
// two tilt degrees of freedom via VehicleOdometry.q instead.
//
// Generic over WHICH angle via the `angle` parameter (0/1 -> roll/pitch)
// -- run one instance per angle you want predicted, matching
// SpaPredictor's own design: one scalar-signal instance per degree of
// freedom (see spa_predictor.h's class comment). Each instance gets its
// own independent mode detection -- no assumption that roll and pitch (or
// either of them and x/y/heave) share frequencies.
//
// predicted_velocity here is the predicted ANGULAR RATE (roll rate / pitch
// rate) at each horizon -- SpaPredictor::predictWithVelocity() gives the
// time-derivative of whatever scalar was fed in "for free", and feeding it
// an angle makes that derivative an angular rate automatically, same
// mechanism as spa_axis_node.cpp getting linear velocity for free from
// position.
//
// Roll/pitch (not the surface-normal components s3x/s3y that
// TrajBase::calcPerchCond() actually consumes) were chosen deliberately
// here: below the perch tilt ceiling this whole system operates under
// (~10-25 deg, see the horiz_accel_limit/inclination discussion), Euler
// angle and s3 component agree to within ~1%, and Euler angles are the
// more directly interpretable/verifiable quantity for THIS validation
// step. Reconstructing s3 from a predicted (roll, pitch) pair when wiring
// this into calcPerchCond later is a well-defined follow-up (s3 = third
// column of the rotation matrix built from predicted roll/pitch and a
// yaw reference), not implemented here.
//
// Deliberately standalone -- not wired into ros_traj_gen_utils' traj_exe
// yet; this is Phase 2 of the SPA rollout (validate against a
// known/simulated signal before anything consumes its output for real).
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <spa_predictor/msg/spa_prediction.hpp>
#include <spa_predictor/spa_predictor.h>
#include <cmath>
#include <memory>
#include <vector>
#include <string>

namespace
{
// Friendly name for the default topic/log text only -- angle_ itself is
// what matters functionally.
std::string angleName(int angle)
{
	return (angle == 0) ? "roll" : "pitch";
}

// Quaternion (w,x,y,z) -> (roll, pitch), aerospace ZYX / NED-FRD
// convention -- matches traj_gen/traj_utils/quaternion.h's
// ToEulerAngles() exactly (verified against it, not re-derived from
// scratch), reimplemented locally rather than depending on traj_gen so
// this package stays standalone (see its package.xml description).
// asin() is guarded against |sinp| > 1 from floating-point roundoff at
// the poles (pitch = +/-90 deg) the same way that function does.
void quatToRollPitch(double w, double x, double y, double z, double* roll, double* pitch)
{
	double sinr_cosp = 2.0 * (w * x + y * z);
	double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
	*roll = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = 2.0 * (w * y - z * x);
	if(std::fabs(sinp) >= 1.0){
		*pitch = std::copysign(M_PI / 2.0, sinp);
	}
	else{
		*pitch = std::asin(sinp);
	}
}
} // namespace

class SpaAngleNode : public rclcpp::Node
{
public:
	SpaAngleNode() : Node("spa_angle_node")
	{
		SpaPredictor::Config cfg; // defaults, overridden by parameters below

		// Declared and read FIRST so output_topic's own default can be
		// derived from it below.
		declare_parameter("angle", 0);
		angle_ = static_cast<int>(get_parameter("angle").as_int());
		if(angle_ != 0 && angle_ != 1){
			RCLCPP_WARN(get_logger(), "angle=%d out of range [0,1] -- clamping to 0 (roll).", angle_);
			angle_ = 0;
		}

		declare_parameter("input_topic", std::string("/pad/fmu/out/vehicle_odometry"));
		declare_parameter("output_topic", std::string("/pad/spa/" + angleName(angle_) + "_prediction"));
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
		declare_parameter("measurement_noise", cfg.measurement_noise);
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
		cfg.measurement_noise = get_parameter("measurement_noise").as_double();
		cfg.nominal_dt = get_parameter("nominal_dt_s").as_double();
		cfg.sigma_max_horizon = get_parameter("sigma_max_horizon_s").as_double();
		cfg.sigma_bin_s = get_parameter("sigma_bin_s").as_double();
		predictor_ = std::make_unique<SpaPredictor>(cfg);

		horizons_ = get_parameter("horizons_s").as_double_array();

		// Matches the best-effort/depth-1 QoS every other PX4 topic in this
		// repo uses (traj_manager.cpp, dummy_publisher.py, spa_axis_node.cpp).
		rclcpp::QoS px4_qos(1);
		px4_qos.best_effort();
		px4_qos.durability_volatile();

		std::string input_topic = get_parameter("input_topic").as_string();
		sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
			input_topic, px4_qos,
			std::bind(&SpaAngleNode::onOdom, this, std::placeholders::_1));

		std::string output_topic = get_parameter("output_topic").as_string();
		pub_ = create_publisher<spa_predictor::msg::SpaPrediction>(output_topic, 10);

		double rate = get_parameter("publish_rate_hz").as_double();
		timer_ = create_wall_timer(
			std::chrono::duration<double>(1.0 / rate),
			std::bind(&SpaAngleNode::onTimer, this));

		RCLCPP_INFO(get_logger(),
			"spa_angle_node up: angle=%d (%s, from VehicleOdometry.q) %s -> %s @ %.1f Hz, "
			"t_fft=%.1fs, %zu horizons",
			angle_, angleName(angle_).c_str(), input_topic.c_str(),
			output_topic.c_str(), rate, cfg.t_fft, horizons_.size());
	}

private:
	void onOdom(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
	{
		// PX4 timestamps are microseconds since boot/epoch (matches the rest
		// of this repo's px4_msgs handling).
		double t = static_cast<double>(msg->timestamp) * 1e-6;
		// q = [w, x, y, z] -- matches this repo's existing VehicleOdometry.q
		// convention (traj_manager.cpp, dummy_publisher.py, PadMotionPlugin.cc).
		double roll, pitch;
		quatToRollPitch(msg->q[0], msg->q[1], msg->q[2], msg->q[3], &roll, &pitch);
		double value = (angle_ == 0) ? roll : pitch;
		predictor_->addMeasurement(t, value);
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

		for(const auto& m : predictor_->currentModes()){
			msg.mode_freq_hz.push_back(m.freq_hz);
			msg.mode_amplitude.push_back(m.amplitude);
			msg.mode_phase_rad.push_back(m.phase_rad);
		}
		msg.offset = predictor_->currentOffset();

		std::vector<double> angularRate;
		std::vector<double> predicted = predictor_->predictAndAssess(horizons_, &angularRate);
		msg.horizon_s = horizons_;
		msg.predicted_value = predicted;
		msg.predicted_velocity = angularRate;
		msg.sigma_s.reserve(horizons_.size());
		for(double h : horizons_){
			msg.sigma_s.push_back(predictor_->sigmaAtHorizon(h));
		}

		msg.dropped_samples = predictor_->droppedSamples();
		pub_->publish(msg);
	}

	int angle_ = 0;
	std::unique_ptr<SpaPredictor> predictor_;
	std::vector<double> horizons_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
	rclcpp::Publisher<spa_predictor::msg::SpaPrediction>::SharedPtr pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpaAngleNode>());
	rclcpp::shutdown();
	return 0;
}
