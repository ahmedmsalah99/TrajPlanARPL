// Wraps SpaPredictor around the pad's ground-truth heave (z), as published
// by pad_motion_gazebo's PadMotionPlugin on px4_msgs/VehicleOdometry, and
// publishes predictions + self-assessment as ros_traj_gen_utils/SpaPrediction.
//
// Deliberately standalone -- not wired into traj_manager.cpp/traj_exe yet.
// This is Phase 2 of the SPA rollout (see the planning discussion this node
// came out of): validate the predictor's mode detection and accuracy-vs-
// horizon (sigma) against a known/simulated signal before anything consumes
// its output for real. Point input_topic at a real (vision-derived) heave
// source later without changing this node's logic.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <ros_traj_gen_utils/msg/spa_prediction.hpp>
#include <ros_traj_gen_utils/spa_predictor.h>
#include <memory>
#include <vector>
#include <string>

class SpaHeaveNode : public rclcpp::Node
{
public:
	SpaHeaveNode() : Node("spa_heave_node")
	{
		SpaPredictor::Config cfg; // defaults, overridden by parameters below

		declare_parameter("input_topic", std::string("/pad/fmu/out/vehicle_odometry"));
		declare_parameter("output_topic", std::string("/pad/spa/heave_prediction"));
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
		// repo uses (traj_manager.cpp, dummy_publisher.py, offboard_bridge.py).
		rclcpp::QoS px4_qos(1);
		px4_qos.best_effort();
		px4_qos.durability_volatile();

		std::string input_topic = get_parameter("input_topic").as_string();
		sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
			input_topic, px4_qos,
			std::bind(&SpaHeaveNode::onOdom, this, std::placeholders::_1));

		std::string output_topic = get_parameter("output_topic").as_string();
		pub_ = create_publisher<ros_traj_gen_utils::msg::SpaPrediction>(output_topic, 10);

		double rate = get_parameter("publish_rate_hz").as_double();
		timer_ = create_wall_timer(
			std::chrono::duration<double>(1.0 / rate),
			std::bind(&SpaHeaveNode::onTimer, this));

		RCLCPP_INFO(get_logger(),
			"spa_heave_node up: %s (heave/z, NED) -> %s @ %.1f Hz, t_fft=%.1fs, %zu horizons",
			input_topic.c_str(), output_topic.c_str(), rate, cfg.t_fft, horizons_.size());
	}

private:
	void onOdom(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
	{
		// PX4 timestamps are microseconds since boot/epoch (matches the rest
		// of this repo's px4_msgs handling, e.g. poscmd_publisher/dummy_publisher).
		double t = static_cast<double>(msg->timestamp) * 1e-6;
		// NED: position[2] is Down-positive, same convention this whole repo
		// uses throughout (TrajBase/QPpolyTraj/apriltag_utils are all NED-
		// native) -- "heave up" is therefore a DECREASE in this value.
		double z = static_cast<double>(msg->position[2]);
		predictor_->addMeasurement(t, z);
	}

	void onTimer()
	{
		if(!predictor_->initialized()){
			return;
		}
		ros_traj_gen_utils::msg::SpaPrediction msg;
		msg.header.stamp = now();
		msg.header.frame_id = "odom"; // NED, matches this repo's odom_frame convention
		msg.initialized = true;

		double filtered = 0.0;
		predictor_->predict(0.0, &filtered);
		msg.filtered_value = filtered;

		for(const auto& m : predictor_->currentModes()){
			msg.mode_freq_hz.push_back(m.freq_hz);
			msg.mode_amplitude.push_back(m.amplitude);
			msg.mode_phase_rad.push_back(m.phase_rad);
		}
		msg.offset = predictor_->currentOffset();

		std::vector<double> predicted = predictor_->predictAndAssess(horizons_);
		msg.horizon_s = horizons_;
		msg.predicted_value = predicted;
		msg.sigma_s.reserve(horizons_.size());
		for(double h : horizons_){
			msg.sigma_s.push_back(predictor_->sigmaAtHorizon(h));
		}

		msg.dropped_samples = predictor_->droppedSamples();
		pub_->publish(msg);
	}

	std::unique_ptr<SpaPredictor> predictor_;
	std::vector<double> horizons_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
	rclcpp::Publisher<ros_traj_gen_utils::msg::SpaPrediction>::SharedPtr pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpaHeaveNode>());
	rclcpp::shutdown();
	return 0;
}
