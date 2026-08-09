// PadMotionPlugin -- a gz-sim Model plugin that reads a model's ACTUAL pose
// from Gazebo every tick and republishes it as px4_msgs/VehicleOdometry
// ground truth, the same message type/convention this repo's drone
// odometry already flows through (traj_manager.cpp, dummy_publisher.py) --
// so any downstream consumer (starting with spa_predictor's
// spa_heave_node) needs zero new parsing code.
//
// Deliberately a passive telemetry bridge, not a motion generator: this
// plugin does not drive, script, or otherwise decide how the pad moves --
// whatever does that (physics, a joint controller, a separate driving
// plugin, manual/GUI manipulation, ...) is a wholly separate concern, and
// this plugin has no configuration of its own. Its only job is "read
// Gazebo's ground truth for this model, publish it as if a real PX4 device
// reported it" -- the same role PX4's own gz_bridge plays for the vehicle
// itself.
//
// This targets gz-sim's Harmonic-era API (`gz::` namespace, GZ_ADD_PLUGIN).
// Fortress (Ignition-branded) users need the `ignition::gazebo` namespace
// and `IGNITION_ADD_PLUGIN` macro instead -- CMakeLists.txt selects the
// gz-sim major version at configure time, but this source file itself is
// written against the newer naming; port the namespace/macro names if
// building against gz-sim7.
//
// Scope: position + linear velocity only. Linear velocity is obtained by
// finite-differencing consecutive poses rather than reading a
// LinearVelocity component, since that component is only populated by
// systems explicitly asked to track it (gz-sim's EnableComponent) --
// differencing works regardless of how (or whether) the model's actual
// motion source cooperates with that, matching "just read whatever is
// really happening" over depending on the mover's implementation.
// Attitude is published as identity / zero angular velocity,
// unconditionally -- NOT yet a passthrough of the model's actual
// orientation. Doing that correctly needs the same composed ENU<->NED /
// FLU<->FRD fixed quaternion rotation PX4's own gz_bridge applies to the
// real vehicle's attitude (see gz_bridge.cpp) -- reuse that exact
// transform, don't re-derive it, when roll/pitch predictors need real pad
// attitude.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

namespace pad_motion
{

class PadMotionPlugin :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity& entity,
	               const std::shared_ptr<const sdf::Element>& /*sdf*/,
	               gz::sim::EntityComponentManager& ecm,
	               gz::sim::EventManager& /*eventMgr*/) override
	{
		model_ = gz::sim::Model(entity);
		if(!model_.Valid(ecm)){
			std::cerr << "[PadMotionPlugin] attached to an invalid model entity "
			          << "-- check the <plugin> is nested inside a <model>. Skipping."
			          << std::endl;
			valid_ = false;
			return;
		}

		if(!rclcpp::ok()){
			rclcpp::init(0, nullptr);
		}
		std::string modelName = model_.Name(ecm);
		// Node name includes the model name so multiple pad instances in one
		// world don't collide.
		node_ = std::make_shared<rclcpp::Node>("pad_motion_plugin_" + modelName);
		// Matches the best-effort/depth-1 QoS every other PX4 topic in this
		// workspace uses (traj_manager.cpp, dummy_publisher.py).
		rclcpp::QoS qos(1);
		qos.best_effort();
		qos.durability_volatile();
		odomPub_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
			"/pad/fmu/out/vehicle_odometry", qos);

		valid_ = true;
		std::cout << "[PadMotionPlugin] configured for model '" << modelName
		          << "' -- reading its actual Gazebo pose every tick and "
		          << "republishing as GROUND-TRUTH VehicleOdometry (NED, "
		          << "position + linear velocity only; identity attitude)."
		          << std::endl;
	}

	void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override
	{
		if(!valid_ || info.paused){
			return;
		}
		auto poseComp = ecm.Component<gz::sim::components::Pose>(model_.Entity());
		if(!poseComp){
			return; // nothing to read yet
		}
		const gz::math::Pose3d& poseEnu = poseComp->Data();
		double t = std::chrono::duration<double>(info.simTime).count();

		gz::math::Vector3d velEnu = gz::math::Vector3d::Zero;
		if(haveLast_){
			double dt = t - lastT_;
			if(dt > 1e-9){
				velEnu = (poseEnu.Pos() - lastPosEnu_) / dt;
			}
		}
		lastPosEnu_ = poseEnu.Pos();
		lastT_ = t;
		haveLast_ = true;

		publishOdometry(poseEnu, velEnu, info);
	}

private:
	void publishOdometry(const gz::math::Pose3d& poseEnu, const gz::math::Vector3d& velEnu,
	                      const gz::sim::UpdateInfo& info)
	{
		// This world assumes an ENU-convention world frame (X-East, Y-North,
		// Z-Up), matching PX4's own gz_bridge and standard ROS practice --
		// confirm before trusting this ground truth in a differently-
		// configured world.
		//   NED.x (N) = ENU.y,  NED.y (E) = ENU.x,  NED.z (D) = -ENU.z
		px4_msgs::msg::VehicleOdometry msg;
		auto simUs = std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count();
		msg.timestamp = static_cast<uint64_t>(simUs);
		msg.timestamp_sample = msg.timestamp;
		msg.position[0] = static_cast<float>(poseEnu.Pos().Y());
		msg.position[1] = static_cast<float>(poseEnu.Pos().X());
		msg.position[2] = static_cast<float>(-poseEnu.Pos().Z());
		// Attitude/angular velocity: identity/zero, unconditionally -- see
		// the class comment for why this isn't a real passthrough yet.
		msg.q = {1.0f, 0.0f, 0.0f, 0.0f};
		msg.velocity[0] = static_cast<float>(velEnu.Y());
		msg.velocity[1] = static_cast<float>(velEnu.X());
		msg.velocity[2] = static_cast<float>(-velEnu.Z());
		msg.angular_velocity = {0.0f, 0.0f, 0.0f};
		odomPub_->publish(msg);
	}

	gz::sim::Model model_;
	bool valid_ = false;

	bool haveLast_ = false;
	gz::math::Vector3d lastPosEnu_ = gz::math::Vector3d::Zero;
	double lastT_ = 0.0;

	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub_;
};

} // namespace pad_motion

GZ_ADD_PLUGIN(pad_motion::PadMotionPlugin,
              gz::sim::System,
              pad_motion::PadMotionPlugin::ISystemConfigure,
              pad_motion::PadMotionPlugin::ISystemPreUpdate)
