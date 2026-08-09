// PadMotionPlugin -- a gz-sim Model plugin that kinematically animates a
// perch pad's heave motion (a configurable sum of sinusoids) and publishes
// its GROUND-TRUTH pose/velocity as px4_msgs/VehicleOdometry, reusing the
// exact message type this repo's drone odometry already flows through
// (traj_manager.cpp, dummy_publisher.py) so any downstream consumer -- most
// immediately ros_traj_gen_utils' spa_heave_node -- needs zero new parsing
// code.
//
// This targets gz-sim's Harmonic-era API (`gz::` namespace, GZ_ADD_PLUGIN).
// Fortress (Ignition-branded) users need the `ignition::gazebo` namespace
// and `IGNITION_ADD_PLUGIN` macro instead -- CMakeLists.txt selects the
// gz-sim major version at configure time, but this source file itself is
// written against the newer naming; port the namespace/macro names if
// building against gz-sim7.
//
// Scope: HEAVE ONLY. The pad's attitude is left at whatever <pose> it was
// given at spawn and never touched here -- roll/pitch motion is a planned
// follow-up (see the SPA planning discussion this package came out of), not
// yet implemented. Adding it means: (1) writing a rotating pose, not just a
// Z offset, into the Pose component below; (2) publishing a non-identity
// `q` in VehicleOdometry, which -- unlike position/velocity's simple axis
// swap -- needs the SAME composed ENU<->NED / FLU<->FRD fixed quaternion
// rotation PX4's own gz_bridge (px4-gazebo-bridge, gz_bridge.cpp) applies to
// the real vehicle's attitude. Reuse that exact transform rather than
// re-deriving it, so the pad's ground truth stays directly comparable to
// the drone's own (identically-converted) odometry in this same simulated
// world.
//
// Kinematic driving requires <static>true</static> on the pad model in SDF
// -- this plugin overwrites the model's Pose component every PreUpdate,
// which conflicts with letting the physics engine integrate it. This is the
// standard pattern for scripted/keyframe motion in gz-sim (a static entity
// whose pose a plugin still legally mutates via the ECM every tick).
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace pad_motion
{

struct HeaveComponent
{
	double amplitude = 0.0;  // m
	double period_s = 1.0;   // s
	double phase_rad = 0.0;  // rad, at sim time t=0
};

class PadMotionPlugin :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity& entity,
	               const std::shared_ptr<const sdf::Element>& sdf,
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

		// Heave is a pure world-Z offset added on top of whatever pose this
		// model was placed at (world file's own <pose>, or wherever a
		// parent <include> put it) -- NOT necessarily the world origin.
		// <mean_height>, if given, overrides just the Z component of that
		// reference (e.g. to heave about an exact height regardless of the
		// spawn pose's own Z).
		auto poseComp = ecm.Component<gz::sim::components::Pose>(model_.Entity());
		referencePose_ = poseComp ? poseComp->Data() : gz::math::Pose3d::Zero;
		if(sdf->HasElement("mean_height")){
			referencePose_.Pos().Z() = sdf->Get<double>("mean_height", referencePose_.Pos().Z());
		}

		if(sdf->HasElement("heave_component")){
			sdf::ElementConstPtr elem = sdf->FindElement("heave_component");
			while(elem){
				HeaveComponent hc;
				hc.amplitude = elem->Get<double>("amplitude", 0.0);
				hc.period_s = elem->Get<double>("period_s", 1.0);
				hc.phase_rad = elem->Get<double>("phase_rad", 0.0);
				if(hc.period_s > 1e-6 && hc.amplitude != 0.0){
					components_.push_back(hc);
				}
				elem = elem->GetNextElement("heave_component");
			}
		}
		if(components_.empty()){
			// Sensible default so the plugin visibly does something even
			// with no <heave_component> configured, rather than silently
			// sitting still.
			components_.push_back({0.3, 4.0, 0.0});
			std::cout << "[PadMotionPlugin] no <heave_component> configured -- "
			          << "defaulting to a single 0.3m/4.0s tone." << std::endl;
		}

		odomTopic_ = sdf->Get<std::string>("odometry_topic", "/pad/fmu/out/vehicle_odometry");
		publishRateHz_ = sdf->Get<double>("publish_rate_hz", 50.0);
		publishPeriod_ = (publishRateHz_ > 0.0) ? (1.0 / publishRateHz_) : 0.0;

		if(!rclcpp::ok()){
			rclcpp::init(0, nullptr);
		}
		std::string modelName = model_.Name(ecm);
		// Node name includes the model name so multiple pad instances in
		// one world don't collide.
		node_ = std::make_shared<rclcpp::Node>("pad_motion_plugin_" + modelName);
		// Matches the best-effort/depth-1 QoS every other PX4 topic in this
		// workspace uses (traj_manager.cpp, dummy_publisher.py).
		rclcpp::QoS qos(1);
		qos.best_effort();
		qos.durability_volatile();
		odomPub_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(odomTopic_, qos);

		valid_ = true;
		std::cout << "[PadMotionPlugin] configured for model '" << modelName << "': "
		          << components_.size() << " heave component(s) about mean height "
		          << referencePose_.Pos().Z() << "m, publishing GROUND-TRUTH "
		          << "VehicleOdometry on " << odomTopic_
		          << " @ " << publishRateHz_ << "Hz (NED, identity attitude -- heave-only scope)."
		          << std::endl;
	}

	void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override
	{
		if(!valid_ || info.paused){
			return;
		}
		double t = std::chrono::duration<double>(info.simTime).count();

		double zOffset = 0.0; // world-Z (up), relative to referencePose_
		double zRate = 0.0;
		for(const auto& hc : components_){
			double w = 2.0 * M_PI / hc.period_s;
			double theta = w * t + hc.phase_rad;
			zOffset += hc.amplitude * std::sin(theta);
			zRate += hc.amplitude * w * std::cos(theta);
		}

		gz::math::Pose3d newPose = referencePose_;
		newPose.Pos().Z() += zOffset;

		auto poseComp = ecm.Component<gz::sim::components::Pose>(model_.Entity());
		if(poseComp){
			poseComp->Data() = newPose;
		}
		else{
			ecm.CreateComponent(model_.Entity(), gz::sim::components::Pose(newPose));
		}
		// Marks the component as changing every tick so other systems'
		// change-tracking (rendering, any pose-publisher/sensor) picks up
		// this write -- a direct Data() mutation above alone isn't
		// guaranteed to be noticed by every downstream system.
		ecm.SetChanged(model_.Entity(), gz::sim::components::Pose::typeId,
		                gz::sim::ComponentState::PeriodicChange);

		if(publishPeriod_ > 0.0 && (t - lastPublishT_) < publishPeriod_){
			return;
		}
		lastPublishT_ = t;
		publishOdometry(newPose, zRate, info);
	}

private:
	void publishOdometry(const gz::math::Pose3d& worldPoseEnu, double zRateEnuUp,
	                      const gz::sim::UpdateInfo& info)
	{
		// This world/plugin assumes an ENU-convention world frame (X-East,
		// Y-North, Z-Up), matching PX4's own gz_bridge and standard ROS
		// practice -- if embedding wave_pad into a DIFFERENT existing
		// world, confirm that world uses the same convention before
		// trusting this ground truth.
		//   NED.x (N) = ENU.y,  NED.y (E) = ENU.x,  NED.z (D) = -ENU.z
		// Heave-only scope: attitude is untouched (identity q, zero angular
		// velocity) regardless of frame convention -- see the class comment
		// for what adding roll/pitch here requires.
		px4_msgs::msg::VehicleOdometry msg;
		auto simUs = std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count();
		msg.timestamp = static_cast<uint64_t>(simUs);
		msg.timestamp_sample = msg.timestamp;
		msg.position[0] = static_cast<float>(worldPoseEnu.Pos().Y());
		msg.position[1] = static_cast<float>(worldPoseEnu.Pos().X());
		msg.position[2] = static_cast<float>(-worldPoseEnu.Pos().Z());
		msg.q = {1.0f, 0.0f, 0.0f, 0.0f};
		msg.velocity[0] = 0.0f;
		msg.velocity[1] = 0.0f;
		msg.velocity[2] = static_cast<float>(-zRateEnuUp);
		msg.angular_velocity = {0.0f, 0.0f, 0.0f};
		odomPub_->publish(msg);
	}

	gz::sim::Model model_;
	bool valid_ = false;
	gz::math::Pose3d referencePose_ = gz::math::Pose3d::Zero;
	std::vector<HeaveComponent> components_;
	std::string odomTopic_;
	double publishRateHz_ = 50.0;
	double publishPeriod_ = 0.02;
	double lastPublishT_ = -1.0;

	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub_;
};

} // namespace pad_motion

GZ_ADD_PLUGIN(pad_motion::PadMotionPlugin,
              gz::sim::System,
              pad_motion::PadMotionPlugin::ISystemConfigure,
              pad_motion::PadMotionPlugin::ISystemPreUpdate)
