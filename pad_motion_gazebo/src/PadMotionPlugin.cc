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
// Full pose + linear/angular velocity, all obtained by finite-differencing
// consecutive PreUpdate samples (position, then orientation) rather than
// reading LinearVelocity/AngularVelocity components, since those are only
// populated by systems explicitly asked to track them (gz-sim's
// EnableComponent) -- differencing works regardless of how (or whether)
// the model's actual motion source cooperates with that, matching "just
// read whatever is really happening" over depending on the mover's
// implementation. NOT field-verified against a known-rate test rig in this
// sandbox (no gz-sim toolchain here) -- before trusting the attitude/
// angular_velocity fields in a real control loop, sanity check by driving
// the model at a known, fixed angular rate about a known axis and
// confirming the sign and axis of msg.angular_velocity/msg.q match
// expectation.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
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
		          << "republishing as GROUND-TRUTH VehicleOdometry (NED/FRD, "
		          << "full pose + linear/angular velocity)." << std::endl;
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
		// Angular velocity of the BODY (FLU) frame, expressed in that same
		// body frame -- i.e. what px4_msgs::VehicleOdometry::angular_velocity
		// needs (after the FLU->FRD axis flip below), not a world-frame rate.
		gz::math::Vector3d angVelFlu = gz::math::Vector3d::Zero;
		if(haveLast_){
			double dt = t - lastT_;
			if(dt > 1e-9){
				velEnu = (poseEnu.Pos() - lastPosEnu_) / dt;

				// Standard small-angle quaternion finite difference: if
				// dq/dt = 0.5 * q (x) (0, w_body) (the usual body-rate
				// kinematic equation, q mapping body->world), then for small
				// dt, q2 ~= q1 (x) (1, 0.5*dt*w_body), so
				// w_body ~= (2/dt) * vec(q1^-1 (x) q2), expressed in q1's
				// (i.e. this tick's PREVIOUS) body frame -- close enough to
				// the current body frame for a single physics-tick dt to
				// treat as "the current body rate" (same order of error as
				// the finite difference itself).
				gz::math::Quaterniond qRel = lastRotEnu_.Inverse() * poseEnu.Rot();
				// Take the short-path representative (positive scalar part)
				// so vec()*2/dt is the small-angle-consistent rate, not one
				// that's aliased by the double cover (q and -q are the same
				// rotation, but only one of them is "small" for small dt).
				if(qRel.W() < 0.0){
					qRel = gz::math::Quaterniond(-qRel.W(), -qRel.X(), -qRel.Y(), -qRel.Z());
				}
				angVelFlu = gz::math::Vector3d(qRel.X(), qRel.Y(), qRel.Z()) * (2.0 / dt);
			}
		}
		lastPosEnu_ = poseEnu.Pos();
		lastRotEnu_ = poseEnu.Rot();
		lastT_ = t;
		haveLast_ = true;

		publishOdometry(poseEnu, velEnu, angVelFlu, info);
	}

private:
	void publishOdometry(const gz::math::Pose3d& poseEnu, const gz::math::Vector3d& velEnu,
	                      const gz::math::Vector3d& angVelFlu, const gz::sim::UpdateInfo& info)
	{
		// This world assumes an ENU-convention world frame (X-East, Y-North,
		// Z-Up) and an FLU body frame (X-Forward, Y-Left, Z-Up, Gazebo's
		// default), matching PX4's own gz_bridge and standard ROS practice --
		// confirm before trusting this ground truth in a differently-
		// configured world.
		//
		// Position/linear velocity: a pure world-frame axis remap (ENU->NED),
		//   NED.x (N) = ENU.y,  NED.y (E) = ENU.x,  NED.z (D) = -ENU.z
		//
		// Orientation/angular velocity additionally need the BODY frame
		// remapped (FLU->FRD), since VehicleOdometry.q/.angular_velocity are
		// body-frame quantities, not world-frame. Both fixed rotations below
		// are the same ones PX4's own ROS<->PX4 bridges use (px4_ros_com's
		// frame_transforms.cpp: NED_ENU_Q, AIRCRAFT_BASELINK_Q) -- reused
		// here rather than re-derived, though independently confirmed by
		// construction: kNedEnuQ is the quaternion for the rotation matrix
		// [[0,1,0],[1,0,0],[0,0,-1]] (a 180 deg rotation about the axis
		// bisecting world X/Y, i.e. (1,1,0)/sqrt(2) -- exactly the matrix the
		// position remap above implements), and kFrdFluQ is the quaternion
		// for diag(1,-1,-1) (a 180 deg rotation about body X). Both are
		// self-inverse (180 deg single-axis rotations), so the same two
		// quaternions convert in either direction.
		//
		// A model's orientation in Gazebo (poseEnu.Rot() = q_enu_flu) maps
		// body(FLU)-frame vectors to world(ENU)-frame vectors. Composing:
		//   v_world_ned = kNedEnuQ * v_world_enu
		//               = kNedEnuQ * (q_enu_flu * v_body_flu)
		//               = kNedEnuQ * q_enu_flu * (kFrdFluQ * v_body_frd)   [kFrdFluQ self-inverse]
		//               = (kNedEnuQ * q_enu_flu * kFrdFluQ) * v_body_frd
		// so q_ned_frd = kNedEnuQ * q_enu_flu * kFrdFluQ.
		static const gz::math::Quaterniond kNedEnuQ(0.0, 0.70710678118654752440, 0.70710678118654752440, 0.0);
		static const gz::math::Quaterniond kFrdFluQ(0.0, 1.0, 0.0, 0.0);

		gz::math::Quaterniond qNedFrd = kNedEnuQ * poseEnu.Rot() * kFrdFluQ;
		// Angular velocity is already body-frame (see PreUpdate's comment) --
		// only needs the FLU->FRD axis flip (X unchanged, Y/Z negated), the
		// same remap kFrdFluQ performs on a vector.
		gz::math::Vector3d angVelFrd(angVelFlu.X(), -angVelFlu.Y(), -angVelFlu.Z());

		px4_msgs::msg::VehicleOdometry msg;
		auto simUs = std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count();
		msg.timestamp = static_cast<uint64_t>(simUs);
		msg.timestamp_sample = msg.timestamp;
		msg.position[0] = static_cast<float>(poseEnu.Pos().Y());
		msg.position[1] = static_cast<float>(poseEnu.Pos().X());
		msg.position[2] = static_cast<float>(-poseEnu.Pos().Z());
		// [w, x, y, z] -- matches this repo's existing VehicleOdometry.q
		// convention (traj_manager.cpp, dummy_publisher.py).
		msg.q = {static_cast<float>(qNedFrd.W()), static_cast<float>(qNedFrd.X()),
		         static_cast<float>(qNedFrd.Y()), static_cast<float>(qNedFrd.Z())};
		msg.velocity[0] = static_cast<float>(velEnu.Y());
		msg.velocity[1] = static_cast<float>(velEnu.X());
		msg.velocity[2] = static_cast<float>(-velEnu.Z());
		msg.angular_velocity[0] = static_cast<float>(angVelFrd.X());
		msg.angular_velocity[1] = static_cast<float>(angVelFrd.Y());
		msg.angular_velocity[2] = static_cast<float>(angVelFrd.Z());
		odomPub_->publish(msg);
	}

	gz::sim::Model model_;
	bool valid_ = false;

	bool haveLast_ = false;
	gz::math::Vector3d lastPosEnu_ = gz::math::Vector3d::Zero;
	gz::math::Quaterniond lastRotEnu_ = gz::math::Quaterniond::Identity;
	double lastT_ = 0.0;

	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub_;
};

} // namespace pad_motion

GZ_ADD_PLUGIN(pad_motion::PadMotionPlugin,
              gz::sim::System,
              pad_motion::PadMotionPlugin::ISystemConfigure,
              pad_motion::PadMotionPlugin::ISystemPreUpdate)
