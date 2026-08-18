// PadMotionPlugin -- a gz-sim Model plugin that reads a model's ACTUAL pose
// from Gazebo every tick and republishes it as px4_msgs/VehicleOdometry
// ground truth, the same message type/convention this repo's drone
// odometry already flows through (traj_manager.cpp, dummy_publisher.py) --
// so any downstream consumer (starting with spa_predictor's
// spa_axis_node/spa_angle_node) needs zero new parsing code.
//
// Deliberately a passive telemetry bridge, not a motion generator: this
// plugin does not drive, script, or otherwise decide how the pad moves --
// whatever does that (physics, a joint controller, a separate driving
// plugin, manual/GUI manipulation, ...) is a wholly separate concern, and
// this plugin has NO motion configuration of its own. Its job is "read
// Gazebo's ground truth for this model, publish it as if a real PX4 device
// reported it" -- the same role PX4's own gz_bridge plays for the vehicle
// itself.
//
// It DOES accept optional, narrowly-scoped SENSOR-NOISE configuration (see
// publishOdometry()) -- a different thing from motion configuration: it
// models a real sensor's imperfection on top of whatever the model is
// actually doing, it doesn't decide or influence that motion at all. All
// noise parameters default to 0 (no noise), reproducing the original
// exact-ground-truth behavior unless explicitly opted into.
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
#include <random>
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

		// Optional synthetic sensor noise -- see publishOdometry()'s comment
		// for the model and why it's applied there, not here. All default to
		// 0.0 (no noise, exact ground truth), so existing worlds that don't
		// set these are completely unaffected.
		//
		// Deliberately uses the single-argument sdf::Element::Get<T>(key)
		// form (returns std::pair<T,bool>: value, wasFound) plus a local
		// helper, rather than the two-argument Get<T>(key, default) form --
		// on at least one real SDFormat release (confirmed via a field build
		// error: "cannot convert 'std::pair<double,bool>' to 'double' in
		// assignment"), the two-argument overload ALSO returns
		// std::pair<T,bool> instead of T directly, breaking a direct
		// assignment. The single-argument pair-returning form is documented
		// consistently, so building the "use default if not found" logic
		// locally sidesteps that overload-return-type inconsistency entirely.
		auto getSdfDouble = [&](const char* key, double defaultValue){
			auto result = sdf->Get<double>(key);
			return result.second ? result.first : defaultValue;
		};
		auto getSdfInt = [&](const char* key, int defaultValue){
			auto result = sdf->Get<int>(key);
			return result.second ? result.first : defaultValue;
		};
		positionNoiseStd_ = getSdfDouble("position_noise_std", 0.0);
		velocityNoiseStd_ = getSdfDouble("velocity_noise_std", 0.0);
		attitudeNoiseStd_ = getSdfDouble("attitude_noise_std", 0.0);
		angularVelocityNoiseStd_ = getSdfDouble("angular_velocity_noise_std", 0.0);
		// >=0: reproducible noise (e.g. comparing two SPA tuning runs
		// apples-to-apples). <0 (default): a real random seed each run.
		int seed = getSdfInt("noise_seed", -1);
		if(seed >= 0){
			rng_.seed(static_cast<unsigned int>(seed));
		}
		else{
			rng_.seed(std::random_device{}());
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
		bool anyNoise = positionNoiseStd_ > 0.0 || velocityNoiseStd_ > 0.0 ||
		                attitudeNoiseStd_ > 0.0 || angularVelocityNoiseStd_ > 0.0;
		std::cout << "[PadMotionPlugin] configured for model '" << modelName
		          << "' -- reading its actual Gazebo pose every tick and "
		          << "republishing as GROUND-TRUTH VehicleOdometry (NED/FRD, "
		          << "full pose + linear/angular velocity)";
		if(anyNoise){
			std::cout << ", with synthetic sensor noise (std: pos=" << positionNoiseStd_
			          << "m, vel=" << velocityNoiseStd_ << "m/s, att=" << attitudeNoiseStd_
			          << "rad, angvel=" << angularVelocityNoiseStd_ << "rad/s)";
		}
		std::cout << "." << std::endl;
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

		gz::math::Vector3d posNed(poseEnu.Pos().Y(), poseEnu.Pos().X(), -poseEnu.Pos().Z());
		gz::math::Vector3d velNed(velEnu.Y(), velEnu.X(), -velEnu.Z());

		// -- optional synthetic sensor noise, applied AFTER the NED/FRD
		// conversion above (equivalent to applying it before, for isotropic
		// per-component Gaussian noise, since ENU<->NED/FLU<->FRD are fixed
		// rotations -- rotating rotationally-symmetric noise leaves its
		// distribution unchanged; applying it here is just less code). Zero-
		// mean, i.i.d. per component/sample; all four std parameters default
		// to 0 (no noise, exact ground truth -- the plugin's original
		// behavior). This lets the SAME plugin also stand in for a
		// realistic, imperfect vision/IMU pad-state measurement (e.g. to
		// test the SPA predictor's robustness/tuning against noise), not
		// just noiseless ground truth. First-order model only: independent
		// white noise per component, no bias/drift/cross-axis correlation --
		// real sensor noise usually isn't this simple; good enough for "does
		// the predictor's smoothing actually help", not a sensor
		// characterization.
		if(positionNoiseStd_ > 0.0){
			std::normal_distribution<double> d(0.0, positionNoiseStd_);
			posNed += gz::math::Vector3d(d(rng_), d(rng_), d(rng_));
		}
		if(velocityNoiseStd_ > 0.0){
			std::normal_distribution<double> d(0.0, velocityNoiseStd_);
			velNed += gz::math::Vector3d(d(rng_), d(rng_), d(rng_));
		}
		if(attitudeNoiseStd_ > 0.0){
			// Small-angle perturbation COMPOSED in the body frame
			// (q_noisy = q_true * dq), NOT additive noise on raw quaternion
			// components -- that would break the unit-norm constraint and
			// produce an invalid rotation. dq is the standard small-angle
			// quaternion for a random body-frame rotation vector
			// delta ~ N(0, attitudeNoiseStd_^2 I):
			// dq = normalize(1, delta.x/2, delta.y/2, delta.z/2).
			std::normal_distribution<double> d(0.0, attitudeNoiseStd_);
			gz::math::Quaterniond dq(1.0, 0.5 * d(rng_), 0.5 * d(rng_), 0.5 * d(rng_));
			dq.Normalize();
			qNedFrd = qNedFrd * dq;
		}
		if(angularVelocityNoiseStd_ > 0.0){
			std::normal_distribution<double> d(0.0, angularVelocityNoiseStd_);
			angVelFrd += gz::math::Vector3d(d(rng_), d(rng_), d(rng_));
		}

		px4_msgs::msg::VehicleOdometry msg;
		auto simUs = std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count();
		msg.timestamp = static_cast<uint64_t>(simUs);
		msg.timestamp_sample = msg.timestamp;
		msg.position[0] = static_cast<float>(posNed.X());
		msg.position[1] = static_cast<float>(posNed.Y());
		msg.position[2] = static_cast<float>(posNed.Z());
		// [w, x, y, z] -- matches this repo's existing VehicleOdometry.q
		// convention (traj_manager.cpp, dummy_publisher.py).
		msg.q = {static_cast<float>(qNedFrd.W()), static_cast<float>(qNedFrd.X()),
		         static_cast<float>(qNedFrd.Y()), static_cast<float>(qNedFrd.Z())};
		msg.velocity[0] = static_cast<float>(velNed.X());
		msg.velocity[1] = static_cast<float>(velNed.Y());
		msg.velocity[2] = static_cast<float>(velNed.Z());
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

	// Optional synthetic sensor noise -- see publishOdometry()'s comment.
	// All 0.0 by default (no noise).
	double positionNoiseStd_ = 0.0;         // m, per axis
	double velocityNoiseStd_ = 0.0;         // m/s, per axis
	double attitudeNoiseStd_ = 0.0;         // rad, small-angle body-frame perturbation
	double angularVelocityNoiseStd_ = 0.0;  // rad/s, per axis
	std::mt19937 rng_;

	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub_;
};

} // namespace pad_motion

GZ_ADD_PLUGIN(pad_motion::PadMotionPlugin,
              gz::sim::System,
              pad_motion::PadMotionPlugin::ISystemConfigure,
              pad_motion::PadMotionPlugin::ISystemPreUpdate)
