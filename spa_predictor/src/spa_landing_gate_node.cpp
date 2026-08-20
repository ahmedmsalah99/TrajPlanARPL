// spa_landing_gate_node -- a Go/NoGo gate for perch landing, built from the
// SPA predictors' own forward-looking predictions (spa_axis_node's x/y/
// heave, spa_angle_node's roll/pitch) rather than the current instant.
// Publishes spa_predictor/LandingGate at horizon_s (default 0.5s) ahead:
//
//   1. Velocity condition: predicted x/y/heave velocity all within
//      +/-velocity_threshold.
//   2. Inclination condition: the pad's predicted surface normal (from
//      PREDICTED roll/pitch + the pad's most recently MEASURED yaw --
//      yaw isn't predicted, out of SPA's current scope) is tilted no more
//      than a configured max angle off level (inclination_cos >=
//      min_inclination_cos).
//   3. Direction condition: the pad's HEADING (yaw only -- roll/pitch are
//      already covered by #2) puts the drone somewhere in its REAR HALF
//      (astern, or behind-left/behind-right out to abeam -- direction_cos
//      <= max_direction_cos_left or max_direction_cos_right, whichever
//      side the drone is actually on, per side_cos), favoring an approach
//      from the pad's stern. Independently configurable per side (e.g. a
//      vessel only boardable from its port quarter). A pad with no
//      meaningful net horizontal motion (below static_speed_threshold_mps
//      -- anchored/hovering, only bobbing with the waves) is treated as
//      STATIC, where this is trivially true -- its yaw has no coherent
//      "forward" to measure a stern arc from.
//
// This does NOT replace TrajBase::calcPerchCond()'s own precise terminal-
// condition rejection test -- it's a coarser, EARLIER sanity filter using
// SPA's predictions, meant to answer "does the near-future look landable"
// before actually committing a trajectory, not to re-derive
// calcPerchCond()'s exact physics.
//
// Needs the DRONE's own odometry (NOT published by anything in this
// package -- the only other consumer, spa_axis_node/spa_angle_node, only
// ever needs the PAD's state) to compute the pad's horizontal offset from
// it. Default topic (/fmu/out/vehicle_odometry) matches
// ros_traj_gen_utils/traj_manager.cpp's own convention exactly (verified
// against that file, not guessed).
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <spa_predictor/msg/spa_prediction.hpp>
#include <spa_predictor/msg/landing_gate.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
// Quaternion (w,x,y,z) -> (roll, pitch, yaw), aerospace ZYX / NED-FRD
// convention -- matches spa_angle_node.cpp's quatToRollPitch() (roll/pitch
// parts identical) and traj_gen/traj_utils/quaternion.h's ToEulerAngles(),
// extended here to also return yaw (spa_angle_node.cpp doesn't need it,
// this node does -- to reconstruct a full rotation from PREDICTED
// roll/pitch + the pad's own MEASURED yaw). Reimplemented locally rather
// than depending on traj_gen so spa_predictor stays standalone (see this
// package's description).
void quatToRollPitchYaw(double w, double x, double y, double z,
                        double* roll, double* pitch, double* yaw)
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

	double siny_cosp = 2.0 * (w * z + x * y);
	double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
	*yaw = std::atan2(siny_cosp, cosy_cosp);
}

// (roll, pitch, yaw) -> quaternion (w,x,y,z), aerospace ZYX convention --
// the exact inverse of quatToRollPitchYaw() above, matching
// traj_gen/traj_utils/quaternion.h's ToQuaternion(yaw,pitch,roll) formula
// (verified against it, reimplemented locally for the same standalone-
// package reason as quatToRollPitchYaw()). Used to compose PREDICTED
// roll/pitch with the pad's MEASURED yaw back into a single rotation, from
// which the surface normal is read off exactly as
// TrajBase::calcPerchCond() reads it from a directly-measured quaternion
// (same 3rd-column formula) -- this reuses calcPerchCond()'s own
// established s3 convention rather than re-deriving what "upward" means
// from scratch, so this gate can't silently disagree with it about sign
// conventions baked into how the pad's SDF model itself was authored.
void rpyToQuat(double roll, double pitch, double yaw, double* w, double* x, double* y, double* z)
{
	double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
	double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
	double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
	*w = cr * cp * cy + sr * sp * sy;
	*x = sr * cp * cy - cr * sp * sy;
	*y = cr * sp * cy + sr * cp * sy;
	*z = cr * cp * sy - sr * sp * cy;
}

// TrajBase::calcPerchCond() reads its s3 as the OUTWARD surface normal --
// the direction pointing away from the pad, into the air the drone
// approaches through -- straight off H's 3rd column, where H is a target
// frame already built so that convention holds (see its own s3 comment:
// "Surface normal s3 (outward)"). The quaternion built here, by contrast,
// is the pad's own raw FRD body attitude (predicted roll/pitch + measured
// yaw, same convention as VehicleOdometry.q) -- and in FRD, body Z points
// DOWN, so at zero tilt the raw 3rd column is (0,0,1): straight into the
// pad, not away from it. Negating it recovers the true outward normal
// (zero tilt -> (0,0,-1), i.e. world-up, matching a flat/level pad facing
// upward) before handing it to the same s3.dot(e3) TrajBase::calcPerchCond()
// itself uses. Skipping this negation was the bug behind PR #131's upward
// check reading false for an essentially-flat pad -- it was comparing the
// INWARD normal against world-up and always getting a near-(-1) result.
void quatToS3(double w, double x, double y, double z, double* sx, double* sy, double* sz)
{
	*sx = -(2.0 * (x * z + w * y));
	*sy = -(2.0 * (y * z - w * x));
	*sz = -(1.0 - 2.0 * (x * x + y * y));
}

// Finds the index in horizons whose value is within tol of target.
// Returns false (idx_out untouched) if none matches -- SpaPrediction's
// horizon_s[] is whatever the publishing node's own horizons_s parameter
// contains, not guaranteed to include any particular value.
bool findHorizonIndex(const std::vector<double>& horizons, double target, double tol, size_t* idx_out)
{
	for(size_t i = 0; i < horizons.size(); i++){
		if(std::fabs(horizons[i] - target) < tol){
			*idx_out = i;
			return true;
		}
	}
	return false;
}
} // namespace

class SpaLandingGateNode : public rclcpp::Node
{
public:
	SpaLandingGateNode() : Node("spa_landing_gate_node")
	{
		declare_parameter("x_topic", std::string("/pad/spa/x_prediction"));
		declare_parameter("y_topic", std::string("/pad/spa/y_prediction"));
		declare_parameter("heave_topic", std::string("/pad/spa/heave_prediction"));
		declare_parameter("roll_topic", std::string("/pad/spa/roll_prediction"));
		declare_parameter("pitch_topic", std::string("/pad/spa/pitch_prediction"));
		declare_parameter("pad_odom_topic", std::string("/pad/fmu/out/vehicle_odometry"));
		declare_parameter("drone_odom_topic", std::string("/fmu/out/vehicle_odometry"));
		declare_parameter("output_topic", std::string("/pad/landing_gate"));
		declare_parameter("publish_rate_hz", 10.0);

		// The horizon every predicted_* quantity is evaluated at. Must match
		// (within horizon_tol_s) an entry already present in each
		// SpaPrediction's own horizon_s[] -- this node does not request its
		// own horizons from the predictors, it reads whatever they already
		// publish (default horizons_s includes 0.5 -- see spa_axis_node.cpp/
		// spa_angle_node.cpp's own default).
		declare_parameter("horizon_s", 0.5);
		declare_parameter("horizon_tol_s", 0.01);

		// |vx|,|vy|,|vz| (predicted, at horizon_s) all <= this to pass the
		// velocity condition. NOT calibrated to any particular vehicle/pad
		// combination -- tune to what your controller can actually track
		// through at touchdown.
		declare_parameter("velocity_threshold", 0.3);

		// s3 . world_up >= this to pass the inclination condition -- i.e. the
		// pad's tilt off dead-level, expressed as a cosine, must not exceed
		// the angle this represents. Default cos(30 deg) ~= 0.866 -- a
		// broad, coarse sanity bound, DELIBERATELY looser than
		// calcPerchCond()'s own precise, azimuth-dependent rejection test
		// (~19-25 deg ceiling, see the horiz_accel_limit budget discussion)
		// -- this gate isn't meant to replace that, only to reject
		// predictions that are obviously not landable before a trajectory
		// is even attempted.
		declare_parameter("min_inclination_cos", std::cos(30.0 * M_PI / 180.0));

		// direction_cos <= this (side_cos < 0, i.e. the drone is on the
		// pad's LEFT/port side) or <= max_direction_cos_right (side_cos >=
		// 0, RIGHT/starboard side) to pass the direction condition -- see
		// side_cos below for which side is which. Each is independently
		// tunable so one side of the approach can be favored/excluded
		// without affecting the other (e.g. a vessel that can only be
		// boarded from its port quarter). direction_cos is the angle
		// between the pad's HEADING (yaw -- roll/pitch are already covered
		// by the inclination condition above, this is yaw only) and the
		// drone's bearing FROM the pad, expressed as a cosine: 0 deg off
		// dead-ahead (bow) = +1.0, dead astern = -1.0. Both default to 0.0
		// (90 deg off the bow): the entire REAR HALF on each side counts
		// as eligible -- astern, or anywhere behind-left/behind-right up
		// to (and including) abeam -- not just dead astern. Lower (toward
		// -1.0) = stricter on that side, narrowing toward dead-astern-only;
		// higher (toward +1.0) = looser, admitting part of the forward
		// half on that side too.
		declare_parameter("max_direction_cos_left", 0.0);
		declare_parameter("max_direction_cos_right", 0.0);

		// Below this pad horizontal speed (m/s, predicted at horizon_s --
		// the same vx/vy the velocity condition above already computes),
		// the pad is considered STATIC: not actually underway in any
		// particular direction (e.g. an anchored/moored vessel, or this
		// package's own sim rig, whose wave plugin heaves/rolls/pitches/
		// yaws in place with no surge/sway translation at all). A static
		// pad's yaw is just sloshing back and forth with the waves, not
		// progressing anywhere -- there's no meaningful "bow" to define a
		// stern-side approach arc from, so the direction condition is
		// trivially satisfied rather than computed. Skipping this was the
		// failure mode behind the earlier (now-replaced) s3-lean-based
		// version of this check flipping direction_ok in and out for a
		// pad that was never actually going anywhere.
		declare_parameter("static_speed_threshold_mps", 0.05);

		// Below this horizontal magnitude (m), treat direction_ok as
		// trivially satisfied instead of computing a direction at all --
		// normalizing a near-zero vector is meaningless/numerically
		// unstable, and there's no real bearing to speak of when the
		// drone is nearly directly over/under the pad.
		declare_parameter("direction_epsilon_m", 0.05);

		horizonS_ = get_parameter("horizon_s").as_double();
		horizonTolS_ = get_parameter("horizon_tol_s").as_double();
		velocityThreshold_ = get_parameter("velocity_threshold").as_double();
		minInclinationCos_ = get_parameter("min_inclination_cos").as_double();
		maxDirectionCosLeft_ = get_parameter("max_direction_cos_left").as_double();
		maxDirectionCosRight_ = get_parameter("max_direction_cos_right").as_double();
		staticSpeedThresholdMps_ = get_parameter("static_speed_threshold_mps").as_double();
		directionEpsilonM_ = get_parameter("direction_epsilon_m").as_double();

		rclcpp::QoS px4_qos(1);
		px4_qos.best_effort();
		px4_qos.durability_volatile();

		std::string padOdomTopic = get_parameter("pad_odom_topic").as_string();
		std::string droneOdomTopic = get_parameter("drone_odom_topic").as_string();
		padOdomSub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
			padOdomTopic, px4_qos, std::bind(&SpaLandingGateNode::onPadOdom, this, std::placeholders::_1));
		droneOdomSub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
			droneOdomTopic, px4_qos, std::bind(&SpaLandingGateNode::onDroneOdom, this, std::placeholders::_1));

		// SpaPrediction publishers use the default (reliable, depth-10) QoS
		// (spa_axis_node.cpp/spa_angle_node.cpp: create_publisher<...>(topic,
		// 10)) -- match it here, not best-effort.
		xSub_ = create_subscription<spa_predictor::msg::SpaPrediction>(
			get_parameter("x_topic").as_string(), 10,
			[this](const spa_predictor::msg::SpaPrediction::SharedPtr msg){ xMsg_ = msg; });
		ySub_ = create_subscription<spa_predictor::msg::SpaPrediction>(
			get_parameter("y_topic").as_string(), 10,
			[this](const spa_predictor::msg::SpaPrediction::SharedPtr msg){ yMsg_ = msg; });
		heaveSub_ = create_subscription<spa_predictor::msg::SpaPrediction>(
			get_parameter("heave_topic").as_string(), 10,
			[this](const spa_predictor::msg::SpaPrediction::SharedPtr msg){ heaveMsg_ = msg; });
		rollSub_ = create_subscription<spa_predictor::msg::SpaPrediction>(
			get_parameter("roll_topic").as_string(), 10,
			[this](const spa_predictor::msg::SpaPrediction::SharedPtr msg){ rollMsg_ = msg; });
		pitchSub_ = create_subscription<spa_predictor::msg::SpaPrediction>(
			get_parameter("pitch_topic").as_string(), 10,
			[this](const spa_predictor::msg::SpaPrediction::SharedPtr msg){ pitchMsg_ = msg; });

		pub_ = create_publisher<spa_predictor::msg::LandingGate>(get_parameter("output_topic").as_string(), 10);

		double rate = get_parameter("publish_rate_hz").as_double();
		timer_ = create_wall_timer(
			std::chrono::duration<double>(1.0 / rate),
			std::bind(&SpaLandingGateNode::onTimer, this));

		RCLCPP_INFO(get_logger(),
			"spa_landing_gate_node up: horizon_s=%.2f, velocity_threshold=%.2f, "
			"min_inclination_cos=%.3f, max_direction_cos_left=%.3f, max_direction_cos_right=%.3f, "
			"static_speed_threshold_mps=%.3f -- pad odom %s, drone odom %s -> %s @ %.1f Hz",
			horizonS_, velocityThreshold_, minInclinationCos_, maxDirectionCosLeft_, maxDirectionCosRight_,
			staticSpeedThresholdMps_, padOdomTopic.c_str(), droneOdomTopic.c_str(),
			get_parameter("output_topic").as_string().c_str(), rate);
	}

private:
	void onPadOdom(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) { padOdomMsg_ = msg; }
	void onDroneOdom(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) { droneOdomMsg_ = msg; }

	// Looks up `field`[idx of horizonS_] in one SpaPrediction message.
	// Returns false (value_out untouched) if the message is missing,
	// uninitialized, or has no matching horizon entry.
	static bool lookupAt(const spa_predictor::msg::SpaPrediction::SharedPtr& msg,
	                      const std::vector<double>& field, double horizonS, double tol, double* value_out)
	{
		if(!msg || !msg->initialized){
			return false;
		}
		size_t idx;
		if(!findHorizonIndex(msg->horizon_s, horizonS, tol, &idx) || idx >= field.size()){
			return false;
		}
		*value_out = field[idx];
		return true;
	}

	void onTimer()
	{
		spa_predictor::msg::LandingGate out;
		out.header.stamp = now();
		out.header.frame_id = "odom"; // NED, matches this repo's odom_frame convention
		out.horizon_s = horizonS_;

		bool ready = true;
		ready &= lookupAt(xMsg_, xMsg_ ? xMsg_->predicted_velocity : std::vector<double>{}, horizonS_, horizonTolS_, &out.vx);
		ready &= lookupAt(yMsg_, yMsg_ ? yMsg_->predicted_velocity : std::vector<double>{}, horizonS_, horizonTolS_, &out.vy);
		ready &= lookupAt(heaveMsg_, heaveMsg_ ? heaveMsg_->predicted_velocity : std::vector<double>{}, horizonS_, horizonTolS_, &out.vz);

		double predRoll = 0.0, predPitch = 0.0;
		ready &= lookupAt(rollMsg_, rollMsg_ ? rollMsg_->predicted_value : std::vector<double>{}, horizonS_, horizonTolS_, &predRoll);
		ready &= lookupAt(pitchMsg_, pitchMsg_ ? pitchMsg_->predicted_value : std::vector<double>{}, horizonS_, horizonTolS_, &predPitch);

		double padX = 0.0, padY = 0.0;
		ready &= lookupAt(xMsg_, xMsg_ ? xMsg_->predicted_value : std::vector<double>{}, horizonS_, horizonTolS_, &padX);
		ready &= lookupAt(yMsg_, yMsg_ ? yMsg_->predicted_value : std::vector<double>{}, horizonS_, horizonTolS_, &padY);

		ready &= (padOdomMsg_ != nullptr);
		ready &= (droneOdomMsg_ != nullptr);

		out.inputs_ready = ready;
		if(!ready){
			out.go = false;
			pub_->publish(out);
			return;
		}

		out.velocity_threshold = velocityThreshold_;
		out.velocity_ok = std::fabs(out.vx) <= velocityThreshold_ &&
		                   std::fabs(out.vy) <= velocityThreshold_ &&
		                   std::fabs(out.vz) <= velocityThreshold_;

		// Pad's own MEASURED yaw (not predicted -- see class comment).
		double measRoll, measPitch, measYaw;
		quatToRollPitchYaw(padOdomMsg_->q[0], padOdomMsg_->q[1], padOdomMsg_->q[2], padOdomMsg_->q[3],
		                   &measRoll, &measPitch, &measYaw);

		double qw, qx, qy, qz;
		rpyToQuat(predRoll, predPitch, measYaw, &qw, &qx, &qy, &qz);
		quatToS3(qw, qx, qy, qz, &out.s3_x, &out.s3_y, &out.s3_z);

		// world_up = (0,0,-1) in NED -- matches TrajBase::calcPerchCond()'s
		// e3 exactly. inclination_cos = cos(angle between s3 and world_up),
		// i.e. how far off dead-level the pad's surface is tilted.
		out.inclination_cos = -out.s3_z;
		out.inclination_ok = out.inclination_cos >= minInclinationCos_;

		// -- Direction condition: the pad's HEADING (yaw) vs. the drone's
		// bearing FROM the pad -- NOT the tilt/roll-pitch lean (that's
		// already covered by the inclination condition above). Favors an
		// approach from the pad's stern: eligible whenever the drone sits
		// anywhere in the pad's rear half relative to its own heading
		// (astern, or behind-left/behind-right out to abeam), not only
		// dead astern.
		double dx = padX - static_cast<double>(droneOdomMsg_->position[0]);
		double dy = padY - static_cast<double>(droneOdomMsg_->position[1]);
		double dNorm = std::hypot(dx, dy);
		double padSpeed = std::hypot(out.vx, out.vy);
		if(padSpeed < staticSpeedThresholdMps_ || dNorm < directionEpsilonM_){
			// Static pad (no coherent heading to define a stern from) or
			// drone too close horizontally to have a meaningful bearing --
			// see the declare_parameter comments above. Trivially
			// satisfied, not a failure.
			out.direction_cos = std::numeric_limits<double>::quiet_NaN();
			out.side_cos = std::numeric_limits<double>::quiet_NaN();
			out.direction_ok = true;
		}
		else{
			// Pad's bow direction in world NED (North, East): yaw=0 -> bow
			// points North, positive yaw rotates the bow toward East --
			// verified against quatToRollPitchYaw()/rpyToQuat()'s own
			// quaternion<->rotation-matrix convention at roll=pitch=0
			// (first column of R reduces to exactly (cos(yaw),sin(yaw),0)
			// there) rather than assumed, so this can't silently disagree
			// with them about which way yaw turns.
			double headingX = std::cos(measYaw);
			double headingY = std::sin(measYaw);
			// "Right" (starboard) of the bow: the SAME body Y axis (FRD --
			// this file's convention throughout, e.g. VehicleOdometry.q)
			// expressed in world frame, again verified rather than assumed
			// (2nd column of R reduces to exactly (-sin(yaw),cos(yaw),0) at
			// roll=pitch=0) -- 90 deg clockwise from the bow when viewed
			// from above.
			double rightX = -std::sin(measYaw);
			double rightY = std::cos(measYaw);
			// Bearing FROM the pad TO the drone -- the other way round
			// from dx,dy above, which is pad-minus-drone.
			double bearingX = -dx;
			double bearingY = -dy;
			out.direction_cos = (headingX * bearingX + headingY * bearingY) / dNorm;
			// side_cos > 0 -> drone is on the pad's RIGHT/starboard side;
			// < 0 -> LEFT/port side. Picks which of the two independently
			// configured thresholds applies this cycle.
			out.side_cos = (rightX * bearingX + rightY * bearingY) / dNorm;
			double maxDirectionCos = (out.side_cos >= 0.0) ? maxDirectionCosRight_ : maxDirectionCosLeft_;
			out.direction_ok = out.direction_cos <= maxDirectionCos;
		}

		out.go = out.velocity_ok && out.inclination_ok && out.direction_ok;
		pub_->publish(out);
	}

	double horizonS_ = 0.5;
	double horizonTolS_ = 0.01;
	double velocityThreshold_ = 0.3;
	double minInclinationCos_ = 0.866;
	double maxDirectionCosLeft_ = 0.0;
	double maxDirectionCosRight_ = 0.0;
	double staticSpeedThresholdMps_ = 0.05;
	double directionEpsilonM_ = 0.05;

	spa_predictor::msg::SpaPrediction::SharedPtr xMsg_, yMsg_, heaveMsg_, rollMsg_, pitchMsg_;
	px4_msgs::msg::VehicleOdometry::SharedPtr padOdomMsg_, droneOdomMsg_;

	rclcpp::Subscription<spa_predictor::msg::SpaPrediction>::SharedPtr xSub_, ySub_, heaveSub_, rollSub_, pitchSub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr padOdomSub_, droneOdomSub_;
	rclcpp::Publisher<spa_predictor::msg::LandingGate>::SharedPtr pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpaLandingGateNode>());
	rclcpp::shutdown();
	return 0;
}
