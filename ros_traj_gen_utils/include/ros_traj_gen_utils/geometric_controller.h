#ifndef _geometric_controller_h
#define _geometric_controller_h
#include <Eigen/Eigen>

// Differential-flatness-based geometric trajectory-tracking controller
// (Lee/Leok/McClamroch, "Geometric Tracking Control of a Quadrotor UAV on
// SE(3)", combined with the Mellinger/Kumar rate-feedforward-from-jerk
// construction), adapted to this repo's world=NED / body=FRD convention.
//
// Converts a flat-output trajectory sample plus the vehicle's actual state
// directly into a body-rate + thrust command -- no separate attitude- or
// rate-loop stage of our own; PX4's own inner rate controller is the only
// control loop left downstream of this one.
//
// Pure Eigen math, no ROS types, so it can be unit-tested on its own.
namespace geometric_control {

// One instant of a differentially-flat trajectory (world/NED frame).
struct FlatState {
    Eigen::Vector3d position     = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d jerk         = Eigen::Vector3d::Zero();
    double yaw     = 0.0;
    double yaw_dot = 0.0;
};

// The vehicle's actual current state: world/NED position+velocity,
// body-FRD-in-world-NED attitude. From odometry/the estimator.
struct VehicleState {
    Eigen::Vector3d position       = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity       = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

// Position/velocity error gains -- these are exactly the kx/kv this repo's
// quadrotor_msgs::msg::PositionCommand already carries per trajectory
// point (previously computed but never consumed by anything downstream).
struct PositionGains {
    Eigen::Vector3d kx = Eigen::Vector3d(7.4, 7.4, 7.4);
    Eigen::Vector3d kv = Eigen::Vector3d(4.8, 4.8, 4.8);
};

// What gets sent to PX4 (VehicleRatesSetpoint-equivalent).
struct RateThrustCommand {
    double thrust_specific = 0.0;     // commanded |thrust force| / mass, m/s^2
    Eigen::Vector3d body_rate = Eigen::Vector3d::Zero(); // commanded body rates, rad/s (FRD)
    Eigen::Quaterniond desired_orientation = Eigen::Quaterniond::Identity(); // logging/diagnostics only
};

class GeometricController {
public:
    // gravity: m/s^2, positive -- this repo's world frame is NED, so
    // gravity's world-frame vector is (0, 0, +gravity).
    //
    // kR: attitude-error-to-rate-correction gain (1/s), per body axis. This
    // is the only feedback gain that belongs to the controller itself
    // rather than to the trajectory: it's a vehicle-tuning constant, not
    // something that should vary per waypoint the way kx/kv can.
    //
    // Deliberately no rate-error (kOm) term anywhere in this class: since
    // the output is a RATE command (not torque), closing a rate-error loop
    // here would just fight PX4's own inner rate controller -- that's
    // exactly the loop responsible for tracking whatever rate we ask for.
    GeometricController(double mass, double gravity, const Eigen::Vector3d& kR);

    RateThrustCommand compute(const FlatState& flat, const VehicleState& state,
                               const PositionGains& gains) const;

private:
    double mass_;
    double gravity_;
    Eigen::Vector3d kR_;
};

// -- exposed for unit testing --
Eigen::Matrix3d hatMap(const Eigen::Vector3d& v);
Eigen::Vector3d veeMap(const Eigen::Matrix3d& m);
// Derivative of a normalized vector u = v/|v|, given v and v_dot.
Eigen::Vector3d normalizedDerivative(const Eigen::Vector3d& v, const Eigen::Vector3d& v_dot);

} // namespace geometric_control

#endif
