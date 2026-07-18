#include <ros_traj_gen_utils/geometric_controller.h>
#include <cmath>

namespace geometric_control {

namespace {
// Guards normalize()-type divisions against near-zero vectors (e.g. a
// commanded thrust near free-fall, or a yaw reference that lines up with
// straight-up/-down).
constexpr double kMinThrustNorm = 1e-3;
constexpr double kMinVecNorm = 1e-6;
}

Eigen::Matrix3d hatMap(const Eigen::Vector3d& v){
    Eigen::Matrix3d m;
    m <<      0, -v.z(),  v.y(),
          v.z(),      0, -v.x(),
         -v.y(),  v.x(),      0;
    return m;
}

Eigen::Vector3d veeMap(const Eigen::Matrix3d& m){
    return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
}

Eigen::Vector3d normalizedDerivative(const Eigen::Vector3d& v, const Eigen::Vector3d& v_dot){
    double n = v.norm();
    if(n < kMinVecNorm){
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d u = v / n;
    return (v_dot - u * u.dot(v_dot)) / n;
}

GeometricController::GeometricController(double mass, double gravity, const Eigen::Vector3d& kR)
    : mass_(mass), gravity_(gravity), kR_(kR) {}

RateThrustCommand GeometricController::compute(const FlatState& flat, const VehicleState& state,
                                                 const PositionGains& gains) const {
    (void)mass_; // thrust is expressed as a specific force (per unit mass); mass
                 // conversion to Newtons/normalized-throttle is the node's job.

    // World frame is NED: gravity's world-frame vector is (0, 0, +gravity).
    const Eigen::Vector3d g_world(0.0, 0.0, gravity_);

    // Desired specific force (thrust force / mass) needed to null the
    // position/velocity error while feedforward-tracking the trajectory's
    // own acceleration, plus gravity compensation.
    Eigen::Vector3d e_p = state.position - flat.position;
    Eigen::Vector3d e_v = state.velocity - flat.velocity;
    Eigen::Vector3d a_cmd = flat.acceleration
                           - gains.kx.cwiseProduct(e_p)
                           - gains.kv.cwiseProduct(e_v)
                           - g_world;

    double a_cmd_norm = a_cmd.norm();
    // Desired body Z axis (FRD: points "down" through the vehicle -- thrust
    // acts along -b3, so b3 itself points opposite the desired thrust force).
    Eigen::Vector3d b3_des;
    if(a_cmd_norm < kMinThrustNorm){
        // Degenerate (near free-fall): hold the current attitude's Z axis
        // rather than normalizing a ~zero vector.
        b3_des = state.orientation.toRotationMatrix().col(2);
        a_cmd_norm = kMinThrustNorm;
    } else {
        b3_des = -a_cmd / a_cmd_norm;
    }

    // Desired heading direction in the world horizontal (N-E) plane.
    Eigen::Vector3d c1(std::cos(flat.yaw), std::sin(flat.yaw), 0.0);
    Eigen::Vector3d b3_cross_c1 = b3_des.cross(c1);
    double b2_norm = b3_cross_c1.norm();
    Eigen::Vector3d b2_des = (b2_norm > kMinVecNorm)
        ? Eigen::Vector3d(b3_cross_c1 / b2_norm)
        // c1 ~ parallel to b3_des (yaw reference lines up with straight
        // up/down, degenerate): fall back to the current body Y axis
        // rather than dividing by ~0.
        : Eigen::Vector3d(state.orientation.toRotationMatrix().col(1));
    Eigen::Vector3d b1_des = b2_des.cross(b3_des);

    Eigen::Matrix3d R_des;
    R_des.col(0) = b1_des;
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;

    // Rate feedforward: differentiate the R_des construction above using
    // the trajectory's jerk (and yaw_dot), approximating a_cmd's own time
    // derivative by the feedforward jerk alone -- the standard practical
    // simplification (differentiating the PD correction terms too would
    // need acceleration error, which is noisy and not worth it in practice).
    Eigen::Vector3d a_cmd_dot = flat.jerk;
    Eigen::Vector3d b3_dot = normalizedDerivative(-a_cmd, -a_cmd_dot);
    Eigen::Vector3d c1_dot = flat.yaw_dot *
        Eigen::Vector3d(-std::sin(flat.yaw), std::cos(flat.yaw), 0.0);
    Eigen::Vector3d b3_cross_c1_dot = b3_dot.cross(c1) + b3_des.cross(c1_dot);
    Eigen::Vector3d b2_dot = normalizedDerivative(b3_cross_c1, b3_cross_c1_dot);
    Eigen::Vector3d b1_dot = b2_dot.cross(b3_des) + b2_des.cross(b3_dot);

    Eigen::Matrix3d R_des_dot;
    R_des_dot.col(0) = b1_dot;
    R_des_dot.col(1) = b2_dot;
    R_des_dot.col(2) = b3_dot;

    Eigen::Matrix3d omega_hat = R_des.transpose() * R_des_dot;
    // Enforce skew-symmetry (the feedforward-jerk approximation above means
    // this isn't exact) before extracting the rate vector.
    Eigen::Vector3d omega_des = veeMap(0.5 * (omega_hat - omega_hat.transpose()));

    // Attitude-error correction, added as a rate correction on top of the
    // feedforward rate above -- see the header comment on kR_ for why there
    // is no separate rate-error (kOm) term here.
    Eigen::Matrix3d R = state.orientation.toRotationMatrix();
    Eigen::Vector3d e_R = 0.5 * veeMap(R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d omega_cmd = omega_des - kR_.cwiseProduct(e_R);

    RateThrustCommand cmd;
    cmd.thrust_specific = a_cmd_norm;
    cmd.body_rate = omega_cmd;
    cmd.desired_orientation = Eigen::Quaterniond(R_des);
    return cmd;
}

} // namespace geometric_control
