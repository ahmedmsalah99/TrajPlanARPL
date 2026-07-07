#include <traj_gen/trajectory/FOV.h>
#include <math.h>
using namespace std;

FOV_constraint::FOV_constraint(Eigen::Vector4d pos, Eigen::Vector3d acc, double tilt){
	init_pos = pos;
	init_acc = acc;
	camTilt = tilt;
}

// Decompose the target direction relative to the camera optical axis, for a given
// position/acceleration/yaw. The optical axis is the thrust axis b3 rotated about
// the body y-axis b2 by (pi/2 + camTilt) (Rodrigues' formula); camTilt is the
// camera mount tilt. Returns the on-axis distance ('right') and off-axis distance
// ('left'); the FOV margin used downstream is (r_h*right - left), eq.(7). yaw is
// an explicit parameter (not read from init_pos) so derivative_FOV can perturb it
// independently when building the linearization's yaw partial.
fov_zero_order FOV_constraint::evalState(const Eigen::Vector3d& target,
                                         const Eigen::Vector3d& pos,
                                         const Eigen::Vector3d& acc,
                                         double yaw){
	fov_zero_order eval;
	const double pi = 3.14159265358979323846;
	Eigen::Vector3d B2(-sin(yaw), cos(yaw), 0.0);              // body y-axis (see note: handedness/mount = Part 2)
	Eigen::Vector3d nd = target - pos;                         // quad -> target
	// thrust axis b3 = normalize(acc + g*e3); in NED (z-down) e3=(0,0,-1) so this is acc.z - g
	Eigen::Vector3d B3 = acc; B3[2] -= g; B3 = B3.normalized();
	double th = pi/2.0 + camTilt;                              // optical axis angle from b3 about b2
	Eigen::Vector3d n_proj = B3*cos(th)
	                       + B2.cross(B3)*sin(th)
	                       + B2*(B2.dot(B3))*(1.0 - cos(th));  // Rodrigues rotation of B3 about B2
	double proj = nd.dot(n_proj);
	Eigen::Vector3d on_axis = n_proj*proj;
	eval.left  = (nd - on_axis).norm();                        // off-axis (perpendicular) distance
	eval.right = on_axis.norm();                               // on-axis distance
	return eval;
}

fov_zero_order FOV_constraint::fov_eval(Eigen::Vector3d target){
	Eigen::Vector3d pos(init_pos[0], init_pos[1], init_pos[2]);
	return evalState(target, pos, init_acc, init_pos[3]);
}

// First-order linearization of the FOV cone margin diff = r_h*right - left, eq.(7)-(8)
// of the paper, with respect to the state [x, y, z, ax, ay, az, yaw] -- 7 variables,
// computed by central finite differences (this replaces the previous CAS-generated
// analytic Jacobian, so it stays correct for any camera tilt, and now also covers
// yaw so the linearized constraint can actually be satisfied by adjusting heading,
// not just position/acceleration).
fov_constr FOV_constraint::derivative_FOV(Eigen::Vector3d target){
	fov_constr sol;
	Eigen::Vector3d pos(init_pos[0], init_pos[1], init_pos[2]);
	Eigen::Vector3d acc = init_acc;
	double yaw = init_pos[3];

	fov_zero_order e0 = evalState(target, pos, acc, yaw);
	double diff0 = r_h*e0.right - e0.left;                     // margin at the current state, diff(x0)

	Eigen::MatrixXd x0(7,1);
	x0 << pos(0), pos(1), pos(2), acc(0), acc(1), acc(2), yaw;

	Eigen::MatrixXd jac(1,7);                                  // grad of (r_h*right - left) w.r.t. [pos, acc, yaw]
	const double h = 1e-6;                                     // central-difference step
	for(int i = 0; i < 7; i++){
		Eigen::Vector3d pp = pos, pm = pos, ap = acc, am = acc;
		double yawp = yaw, yawm = yaw;
		if(i < 3){ pp(i) += h;   pm(i) -= h; }
		else if(i < 6){ ap(i-3) += h; am(i-3) -= h; }
		else{ yawp += h; yawm -= h; }
		fov_zero_order ep = evalState(target, pp, ap, yawp);
		fov_zero_order em = evalState(target, pm, am, yawm);
		double dp = r_h*ep.right - ep.left;
		double dm = r_h*em.right - em.left;
		jac(0,i) = (dp - dm) / (2.0*h);
	}

	sol.Jacobian = jac;
	sol.diff = diff0 + (jac * x0)(0);                         // Taylor constant: diff(x0) + grad . x0
	return sol;
}
