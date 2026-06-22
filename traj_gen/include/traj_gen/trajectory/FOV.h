#pragma once
#include <Eigen/Eigen>
#include <math.h>
#include "TrajBase.h"
typedef struct {
    Eigen::MatrixXd Jacobian; //Position x,y,z and acceleration x,y,z
 double diff;
} fov_constr;


typedef struct {
    double left ; //Position x,y,z and acceleration x,y,z
    double right;
} fov_zero_order;


class FOV_constraint
{
private:
	const double r_h = .76732;
	//const double r_h = 1;
	const double g = 9.81;
	double camTilt;            // camera mount tilt (rad): optical axis = b3 rotated about b2 by (pi/2 + camTilt)
	Eigen::Vector4d init_pos;
	Eigen::Vector3d init_acc;
	Eigen::Matrix3d R;
	// FOV margin decomposition (on-axis 'right', off-axis 'left') of the target
	// direction for a given position/acceleration (uses the stored yaw + camTilt).
	fov_zero_order evalState(const Eigen::Vector3d& target, const Eigen::Vector3d& pos, const Eigen::Vector3d& acc);
public:
	FOV_constraint(Eigen::Vector4d init_pos, Eigen::Vector3d init_acc, double camTilt = 0.25);
	//returns a matrix that details the inequaltiy constraint
	fov_constr derivative_FOV(Eigen::Vector3d target);
	//returns the first order evaluation
	fov_zero_order fov_eval(Eigen::Vector3d target);
};
