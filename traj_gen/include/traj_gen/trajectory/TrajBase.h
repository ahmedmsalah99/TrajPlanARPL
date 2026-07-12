#pragma once
#include <iostream>
#include "Waypoint.h"
#include <vector>
#include <traj_gen/traj_utils/polynomial.h>
#include <math.h>
#include <boost/thread.hpp>
#include "FOV.h"

#define PI 3.14159265

typedef struct {
    Eigen::MatrixXd a, b;
    } QP_constraint; //Ax = b where A is the optimization

typedef struct {
    Eigen::MatrixXd C;
 Eigen::VectorXd d,f; //d <= Cx <= f. Where x is the opitmization variable
} QP_ineq_const;

class TrajBase
{
protected:
	int dim=4; // The dimensions of our vertices.  
	int polyOrder = 10; //we assume for this project our polynomial order is always 10
	//Virtual functions that generate basis functions based on C++
	virtual Eigen::MatrixXd generateObjFun(int minDeriv)=0; //Generates the cost matrix 
	virtual QP_constraint genConstraint(int dimension, int numConstraint)=0; //Generates equality constraints from waypoints
	virtual QP_ineq_const genInEqConstraint( int dimension)=0; //generates inequalty constraints. 
	
	//Generate Joint Constraints one matrix that handles multidimensionality constraints only relevant for single matrix solve
	virtual QP_constraint genJointConstraint()=0; //Generates the A and B matrix
	virtual QP_ineq_const genInEqJointConstraint()=0; //generates inequalty constraints. 
	virtual Eigen::MatrixXd generateJointObjFun(int minDeriv)=0; // Helper function for the gerenate
	bool condCheck(); //Checks if the global condition is satisified 
	//A vector to indicate if the first 4 dimensions are solved previously
	std::vector<bool> traj_valid{ false, false, false,false }; 

	//A single cost vector used 
	Eigen::VectorXd costVector;
	bool useCostVector = false;
	//additional constraints in case you need them 
	std::vector<QP_ineq_const> add_ineq_constr;
	std::vector<QP_constraint> add_eq_constr;
	QP_ineq_const add_joint_ineq_constr;

	//Generates the A and B matrix for a single dimension 
	QP_constraint add_joint_eq_constr;
	//Perching preset hyperparams (defaults reproduce the original behavior)
	double maxInclinationAccel = 4.0; // specific thrust (m/s^2) used at a vertical surface; scaled by sin(inclination)
	double impactNormalVel = 1.0;     // vS1: impact speed INTO the surface (m/s)
	double impactSlideVel = -3.0;     // vS3: impact speed ALONG the surface, down-slope (m/s)
	double minPitch = 0.5;            // inclination below this (rad) -> soft landing (zero impact velocity)
	// eq.(14) approach-band parameters
	double perchBandTol = 0.5;        // q   : (1+q) magnitude tolerance of the acceleration band
	double perchWindow = 0.5;         // t_k : window before impact (s) over which the band is enforced
	double perchBandEps = 0.2;        // small additive slack so the band never collapses to a point
	double fovCamTilt = 0.25;         // FOV camera mount tilt (rad)
	double fovMargin = 0.0;           // FOV safety margin: keep linearized margin >= this
	double fovRh = 0.76732;           // FOV cone ratio r/h = tan(horizontal_FOV / 2), eq.(7)
	// eq.(9) trust region: bounds how far the actual sampled point (x,a,yaw) is
	// allowed to be from its linearization point (x0,a0,yaw0), so the first-order
	// Taylor expansion of the FOV cone constraint stays valid. Enforced as a
	// per-axis (L-infinity) box -- a conservative approximation of the paper's
	// L2 ball, kept linear so it fits the existing d <= Cx <= f QP form.
	double fovTrustPos = 0.05;        // m, per axis (x,y,z)
	double fovTrustAcc = 0.2;         // m/s^2, per axis (ax,ay,az)
	double fovTrustYaw = 0.1;         // rad
	// Minimum-altitude constraint: enforced as an upper bound on z across every
	// segment of the trajectory (NED: altitude above the origin = -z).
	bool minAltitudeEnabled = false;
	double minAltitude = 0.0;         // metres above the world origin
	// Horizontal (x,y) dynamic limits, sampled across every segment's interior
	// (not just at waypoints/endpoints) -- see applyHorizontalLimits(). Each is
	// independently optional: a value <= 0 disables that derivative order's
	// limit, matching limits[]'s existing "0 = unset" convention. Enforced as an
	// axis-aligned box on vx/vy (resp. ax/ay, jx/jy) independently, NOT a true
	// circular horizontal-magnitude limit -- this codebase's QP only supports
	// linear d <= Cx <= f constraints (no SOCP), same tradeoff as the FOV eq.(9)
	// trust region. A square box permits a diagonal magnitude up to limit*sqrt(2);
	// halve the configured value if you need a strict circular guarantee instead.
	double horizVelLimit = 0.0;       // m/s, |vx|,|vy| <= this
	double horizAccelLimit = 0.0;     // m/s^2, |ax|,|ay| <= this
	double horizJerkLimit = 0.0;      // m/s^3, |jx|,|jy| <= this
	bool constrainV = true;
	float duration = 0.1;

public:

	//Waypoints the most basic measure. Contains data up to snap inside of it
	std::vector<waypoint> vertices;
	//A list of segment times this is how long you expect to pass through each way point
	//i.e. [1,2] means you travel from waypoint 0->waypoint 1 in times [ 0,1]
	//Waypoint 1 -> waypoint 2 in time [1,3]
	
	std::vector<double> segmentTimes;
	Eigen::MatrixXd coeffSolved;
	//Basic limits 0 is pose,1is velocity, 2 is acceleration used for solving times
	Eigen::VectorXd limits;	
 	virtual ~TrajBase();
	TrajBase();
	
	//Get the dimension of your waypoints
	int getDim();
	//set polynomial order
	void setPolyOrder(int p);
	int getPolyOrder();
	bool checkSolved(); //checks if the solver returned true
	void overideSolve(); //Dangerous function converts all valid to true forcefully.

	//Set time Vector automatically
	float autogenTimeSegment();
	//Shrink the (already-solved) segment times toward the dynamic limits so the
	//trajectory's peak velocity/acceleration reach limits[1]/limits[2] -> minimal
	//execution time within the limits. Only speeds up when there is slack; never
	//slows down (e.g. a perch terminal's fixed free-fall accel is left alone).
	//Re-solves internally; call after a successful solve().
	void minimizeTime(int degreeOpt, int maxIters = 6);
	//Divides the time equally given a duration
	void equalTimeSegment(double duration);
	//Manually set a time vector
	void setTimeSegment(std::vector<double> t);
	void setCostVector(Eigen::VectorXd vector);
	//Overide the waypoints with your new set
	void setVertices(std::vector<waypoint> w);
	//Set a full stop to the trajectory
	void setFullStop();

	//Quadrotor Specific
	//Automatically calculates the landing envelope inequality constraints for the last points.
	//The Forward Velocity refers to how fast much momentum you wish to apply to landing.
	//Gripper dependent 
	//Can either use rotation matrix or declare a pitch
	void calcPerchCond(double pitch);
	//t_now is the ABSOLUTE time (from the start of the current solve's segment 0)
	//that `pose`/`accel` were sampled at; used to locate which segment/local-time
	//the returned constraint's basis rows should be expressed at.
	bool genInEqFOV(double t_now, Eigen::Vector3d target, Eigen::Vector4d pose, Eigen::Vector3d accel, QP_ineq_const * constraint);
	//Diagnostic only: angle (degrees) between the modeled camera optical axis
	//(built from pose/accel/yaw + fovCamTilt, same as genInEqFOV) and the true
	//direction from pose to target. 0 = camera looks straight at the target.
	double checkFovAxisAngle(Eigen::Vector3d target, Eigen::Vector4d pose, Eigen::Vector3d accel);
	void calcPerchCond(Eigen::Matrix4d Rot);
	//Configure perching parameters (maxInclinationAccel, vS1 into-surface, vS3 along-surface, min inclination rad)
	void setPerchParams(double maxInclAccel, double normalVel, double slideVel, double minIncl);
	//Configure the eq.(14) approach band (q tolerance, window t_k in s, eps slack)
	void setPerchBand(double q, double window, double eps);
	//Configure the FOV camera mount tilt (rad)
	void setFovCamTilt(double tilt);
	//Configure the FOV safety margin (linearized margin kept >= this)
	void setFovMargin(double m);
	//Configure the FOV cone ratio r/h = tan(horizontal_FOV / 2), eq.(7)
	void setFovRh(double r_h);
	//Configure the eq.(9) trust region (per-axis box half-widths) bounding how
	//far the trajectory may sample from each FOV constraint's linearization point
	//(pos in m, accel in m/s^2, yaw in rad)
	void setFovTrustRegion(double pos, double acc, double yaw);
	//Configure the minimum-altitude constraint: when enabled, z is upper-bounded
	//across every segment of the trajectory so the world altitude (-z in NED)
	//never drops below minAlt (metres). Disabled by default.
	void setMinAltitude(bool enable, double minAlt);
	//Push the minimum-altitude inequality onto every vertex (1..end) of the
	//CURRENT vertex list. Call after segmentTimes is set for this plan (e.g.
	//after autogenTimeSegment(), or after the replan-rebuilt segmentTimes) --
	//it uses each segment's exact duration as the constraint window. No-op if
	//the constraint is disabled or segmentTimes isn't sized to vertices yet.
	void applyMinAltitude();
	//Configure the horizontal (x,y) velocity/acceleration/jerk limits. Each
	//value <= 0 disables that derivative order's limit.
	void setHorizontalLimits(double velLimit, double accelLimit, double jerkLimit);
	//Push the horizontal vel/accel/jerk inequalities onto every vertex (1..end)
	//of the CURRENT vertex list, sampling each segment's interior at dt so the
	//limit is enforced all along the trajectory, not just at waypoints. Same
	//calling convention/caveats as applyMinAltitude(): call after segmentTimes
	//is set for this plan; no-op if all three limits are disabled or
	//segmentTimes isn't sized to vertices yet.
	void applyHorizontalLimits();

	//Set Constraints
	//pushes a waypoint into the list
   	 void push_back(waypoint w);
	//gets a waypoint at step t
	waypoint getWaypoint(int t);
	//gives you the number of waypoints
	int numWaypoints();
	
	//General Coefficents 
	void push_ineq_constr(QP_ineq_const constr, int d);
	void push_eq_constr(QP_constraint constr, int d);
	//Add additional joint constraints
	void push_joint_ineq_constr(QP_ineq_const constr);
	void push_joint_eq_constr(QP_constraint constr);


	//clear waypoints
	void clearCostVector();
	void clear_ineq();
	void clear_eq();
	void clear_vertices();	
	void clearAll();

	//Solve the function. This is a virtual function. IT MUST BE DECLARED IN A NEW TRAJECTORY
	//returns true/false depending on solving
	virtual Eigen::MatrixXd solve(int minDeriv) =0;
	//Evaluate Trajectory
	//Calculates the current trajectory at a specific derivative order
	virtual Eigen::VectorXd calculateCurrentPt( int Order, double time)=0;
	//Evaluates the trajectory and returns a matrix
	//Dim (number of indepdent axis of motions) x 5(pose up to snap of derivtave)
	virtual Eigen::MatrixXd evalTraj(double time)=0;
	//Returns an entire list of all trajectories
	virtual Eigen::MatrixXd calculateTrajectory(int Order, double dt)=0;
	//Returns the trajectoroy basis functions with n derivatives
	//example polynomial order 3, time = 1, derivative = 0
	//[1,1,1,1]
	//example polynomial order 3, time = 1, derivative = 1
	//[0,1,2,3]
	virtual Eigen::VectorXd basis(double time, int derivative)=0;

};

