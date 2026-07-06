#pragma once
#include "TrajBase.h"

#include <iostream>
#include "Waypoint.h"
#include <traj_gen/traj_utils/EigenQP.h>
#include <traj_gen/ooqp_interface/OoqpEigenInterface.hpp>
#include <vector>
#include <cassert>

class QPpolyTraj : public TrajBase
{
protected:
	 Eigen::MatrixXd generateObjFun(int minDeriv); //Generates the Q matrix 
	 QP_constraint genConstraint(int dimension, int numConstraint); 
	 QP_ineq_const genInEqConstraint( int dimension); //generates inequalty constraints. 
	//Generate Joint Constraints one matrix that encapsulates all dimension
	 QP_constraint genJointConstraint(); //Generates the A and B matrix
	 QP_ineq_const genInEqJointConstraint(); //generates inequalty constraints. 
	 Eigen::MatrixXd generateJointObjFun(int minDeriv); // Helper function for the gerenate
	 Eigen::MatrixXd generateQ(int minDeriv, double time); // Helper function for the gerenate ObjFunction
	 // The EXACT number of equality rows genConstraint() will write for a single
	 // dimension, given the current vertices: vertex 0 always contributes 5 rows
	 // (pos + vel/accel/jerk/snap, unconditionally -- unset ones are forced to 0);
	 // the last vertex contributes 1 (pos) + one row per SET higher derivative
	 // (unset ones are correctly left out, i.e. free); interior vertices
	 // contribute 2 (position continuity) + two rows per SET higher derivative.
	 // Must be kept in exact sync with genConstraint()'s row-writing logic --
	 // previously this was a fixed formula (6*vertices.size()-2 + ...) that
	 // assumed every vertex sets all 4 higher derivatives, so a vertex that
	 // deliberately leaves some free (e.g. a perch terminal's unset jerk/snap)
	 // left the pre-allocated A/b rows for those derivatives never written --
	 // trailing all-zero rows that rank-deficiency the assembled equality matrix.
	 int countEqConstraintRows();

public:
    bool fast = false; //Determines if we fast solve. Fast solve is faster
    QPpolyTraj();
    // However it comes at the loss of optimality will pick a near optimal solution
    //constructor d is how many dimensions a point has 3 if only X,y,Z 4 if X,Y,Z,yaw
    QPpolyTraj(int d);
    QPpolyTraj(int d, std::vector<double> time);
    // Uses OOQP to Solve average time 2mS on laptop
    // Solves a much tighter while also smooth trajectory compare to fast solve
    //Fast Solve solves a similar problem to the QP problem though with a slight modification to increase speed
    //Average time 0.2mS on Laptop around 10x faster than solve above
    //Downside the trajectory will be slightly more loose and less optimal than solve
    //Left in to demonstrate fast solver
    Eigen::MatrixXd MTsolve(int minDeriv); 
    Eigen::MatrixXd fastMTSolve(int minDeriv); 
    Eigen::MatrixXd fastSolve(int minDeriv); 
    Eigen::MatrixXd SMsolve(int minDeriv);


	
	//Helper function that checks if the global boundary is satisified based on its coefficients 
	bool calcGlobalBound(int segNum, int derivOrder);
	bool calcThrBound(int segNum, float bounds);
	bool calcThrRatBound(int segNum, float bounds);
	bool calcAngVBound(int segNum, float bounds);




	//returns true/false depending on solving
	Eigen::MatrixXd solve(int minDeriv);
	//Evaluate Trajectory
	Eigen::VectorXd calculateCurrentPt( int Order, double time);
	Eigen::MatrixXd evalTraj(double time);
	Eigen::MatrixXd calculateTrajectory(int Order, double dt);
	Eigen::VectorXd basis(double time, int derivative);


};
