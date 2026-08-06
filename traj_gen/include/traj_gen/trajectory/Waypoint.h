#include <Eigen/Eigen>
#ifndef Waypoint_H
#define Waypoint_H
#include <nav_msgs/msg/odometry.hpp>
#include <vector>


typedef struct {
    int derivOrder = 0;
    double timeOffset = 0.0;
    //Seconds before this vertex where sampling STOPS. timeOffset opens the
    //window at (time - timeOffset); endOffset closes it at (time - endOffset)
    //instead of at the vertex itself, so the tail of the segment is left
    //unconstrained. 0 keeps the original behaviour (sample right up to the
    //vertex).
    double endOffset = 0.0;
    // When true, genInEqConstraint()/genInEqJointConstraint() ignore
    // timeOffset entirely and instead open the window near the segment's
    // OWN start (a fixed, always-valid ineqSampleDt) every time they're
    // called, re-deriving the window's width from whatever the segment's
    // CURRENT duration actually is at sample-time.
    //
    // timeOffset alone can't express "constrain nearly this whole segment"
    // robustly: it's a snapshot computed once, when the constraint is built
    // (e.g. segmentTimes[i-1] - ineqSampleDt at that instant), but the QP
    // solve's retry loop grows segmentTimes afterwards without ever
    // recomputing it. Since sampling opens the window at (time - timeOffset)
    // using the CURRENT (grown) time against that FROZEN timeOffset, the
    // window's fixed width doesn't cover the newly-added time -- it silently
    // slides forward, uncovering an amount of the segment's start equal to
    // however much the retry loop has grown it by. Confirmed in the field:
    // MIN_ALTITUDE (which wants "constrain nearly the whole segment") was
    // observed needing many retries to solve, and each one was quietly
    // shrinking how much of the segment the floor constraint actually
    // covered rather than genuinely finding a compliant shape.
    bool spanFromStart = false;
    Eigen::Vector4d lower, upper;
    Eigen::Vector4d InEqDim; //Declares wether this constraint is active or not
} waypoint_ineq_const;

//The waypoint class contains possible inputs
//
class waypoint {
private:
	Eigen::VectorXd pos; //Position constraint
	Eigen::VectorXd vel; //velcoity constraint 
	Eigen::VectorXd accel; //aceleration constraint
	Eigen::VectorXd jerk; //aceleration constraint
	Eigen::VectorXd snap; //aceleration constraint
	Eigen::VectorXd status; //This is a variable that determines the order of the waypoint.
	int dim; // how many dimensions are our vectors. 
	//For example status 0 means only position constraint is set
public: 
	std::vector<waypoint_ineq_const> ineq_constraint;

	//Constructor if we choose not to set one of the values 
	//Initialize the waypoint from an odometry message 
	waypoint(nav_msgs::msg::Odometry odom);
	//Null means this waypoint has no velocity or acceleration constraint.
	waypoint(Eigen::VectorXd pose);
	//For the Bezier Curves you can add just an inequalit constraint
	waypoint(waypoint_ineq_const ineq_const);
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo);
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo, Eigen::VectorXd accele);		
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo, Eigen::VectorXd accele, Eigen::VectorXd jerke);		
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo, Eigen::VectorXd accele, Eigen::VectorXd jerke, Eigen::VectorXd snape);
	// The below getters have a success or fail condition. In the case the variables were never set
	//return 0 fail. These conditions do not exist.
	//Else reeturn 1 the variables exist and we placed them in the pointer
	int getPos(Eigen::VectorXd* pose);
	int getVelo(Eigen::VectorXd* velo);
	int getAccel(Eigen::VectorXd* accelo);
	int getJerk(Eigen::VectorXd* jerke);
	int getSnap(Eigen::VectorXd* snape);
	//Get Constraint puts the value of the constraint order in to that pointer if possible
	//then returns a 1 to indicate success 
	// If not possible it does nothing to your pointer and returns a 0
	//The order of your polynomial constraint is 0 for pos 1 for velocity etc...
	int getConstraint(Eigen::VectorXd* output, int order);
	Eigen::VectorXd getStatus();
	//Choose which derivative orders are constrained at this waypoint. Entry i
	//== 1 constrains order i (0=pos, 1=vel, 2=accel, 3=jerk, 4=snap); 0 leaves
	//it free for the optimizer. getConstraint() only emits an equality row for
	//entries set to 1, so zeroing an entry genuinely removes that constraint.
	void setStatus(Eigen::VectorXd input);
	//Sets the variable throws an exception if your dimensions do not match
	void setPos(Eigen::VectorXd input);
	void setVel(Eigen::VectorXd input);
	void setAccel(Eigen::VectorXd input);
	void setJerk(Eigen::VectorXd input);
	void setSnap(Eigen::VectorXd input);

    //for Bernstein Polynomial time is irrelevant it will treat this as the whole corridor
	void addInEqualityConstraint(waypoint_ineq_const ineq_const);
	//Give dimension
	int getDim();
	//Sets all higher order derivatves 1-4 to 0 as a constraint.
	void setFullStop();
};
#endif
