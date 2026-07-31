#ifndef _ros_replanner_utils_h
#define _ros_replanner_utils_h
#include <vector>
#include <Eigen/Eigen>
#include <iostream>
#include "odom_utils.h"
#include <rclcpp/rclcpp.hpp>
#include <traj_gen/trajectory/Waypoint.h>
#include <traj_gen/trajectory/QPpolyTraj.h>
#include <ros_traj_gen_utils/apriltag_utils.h>
#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features


class ros_replan_utils {
private:
TrajBase * trajectory;
int initial_plan = 0;
odom_utils * odom_l;
std::vector<waypoint> future_v;
std::vector<double> segmentTimes;
Eigen::Matrix4d prevTarget;
double forwardV = 0.0;
double pitch = 0.0;
int fullStop = 0.0;
int curr_v =0;
//NEW PLAN WHAT IS A GOOD IDEA HERE
//Target updates
bool visualFeedback = false;
// How replan() anchors each new plan's start state. True (default) re-pins it
// to the measured odometry every cycle; false continues from where the
// previous plan predicted the vehicle would be. See setAnchorOdom().
bool anchorOdom = true;
bool fovEnable = false;
// Fraction of the remaining segment's duration (measured from the current
// replan point) over which the FOV constraint is enforced; the rest of the
// segment (the final approach, where perch terminal dynamics dominate) is
// left unconstrained so requiring FOV there can't make the whole solve
// infeasible.
double fovCoverageFraction = 0.5;
//Replanner retry tuning (set from config; defaults preserve prior behavior)
double retryStep = 0.2;   // seconds added to segment time(s) per failed solve
int retryMax = 10;        // max solve retries before giving up / reverting
double minSegTime = 0.5;  // segments shorter than this are merged/skipped
public:
ros_replan_utils();

//Initial Points you wish to pass through
ros_replan_utils(TrajBase * traj, odom_utils* odom,std::vector<waypoint>*  vertices, bool visual_in);
void set_params(TrajBase * traj, odom_utils* odom,std::vector<waypoint>*  vertices, bool visual_in);

//Get the trajectory 
TrajBase * getTraj();

//Initializing PLan
bool initialPlan(int degreeOpt);
bool initialPlan(int degreeOpt, Eigen::Matrix4d target);
//Elapesed time from the last replanning
//Replanning when you plan to this replanner uses the previous target acqueisiton
bool replan(int degreeOpt, double t_elap, double t_off);
//This replanner uses a new final target acquisition 
bool replan(int degreeOpt, double t_elap, double t_off, Eigen::Matrix4d Target);
void setFOVEnable(bool in);
//Configure what fraction (0-1] of the remaining segment's duration the FOV
//constraint is enforced over, starting from the current replan point.
void setFOVCoverageFraction(double frac);
void setTime(std::vector<double> times_in);
//Configure retry tuning (step seconds, max retries, min segment time)
void setReplanParams(double step, int maxRetries, double minSeg);
//Choose what replan() anchors a new plan's start position/velocity to.
//
//  true  (default, unchanged behaviour): the measured odometry, plus the
//        displacement the old plan covers over t_off. This re-pins the
//        setpoint onto the vehicle every replan, which also resets the
//        tracking error PX4 is acting on -- and since the plan's own
//        acceleration feedforward starts at zero (waypoint's odometry
//        constructor constrains accel/jerk/snap to 0, so accel leaves the
//        origin as O(t^3) and is still ~0.003 m/s^2 at t_off), that error is
//        effectively the only thing driving the vehicle. Wiping it every
//        cycle makes the vehicle decelerate at each replan and accelerate
//        again between them.
//
//  false: the previous plan's predicted state at the same instant. The
//        trajectory then advances on its own timeline, the tracking error
//        survives the replan, and the acceleration profile is allowed to
//        develop. Drift is not corrected here, but PX4 closes the position
//        loop against the setpoint anyway.
void setAnchorOdom(bool in);

};
#endif
