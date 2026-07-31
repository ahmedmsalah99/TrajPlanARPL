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
// How replan() anchors each new plan's start state -- see setAnchorOdom()
// and setOdomBlend(). anchorOdom==false forces the blend to 0.
bool anchorOdom = true;
double odomBlend = 1.0;
// Whether the start waypoint leaves jerk and snap free for the optimizer.
// See setFreeStartJerkSnap().
bool freeStartJerkSnap = false;
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
//How much of the measured tracking error each replan folds into the new
//plan's start state. replan() knows where the old plan expected the vehicle
//to be right now (point_info), so (odom - point_info) IS the tracking error,
//and this is the fraction of it applied:
//
//  1.0 (default, unchanged behaviour): the whole error, every cycle. This
//      re-pins the setpoint onto the vehicle and so zeroes the error PX4 is
//      acting on. Since the plan's own acceleration feedforward starts at
//      zero (waypoint's odometry constructor constrains accel/jerk/snap to 0,
//      so accel leaves the origin as O(t^3) and is ~0.003 m/s^2 at t_off),
//      that error is effectively the only thing driving the vehicle -- wiping
//      it each cycle makes the vehicle decelerate at every replan and
//      accelerate between them, a visible stutter.
//
//  0.0: none of it. The trajectory advances purely on its own predicted
//      timeline, which removes the stutter but leaves no feedback at all --
//      measured in flight, successive plans ran 2.6 m ahead of the vehicle in
//      y and ~2 m in z, diverging without bound while the vehicle chased a
//      runaway setpoint above the plan's own peak velocity.
//
//  in between: the setpoint keeps enough lead to drive the vehicle and let
//      the acceleration profile develop, while the error stays bounded.
//      Start around 0.1-0.3.
void setOdomBlend(double in);

//Leave jerk and snap free at each plan's start waypoint, instead of pinning
//them (to zero for an initial plan, or to the previous plan's values for a
//replan).
//
//Position, velocity and acceleration stay constrained either way, so the
//commanded thrust/tilt is still continuous across a replan. But constraining
//jerk AND snap on top of that means a(0)=0, a'(0)=0 and a''(0)=0 for an
//initial plan, so acceleration can only leave the origin as O(t^3) -- the
//plan spends its first second barely accelerating, and its feedforward is
//~0.003 m/s^2 at t_off. Freeing the top two orders lets the optimizer ramp
//acceleration immediately (a ~ j(0)*t), so the plan itself drives the vehicle
//rather than relying on position error to do it.
void setFreeStartJerkSnap(bool in);

//Coarse switch kept for existing configs: false forces the blend above to 0,
//true leaves it at whatever setOdomBlend() was given.
void setAnchorOdom(bool in);

};
#endif
