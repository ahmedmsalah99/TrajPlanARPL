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
// Whether replan(), when the real-time countdown on the final segment is
// about to run out (see lastSegmentLow in the .cpp), rescues it with a fresh
// distance-based estimate from autogenTimeSegment() instead of giving up.
// The countdown itself (shrinking the one total duration autogenTimeSegment()
// computed at the very first initialPlan(), by real elapsed time each cycle)
// stays the default source of truth either way -- this only controls what
// happens when that countdown would otherwise go infeasible. See
// setReallocateTime().
bool reallocateTime = true;
// Set once replan() finds the last segment's honestly-computed remaining
// duration below minSegTime -- i.e. there isn't enough real time left before
// the NEXT scheduled replan call to safely re-derive anything. While true,
// replan() declines immediately without touching anything, leaving the
// already-installed (and, thanks to reallocateTime's continuous correction
// up to this point, trustworthy) trajectory to fly to its own natural
// completion. Reset to false at the start of every fresh flight -- see
// initialPlan().
bool committedFinalApproach = false;
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

//replan() never called autogenTimeSegment() again after initialPlan() -- it
//only ever shrank the one total duration computed once, at the very first
//initialPlan(), based on the straight-line distance to target at flight
//start. That estimate knows nothing about min_altitude, the perch band, or
//any other constraint that makes the real, constrained path slower than a
//straight line. If it under-estimates, the countdown eventually runs out
//while the vehicle is still far from the target -- confirmed in the field:
//repeated "can't replan ... segmentTimes[curr_v] 0" events, followed by the
//flight ending (a single "replanning time done, take a hover") well short of
//the target, with nothing ever restarting it.
//
//true (default): when the countdown on the LAST segment is about to go
//     infeasible (below minSegTime), rescue it -- recompute a fresh
//     distance-based estimate via autogenTimeSegment() from the anchor's
//     ACTUAL current position, and take the max with the countdown. This
//     only tops up a failing budget; a healthy countdown is left alone.
//     (An earlier version called autogenTimeSegment() unconditionally, every
//     cycle, replacing the countdown outright -- that made the allocated
//     time highly sensitive to ordinary tracking/vision noise near the
//     target, since small distance changes swing the heuristic's output by
//     several seconds per metre at typical v_max/a_max, and each swing
//     re-pinned the perch terminal condition to a different deadline,
//     producing its own violent terminal-maneuver whiplash.)
//
//     The rescued estimate is NOT floored at minSegTime (an earlier version
//     did this). Flooring forces every near-arrival segment to span at least
//     minSegTime regardless of how little real distance is left, which
//     throttles the commanded approach velocity every cycle (distance keeps
//     shrinking but the time budget keeps resetting to the same floor) --
//     an asymptotic stall that never quite lets the vehicle finish. Instead,
//     once the honestly-computed remaining duration is itself below
//     minSegTime -- genuinely too little time for another safe replan cycle
//     -- replan() commits: see committedFinalApproach. That lets the final,
//     honestly-short approach actually fly at the speed it needs to, while
//     still avoiding the anchor-corruption spiral a too-short, but
//     REPLACED-anyway, trajectory caused (evalTraj() clamping the next
//     cycle's anchor query to the trajectory's endpoint once t_elap exceeds
//     its duration -- and with replan_odom_blend at 0, that fictitious
//     "already there" anchor feeding an even smaller next estimate).
//false: original behaviour -- only ever shrink the one duration computed
//     at flight start, give up permanently if it runs out.
void setReallocateTime(bool in);

};
#endif
