#ifndef _ros_traj_arpl_utils_h
#define _ros_traj_arpl_utils_h
#include <vector>
#include <Eigen/Eigen>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <traj_gen/traj_utils/quaternion.h>
#include <traj_gen/trajectory/Waypoint.h>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <traj_gen/trajectory/TrajBase.h>

#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features
#define HOVER 0
#define FLIGHT 1
#define END 2
class poscmd_publisher {
private:
std::vector<quadrotor_msgs::msg::PositionCommand>  flightTraj;
volatile int count = 0;
quadrotor_msgs::msg::PositionCommand finalState;
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pubCMD;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Time begin;
void setNewFlightPath(TrajBase * traj);

TrajBase * currTraj;
bool normalPoly = true;
double totalTime = 0.0;
//Should be moved to polynomial messages.
public:
int state = HOVER;
std::string frame_id="simulator";
double kx=7.4;
double kv=4.8;
std::vector<quadrotor_msgs::msg::PositionCommand> position_cmd_history;
poscmd_publisher( rclcpp::Node::SharedPtr node, std::string cmd_topic, double dt);
static std::vector<quadrotor_msgs::msg::PositionCommand>  arplCMDlist(double dt, double kx, double kv, std::string frame_id, TrajBase * traj); //ARPL COMMAND SPECIFIC

void startFlight(TrajBase * traj);

// Real elapsed time (s) since the currently-committed trajectory's t=0, i.e.
// since startFlight() was last called -- the SAME clock timerCallback() uses
// (traj_time = now-begin) to sample currTraj and actually publish
// PositionCommands. This is the authoritative "how far along the trajectory
// the vehicle is actually being commanded right now"; callers computing
// their own independent elapsed-time estimate for replan()'s t_elap can
// silently drift from what's really being flown.
double getTrajTime();

void setEND();
//Timer Callback
void timerCallback();

void endFlight();
int getState();
};

#endif
