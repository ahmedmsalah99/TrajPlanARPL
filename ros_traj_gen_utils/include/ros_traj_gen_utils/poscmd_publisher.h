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
#include <std_msgs/msg/float64.hpp>
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
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubThrust;
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

//Publish the trajectory's t=0 setpoint continuously, WITHOUT starting the
//trajectory clock. Used while waiting for offboard to be enabled: PX4 needs an
//already-flowing setpoint stream before it will accept OFFBOARD at all, so
//publishing nothing until the flight starts would deadlock against a bridge
//that refuses to enable offboard on a stale command. Holding the plan's START
//point (rather than letting the clock run out and holding its END point) also
//means offboard engages with the vehicle already where the trajectory begins.
void holdTrajectoryStart(TrajBase * traj);

void setEND();
//Timer Callback
void timerCallback();

void endFlight();
int getState();
};

#endif
