#include <ros_traj_gen_utils/poscmd_publisher.h>
using namespace std;

namespace {
// [THRUST_CHECK] x500 (Gazebo SITL default quadrotor) base_link + rotor mass,
// from PX4/PX4-gazebo-models' x500_base/model.sdf -- hardcoded since this is
// a one-off diagnostic, not a per-vehicle-configurable parameter.
constexpr double kQuadMassKg = 2.0;
constexpr double kGravity = 9.81;
}

poscmd_publisher::poscmd_publisher(rclcpp::Node::SharedPtr node, std::string cmd_topic, double dt )
	: node_(node), begin(node->now()){
	pubCMD = node_->create_publisher<quadrotor_msgs::msg::PositionCommand>(cmd_topic, 10);
	// [THRUST_CHECK] required thrust magnitude implied by the commanded
	// acceleration -- see timerCallback(). Topic derived from cmd_topic
	// (.../position_cmd -> .../required_thrust_n) to match this repo's
	// vehicle_name-prefixed topic convention.
	std::string thrust_topic = cmd_topic;
	const std::string suffix = "/position_cmd";
	if(thrust_topic.size() > suffix.size() &&
	   thrust_topic.compare(thrust_topic.size() - suffix.size(), suffix.size(), suffix) == 0){
		thrust_topic = thrust_topic.substr(0, thrust_topic.size() - suffix.size());
	}
	thrust_topic += "/required_thrust_n";
	pubThrust = node_->create_publisher<std_msgs::msg::Float64>(thrust_topic, 10);
	state =END;
	timer_ = node_->create_wall_timer(
		std::chrono::duration<double>(dt),
		std::bind(&poscmd_publisher::timerCallback, this));
}



void poscmd_publisher::timerCallback(){
	if(state==END){
		return;// no flight
	}
	quadrotor_msgs::msg::PositionCommand point;
	if(state==FLIGHT){
		rclcpp::Time now = node_->now();
		double traj_time = (now-begin).seconds();
		if(traj_time >= (totalTime)){
			traj_time = totalTime;
			state = HOVER;
		}
		//std::cout <<traj_time <<std::endl;

		Eigen::MatrixXd pt;
		pt = currTraj->evalTraj(traj_time);
		geometry_msgs::msg::Point posXYZ;
		posXYZ.x = pt(0,0);
		posXYZ.y = pt(0,1);
		posXYZ.z = pt(0,2);
		geometry_msgs::msg::Vector3 veloXYZ;
		veloXYZ.x = pt(1,0);
		veloXYZ.y = pt(1,1);
		veloXYZ.z = pt(1,2);
		geometry_msgs::msg::Vector3 accelXYZ;
		accelXYZ.x = pt(2,0);
		accelXYZ.y = pt(2,1);
		accelXYZ.z = pt(2,2);

		// [THRUST_CHECK] F_thrust = mass * (a_cmd - g_world), NED
		// (g_world = (0,0,+gravity)). This is the raw commanded/feedforward
		// acceleration only -- PX4's own position/velocity error feedback on
		// top of it can only push the real requirement higher, so this is a
		// conservative lower bound on what thrust is actually needed.
		Eigen::Vector3d thrustAccelCmd(accelXYZ.x, accelXYZ.y, accelXYZ.z);
		Eigen::Vector3d thrustGWorld(0.0, 0.0, kGravity);
		Eigen::Vector3d thrustVec = kQuadMassKg * (thrustAccelCmd - thrustGWorld);
		std_msgs::msg::Float64 thrustMsg;
		thrustMsg.data = thrustVec.norm();
		pubThrust->publish(thrustMsg);

		geometry_msgs::msg::Vector3 jerkXYZ;
		jerkXYZ.x = pt(3,0);
		jerkXYZ.y = pt(3,1);
		jerkXYZ.z = pt(3,2);
		point.position = posXYZ;
		point.velocity = veloXYZ;
		point.acceleration = accelXYZ;
		point.jerk =  jerkXYZ;
		point.yaw = 0;//pt(0,3);
		point.yaw_dot = 0; // pt(1,3);
		point.kx[0] = kx;
		point.kv[0] = kv;
		point.kx[1] = kx;
		point.kv[1] = kv;
		point.kx[2] = kx;
		point.kv[2] = kv;
		point.header.frame_id = frame_id;
		//std::cout << count <<std::endl;
		//if(traj_time >= (currTraj.segmentTimes.back())){
		finalState = point;
		if(totalTime- traj_time < 0.01){
			// NOTE (ROS 2 port): the ROS 1 version set the global parameter
			// /quadrotor/quadrotor_simulator_so3/enableUnity = false here. ROS 2 has no
			// global parameters; setting another node's parameter requires an
			// AsyncParametersClient. This simulator-specific hook is omitted in the port.
		}
			//Set the final state to the last point
		//}
		//else{
			//point = flightTraj[count];
			//position_cmd_history.push_back(point);
			//count+=1;
			//std::cout << count <<std::endl;
		//}

	}
	if(state==HOVER){
		point = finalState;
	}
    point.header.stamp = node_->now();
	pubCMD->publish(point);
}


void poscmd_publisher::setNewFlightPath( TrajBase * traj){
		count = 0; //reset count
		currTraj = traj;
		totalTime = 0.0;
		for(size_t i=0;i<traj->segmentTimes.size();i++){
			totalTime+=traj->segmentTimes[i];
		}
		//Round down the total time
		totalTime = totalTime/0.01;
		totalTime = floor(totalTime);
		totalTime = totalTime*0.01;
		begin = node_->now();
		state = FLIGHT;
}



int poscmd_publisher::getState(){
	return state;
}

void poscmd_publisher::setEND(){
	state =END;
}


//Static Function
void poscmd_publisher::startFlight(TrajBase * traj){
	setNewFlightPath(traj);
}

std::vector<quadrotor_msgs::msg::PositionCommand> poscmd_publisher::arplCMDlist(double dt, double kx, double kv, std::string frame_id, TrajBase * traj){
	std::vector<quadrotor_msgs::msg::PositionCommand>  posCMD;
	Eigen::MatrixXd pos =  traj->calculateTrajectory( 0, dt);
	Eigen::MatrixXd velo =  traj->calculateTrajectory( 1, dt);
	Eigen::MatrixXd accel =  traj->calculateTrajectory( 2, dt);
	Eigen::MatrixXd jerk =  traj->calculateTrajectory( 3, dt);
	//double totalTime = traj.segmentTimes[traj.segmentTimes.size()-1];
	double totalTime = 0.0;
	for(size_t i =0;i<traj->segmentTimes.size();i++){
		totalTime+=traj->segmentTimes[i];
	}
	for(int j =0; j < totalTime/dt; j++){
		quadrotor_msgs::msg::PositionCommand point;
		geometry_msgs::msg::Point posXYZ;
		posXYZ.x = pos(j,0);
		posXYZ.y = pos(j,1);
		posXYZ.z = pos(j,2);
		geometry_msgs::msg::Vector3 veloXYZ;
		veloXYZ.x = velo(j,0);
		veloXYZ.y = velo(j,1);
		veloXYZ.z = velo(j,2);
		geometry_msgs::msg::Vector3 accelXYZ;
		accelXYZ.x = accel(j,0);
		accelXYZ.y = accel(j,1);
		accelXYZ.z = accel(j,2);
		geometry_msgs::msg::Vector3 jerkXYZ;
		jerkXYZ.x = jerk(j,0);
		jerkXYZ.y = jerk(j,1);
		jerkXYZ.z = jerk(j,2);
		point.position = posXYZ;
		point.velocity = veloXYZ;
		point.acceleration = accelXYZ;
		point.jerk =  jerkXYZ;
		point.yaw = pos(j,3);
		point.yaw_dot = velo(j,3);
		point.kx[0] = kx;
		point.kv[0] = kv;
		point.kx[1] = kx;
		point.kv[1] = kv;
		point.kx[2] = kx;
		point.kv[2] = kv;
		point.header.frame_id = frame_id;
		posCMD.push_back(point);
	}
	return posCMD;
}
