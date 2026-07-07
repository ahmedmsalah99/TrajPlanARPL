#include <ros_traj_gen_utils/apriltag_utils.h>
using namespace std;

apriltag_utils::apriltag_utils(){
	// Defaults reproduce the original hard-coded extrinsics (camera 0.3 m forward,
	// identity rotation, no tag translation offset). Override via setExtrinsics()
	// from config.
	setExtrinsics(Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
}

// Build the camera-in-body transform H_RC and the tag->target transform H_TAG.
//  - H_RC rotation = camToBodyRot, the full camera-frame -> body-frame rotation.
//    This covers both a fixed mount convention (e.g. a nadir-facing camera whose
//    axes don't line up 1:1 with the body axes) and any additional tilt, composed
//    by the caller (e.g. Rtilt_y * body_R_cam for a camera tilted up from nadir by
//    some angle). The tag pose is taken directly in the camera frame.
//  - camTranslation is the camera position offset in the body frame.
//  - H_TAG rotation is a fixed convention; tagTranslation is its translation offset.
void apriltag_utils::setExtrinsics(const Eigen::Vector3d& camTranslation, const Eigen::Matrix3d& camToBodyRot,
                                   const Eigen::Vector3d& tagTranslation){
	H_RC = Eigen::Matrix4d::Identity();
	H_RC.block<3,3>(0,0) = camToBodyRot;
	H_RC.block<3,1>(0,3) = camTranslation;

	H_TAG = Eigen::Matrix4d::Identity();
	H_TAG.block<3,1>(0,3) = tagTranslation;
}

void apriltag_utils::setNode(rclcpp::Node::SharedPtr node){
	node_ = node;
	// Buffer-filling timer, created once at start: samples the latest odometry
	// (fed via updateOdom) into the circular buffer for time-syncing detections.
	timer = node_->create_wall_timer(std::chrono::milliseconds(10),
		std::bind(&apriltag_utils::timerCallback, this));
}

void apriltag_utils::timerCallback()
{

	static bool first_read = false;
	nav_msgs::msg::Odometry header;
	//if buffer isn't full check if the last value has the same time stamp
	//if the time stamps are the same then remove it redundent call backs bad. 
	if (first_read){
		if(	odom_l.getCurrOdom(&header)){
			//std::cout << " Time of odom " <<std::endl;
			//std::cout << header.header.stamp << std::endl;
			int index = circle_start - 1;
			if(index < 0){
				index +=BUFFER_SIZE;
			}
			//remove redudent timer callbacks
			if(abs(rclcpp::Time(header.header.stamp).seconds() - rclcpp::Time(odom_buffer[index].header.stamp).seconds()) < 0.01){
				return;
			}
		}
	}
	first_read = true;
	if(	odom_l.getCurrOdom(&header)){
		odom_buffer[circle_start] = header;
		circle_start=(circle_start+1)%BUFFER_SIZE;
		if(circle_start==circle_end){
			circle_end = (circle_end+1)%BUFFER_SIZE;
		}
	}
}

void apriltag_utils::updateOdom(const nav_msgs::msg::Odometry &msg){
	// Store the latest odometry; the startup timer samples it into the circular
	// buffer. Fed from the main odom callback (no dedicated subscription needed).
	odom_l.outputListiner(msg);
}

//Takes the apriltage detection message and stores it in a perch_constraint  format
void apriltag_utils::aprilListen(const geometry_msgs::msg::PoseStamped &msg){
	
	//if buffer isn't full return 
	if ((circle_start+1)!=circle_end){
		return;
	}
	//Check if the Odom is being read
	double tag_read = rclcpp::Time(msg.header.stamp).seconds();
	int index = circle_start -40;
	//std::cout << "Tag read" <<std::endl;
	if(index <0){
		index+=BUFFER_SIZE;
	}
	if(tag_read > rclcpp::Time(odom_buffer[index].header.stamp).seconds()){
		while(tag_read > rclcpp::Time(odom_buffer[index].header.stamp).seconds()){
			//std::cout << tag_read - odom_buffer[index].header.stamp.toSec() <<std::endl;
			index=(index+1)%BUFFER_SIZE;
			if(index == circle_start){
				//flag =0;
				//std::cout << "Buffer failed 1" <<std::endl;
				// return;
				index = (circle_start - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        		break;
			}
		}
	}
	else{
		while(tag_read < rclcpp::Time(odom_buffer[index].header.stamp).seconds()){
			index-=1;
			if(index <0){
				index+=BUFFER_SIZE;
			}
			if(index == circle_end){
				//flag =0;
				//std::cout << "Buffer failed 2" <<std::endl;
				return;
			}
		}
	}
	//std::cout << "Buffer SUccess" <<std::endl;
	current_heading = odom_buffer[index];
	current_target = msg;
	flag = 1;
}

bool apriltag_utils::getLanding(Eigen::Matrix4d * pointer_in){
	if(flag==1){
		//aprilOdomSub.shutdown();
		//std::cout << "Succesful Read" <<std::endl;
		joint_pose comb_pose;
		comb_pose.quad =  current_heading;
		comb_pose.target = current_target.pose;
		*pointer_in = WorldRot(comb_pose);
		/*std::cout << " Current quadrotor " <<std::endl;
		std::cout << current_heading.header.stamp <<std::endl;
		std::cout << " Current Target " <<std::endl;
		std::cout << current_target.header.stamp <<std::endl;*/
		//flag = 0;
		//std::cout << "Odometry listiner thereshold " << odom_l.now - current_target.header.stamp.toSec() <<std::endl;
		//aprilOdomSub = nh.subscribe(topic_name, 1, &odom_utils::outputListiner, &odom_l);
		return true;
	}
	return false;

} 

bool apriltag_utils::getLanding(Eigen::Matrix4d * pointer_in, nav_msgs::msg::Odometry * msg2){
	if(flag==1){
		//aprilOdomSub.shutdown();
		//std::cout << "Succesful Read" <<std::endl;
		joint_pose comb_pose;
		comb_pose.quad =  current_heading;
		comb_pose.target = current_target.pose;
		*pointer_in = WorldRot(comb_pose);
		//msg2->header  = current_target.header;
		//std::cout << " Current difference " <<std::endl;
		//std::cout << current_heading.header.stamp -current_target.header.stamp <<std::endl;
		//std::cout << " Current Target " <<std::endl;
		//std::cout << current_target.header.stamp <<std::endl;
		//std::cout << "Odometry listiner thereshold " << odom_l.now - current_target.header.stamp.toSec() <<std::endl;
		//std::cout << current_heading.header.stamp <<std::endl;
		//std::cout << current_target.header.stamp <<std::endl;
		//aprilOdomSub = nh.subscribe(topic_name, 1, &odom_utils::outputListiner, &odom_l);
		return true;
	}
	return false;

} 

nav_msgs::msg::Odometry apriltag_utils::convertMsg(Eigen::Matrix4d H, nav_msgs::msg::Odometry header){
	nav_msgs::msg::Odometry  pose;
	pose.header = header.header;
	pose.pose.pose.position.x = H(0,3);
	pose.pose.pose.position.y = H(1,3);
	pose.pose.pose.position.z = H(2,3);
	Eigen::Quaterniond q(H.block<3,3>(0,0));
	pose.pose.pose.orientation.x = q.x();
	pose.pose.pose.orientation.y = q.y();
	pose.pose.pose.orientation.z = q.z();
	pose.pose.pose.orientation.w = q.w();
	return pose;
}


bool apriltag_utils::getLanding(joint_pose * pointer_in){
	if(flag==1){
		//ros::NodeHandle nh;
		//aprilOdomSub.shutdown();
		//std::cout << "Succesful Read" <<std::endl;
		pointer_in->quad =  current_heading;
		pointer_in->target = current_target.pose;
		//aprilOdomSub = nh.subscribe(topic_name, 1, &odom_utils::outputListiner, &odom_l);
		return true;
	}
	return false;

} 


Eigen::Matrix4d apriltag_utils::WorldRot(joint_pose pose){
	//Convert Odometry to Homogenous Transform 
	nav_msgs::msg::Odometry odom_read = pose.quad;
	Eigen::Matrix4d H_IR= Eigen::Matrix4d::Zero();
	H_IR(0,3) = odom_read.pose.pose.position.x;
	H_IR(1,3) = odom_read.pose.pose.position.y;
	H_IR(2,3) = odom_read.pose.pose.position.z;
	H_IR(3,3)=1;
	Eigen::Quaterniond q;
	q.x() = odom_read.pose.pose.orientation.x;
	q.y() = odom_read.pose.pose.orientation.y;
	q.z() = odom_read.pose.pose.orientation.z;
	q.w() = odom_read.pose.pose.orientation.w;   // x, y, z, w in order
	/**< quaternion -> rotation Matrix */
	H_IR.block<3,3>(0,0) = q.toRotationMatrix();
	//std::cout << "H_IR" <<H_IR <<std::endl;
	//std::cout << "camera w.r. inertial frame" << H_IR*H_RC <<std::endl;

	//Convert Apriltag to Homogenous Transform;
	Eigen::Matrix4d H_CT = Eigen::Matrix4d::Zero();	
	geometry_msgs::msg::Pose bundle = pose.target;
	H_CT(0,3) = bundle.position.x;//*0.25;
	H_CT(1,3) = bundle.position.y;//*0.25;
	H_CT(2,3) = bundle.position.z;//*0.25;
	H_CT(3,3)=1;
	q.x() = bundle.orientation.x;
	q.y() = bundle.orientation.y;
	q.z() = bundle.orientation.z;
	q.w() = bundle.orientation.w;   // x, y, z, w in order
	Eigen::Matrix3d flipper = Eigen::Matrix3d::Zero();
	flipper(0,0) = 1;
	flipper(1,1) = -1;
	flipper(2,2) = -1;
	H_CT.block<3,3>(0,0) = q.toRotationMatrix();
	H_CT = H_CT*H_TAG;
	//H_CT.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
	//std::cout << "H_CT" <<H_CT <<std::endl;
	/*
	Eigen::Matrix4d H_CT_inv = H_CT.inverse();
	std::cout << "H_CT_inv" <<H_CT_inv <<std::endl;
	H_CT.block<3,3>(0,0) = H_CT.block<3,3>(0,0) *flipper;
	std::cout << "H_CT_diag_flip" <<H_CT <<std::endl;
*/
	//std::cout << "Process Apriltag to Homogenous Matrix " <<std::endl;
	Eigen::Matrix4d H_IT = H_IR*H_RC*H_CT;
	return H_IT;
}
