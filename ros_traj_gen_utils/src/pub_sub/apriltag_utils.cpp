#include <ros_traj_gen_utils/apriltag_utils.h>
#include <algorithm>
using namespace std;

namespace {
// [SYNC] Odometry at the tag's exact capture instant, interpolated between
// the two buffered samples bracketing it -- position lerp, orientation
// slerp (the correct way to interpolate rotation). At 100Hz buffer sampling,
// snapping to whichever single sample is nearest can be off by up to ~10ms;
// during any real rotation rate that directly rotates the world-frame
// target pose WorldRot() computes, since H_IR's orientation comes straight
// from this sample.
nav_msgs::msg::Odometry interpolateOdometry(const nav_msgs::msg::Odometry& before,
                                             const nav_msgs::msg::Odometry& after,
                                             double alpha){
	alpha = std::clamp(alpha, 0.0, 1.0);
	nav_msgs::msg::Odometry out = after; // header/frame ids etc from the later sample

	out.pose.pose.position.x = before.pose.pose.position.x +
		alpha * (after.pose.pose.position.x - before.pose.pose.position.x);
	out.pose.pose.position.y = before.pose.pose.position.y +
		alpha * (after.pose.pose.position.y - before.pose.pose.position.y);
	out.pose.pose.position.z = before.pose.pose.position.z +
		alpha * (after.pose.pose.position.z - before.pose.pose.position.z);

	Eigen::Quaterniond q_before(before.pose.pose.orientation.w, before.pose.pose.orientation.x,
	                            before.pose.pose.orientation.y, before.pose.pose.orientation.z);
	Eigen::Quaterniond q_after(after.pose.pose.orientation.w, after.pose.pose.orientation.x,
	                           after.pose.pose.orientation.y, after.pose.pose.orientation.z);
	Eigen::Quaterniond q_interp = q_before.slerp(alpha, q_after);
	out.pose.pose.orientation.w = q_interp.w();
	out.pose.pose.orientation.x = q_interp.x();
	out.pose.pose.orientation.y = q_interp.y();
	out.pose.pose.orientation.z = q_interp.z();

	// Velocity too, for consistency -- current_heading is also handed out
	// whole via getLanding(joint_pose*), not just consumed by WorldRot().
	out.twist.twist.linear.x = before.twist.twist.linear.x +
		alpha * (after.twist.twist.linear.x - before.twist.twist.linear.x);
	out.twist.twist.linear.y = before.twist.twist.linear.y +
		alpha * (after.twist.twist.linear.y - before.twist.twist.linear.y);
	out.twist.twist.linear.z = before.twist.twist.linear.z +
		alpha * (after.twist.twist.linear.z - before.twist.twist.linear.z);
	out.twist.twist.angular.x = before.twist.twist.angular.x +
		alpha * (after.twist.twist.angular.x - before.twist.twist.angular.x);
	out.twist.twist.angular.y = before.twist.twist.angular.y +
		alpha * (after.twist.twist.angular.y - before.twist.twist.angular.y);
	out.twist.twist.angular.z = before.twist.twist.angular.z +
		alpha * (after.twist.twist.angular.z - before.twist.twist.angular.z);
	return out;
}
} // namespace

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
	odom_l.outputListiner(msg, node_);
}

//Takes the apriltage detection message and stores it in a perch_constraint  format
void apriltag_utils::aprilListen(const geometry_msgs::msg::PoseStamped &msg){

	std::cout << "[APRILDIAG] aprilListen called, tag stamp="
	          << rclcpp::Time(msg.header.stamp).seconds() << std::endl;

	//if buffer isn't full return
	if ((circle_start+1)!=circle_end){
		std::cout << "[APRILDIAG] DROP: odom buffer not full yet (circle_start="
		          << circle_start << ", circle_end=" << circle_end << ")" << std::endl;
		return;
	}
	//Check if the Odom is being read
	double tag_read = rclcpp::Time(msg.header.stamp).seconds();
	int index = circle_start -40;
	//std::cout << "Tag read" <<std::endl;
	if(index <0){
		index+=BUFFER_SIZE;
	}
	double oldest = rclcpp::Time(odom_buffer[circle_end].header.stamp).seconds();
	int newest_idx = (circle_start - 1 + BUFFER_SIZE) % BUFFER_SIZE;
	double newest = rclcpp::Time(odom_buffer[newest_idx].header.stamp).seconds();
	std::cout << "[APRILDIAG] odom buffer covers [" << oldest << ", " << newest
	          << "] (span=" << (newest - oldest) << "s), tag_read=" << tag_read
	          << " (tag_read - newest=" << (tag_read - newest) << ")" << std::endl;
	bool clamped_newest = false;
	if(tag_read > rclcpp::Time(odom_buffer[index].header.stamp).seconds()){
		while(tag_read > rclcpp::Time(odom_buffer[index].header.stamp).seconds()){
			//std::cout << tag_read - odom_buffer[index].header.stamp.toSec() <<std::endl;
			index=(index+1)%BUFFER_SIZE;
			if(index == circle_start){
				//flag =0;
				//std::cout << "Buffer failed 1" <<std::endl;
				// return;
				index = (circle_start - 1 + BUFFER_SIZE) % BUFFER_SIZE;
				clamped_newest = true;
				std::cout << "[APRILDIAG] tag newer than whole buffer, "
				          << "clamped to newest sample" << std::endl;
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
				std::cout << "[APRILDIAG] DROP: tag older than whole buffer "
				          << "(tag_read - oldest=" << (tag_read - oldest) << "s)" << std::endl;
				return;
			}
		}
	}

	// [SYNC] index brackets tag_read from one side (whichever the search
	// above found); interpolate against its neighbor on the other side so
	// current_heading reflects the vehicle's pose at the tag's actual
	// capture instant, not whichever single buffered sample happened to be
	// nearest. Not possible when clamped to the newest sample (no neighbor
	// exists beyond it) or on an exact timestamp match.
	double t_index = rclcpp::Time(odom_buffer[index].header.stamp).seconds();
	if(clamped_newest || t_index == tag_read){
		current_heading = odom_buffer[index];
		std::cout << "[APRILDIAG] SUCCESS: synced tag to odom_buffer[" << index
		          << "] (no interpolation), |dt|=" << (tag_read - t_index) << "s" << std::endl;
	}
	else if(t_index < tag_read){
		// index is the "before" sample.
		int idx_after = (index + 1) % BUFFER_SIZE;
		double t_after = rclcpp::Time(odom_buffer[idx_after].header.stamp).seconds();
		double alpha = (t_after > t_index) ? (tag_read - t_index) / (t_after - t_index) : 0.0;
		current_heading = interpolateOdometry(odom_buffer[index], odom_buffer[idx_after], alpha);
		std::cout << "[APRILDIAG] SUCCESS: synced tag between odom_buffer[" << index << "]/["
		          << idx_after << "], alpha=" << alpha << " (interpolated)" << std::endl;
	}
	else{
		// index is the "after" sample.
		int idx_before = (index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
		double t_before = rclcpp::Time(odom_buffer[idx_before].header.stamp).seconds();
		double alpha = (t_index > t_before) ? (tag_read - t_before) / (t_index - t_before) : 0.0;
		current_heading = interpolateOdometry(odom_buffer[idx_before], odom_buffer[index], alpha);
		std::cout << "[APRILDIAG] SUCCESS: synced tag between odom_buffer[" << idx_before << "]/["
		          << index << "], alpha=" << alpha << " (interpolated)" << std::endl;
	}
	current_target = msg;
	flag = 1;
}

bool apriltag_utils::getLanding(Eigen::Matrix4d * pointer_in){
	if(flag==1){
		//aprilOdomSub.shutdown();
		std::cout << "Succesful Read" <<std::endl;
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
	std::cout << "vivion flag is false " << std::endl;
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
