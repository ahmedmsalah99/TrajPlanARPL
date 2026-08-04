#include <ros_traj_gen_utils/ros_replanner_utils.h>
#include <ros_traj_gen_utils/ros_traj_utils.h>
#include <iostream>
#include <algorithm>

namespace {
// status entries are (pos, vel, accel, jerk, snap); 1 constrains that order.
// Keep the first three -- they make the commanded thrust/tilt continuous --
// and release the top two so the optimizer can choose them.
void relaxStartJerkSnap(waypoint * w){
	Eigen::VectorXd st = w->getStatus();
	if(st.rows() >= 5){
		st[3] = 0;
		st[4] = 0;
		w->setStatus(st);
	}
}
} // namespace
using namespace std;

ros_replan_utils::ros_replan_utils(){
	
}


ros_replan_utils::ros_replan_utils(TrajBase * traj, odom_utils* odom, std::vector<waypoint>* vertices, bool visual_in){
	set_params(traj,  odom, vertices, visual_in);
}


void ros_replan_utils::setTime(std::vector<double> times_in){
	segmentTimes.clear();
	segmentTimes=times_in;
}


void ros_replan_utils::set_params(TrajBase * traj, odom_utils* odom, std::vector<waypoint>* vertices, bool visual_in){
	trajectory = traj;
	odom_l = odom;
	future_v.clear();
	curr_v =0;
	std::vector<waypoint> clone_V(*vertices);
	future_v = clone_V;
	visualFeedback = visual_in;
	if(visualFeedback){	 
		fullStop = 0;
	}else{
		fullStop = 1;
	}
}

TrajBase * ros_replan_utils::getTraj(){
	return trajectory;
}

bool ros_replan_utils::initialPlan(int degreeOpt){
        //std::cout << "initial plan " <<std::endl;
	curr_v =0;
	// trajectory is a single object reused across the whole session (see
	// set_params()'s bare pointer assignment) -- it is NOT a fresh instance per
	// call. Whatever it held before this call may currently be what poscmd_publisher
	// is actively flying via that same pointer. Every failure path below must
	// restore this, exactly like replan() already does, or a failed re-solve
	// (e.g. the idle visual-tracking loop retrying every cycle while waiting for
	// offboard) permanently clobbers a trajectory that is live on a completely
	// separate timer -- leaving it traj_valid=false, so poscmd_publisher's next
	// evalTraj() call returns Constant(5,dim,-1e10) and that garbage gets
	// published as the actual PositionCommand.
	std::vector<waypoint> vertices_prev = trajectory->vertices;
	Eigen::MatrixXd coeffSolved_prev = trajectory->coeffSolved;
	std::vector<double> segmentTimes_prev = trajectory->segmentTimes;
	nav_msgs::msg::Odometry current_heading;
	if(odom_l->getCurrOdom(&current_heading)){
		waypoint start(current_heading);
		if(freeStartJerkSnap){
			relaxStartJerkSnap(&start);
		}
		trajectory->push_back(start);
	}
	trajectory->vertices.clear();
    //std::cout << "first point pushed " <<std::endl;	
	for (int i =0;i < future_v.size();i++){
		trajectory->push_back(future_v[i]);
	}
        //std::cout << "last point added" <<std::endl;
	if(fullStop ==1){
		trajectory->setFullStop();
	}
	else{
		if(!trajectory->calcPerchCond(prevTarget)){
			std::cout << " could not plan flight: perch terminal condition exceeds "
			          << "the configured horizontal limits" << std::endl;
			trajectory->overideSolve();
			trajectory->vertices = vertices_prev;
			trajectory->coeffSolved = coeffSolved_prev;
			trajectory->segmentTimes = segmentTimes_prev;
			return false;
		}
	}

	//set Time
        //std::cout << " initially generate segment time " <<std::endl;
	trajectory->autogenTimeSegment();
	// applyMinAltitude uses each segment's exact duration as its constraint
	// window, so it must run after segmentTimes is populated.
	trajectory->applyMinAltitude();
	trajectory->applyHorizontalLimits();
	segmentTimes = trajectory->segmentTimes;

	//segmentTimes[0] += 0.6;
	//segmentTimes[1] += 0.3;
	//trajectory.segmentTimes = segmentTimes;
	int count = 0;
	Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
	while (!(trajectory->checkSolved())){
		for(int i = 0; i < trajectory->segmentTimes.size();i++){
			trajectory->segmentTimes[i] +=retryStep;
		}
		Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
		count+=1;
		if(count == retryMax){
			std::cout << " could not plan flight" << std::endl;
			trajectory->overideSolve();
			trajectory->vertices = vertices_prev;
			trajectory->coeffSolved = coeffSolved_prev;
			trajectory->segmentTimes = segmentTimes_prev;
			return false;
		}
	}
	// Minimal execution time: shrink the allocation toward the dynamic limits
	// (peak vel/accel reach v_max/a_max) now that we have a feasible solve.
	// trajectory->minimizeTime(degreeOpt);
	segmentTimes = trajectory->segmentTimes;
	return true;
}

bool ros_replan_utils::initialPlan(int degreeOpt, Eigen::Matrix4d target){
        //std::cout << "initial plan " <<std::endl;
	curr_v =0;
	// See the matching comment in the other initialPlan() overload: trajectory
	// is a single object reused for the whole session, possibly the exact one
	// poscmd_publisher is actively flying right now, so every failure path
	// below must restore it rather than leaving it cleared/invalid.
	std::vector<waypoint> vertices_prev = trajectory->vertices;
	Eigen::MatrixXd coeffSolved_prev = trajectory->coeffSolved;
	std::vector<double> segmentTimes_prev = trajectory->segmentTimes;
	nav_msgs::msg::Odometry current_heading;
		trajectory->vertices.clear();

	if(odom_l->getCurrOdom(&current_heading)){
		waypoint start(current_heading);
		if(freeStartJerkSnap){
			relaxStartJerkSnap(&start);
		}
		trajectory->push_back(start);
	}
        //std::cout << "first point pushed " <<std::endl;
	// Make the final waypoint coincide with the target position so the initial
	// plan terminates at the target (mirrors replan(), which overwrites the last
	// waypoint with the target). Replacing the position -- rather than appending
	// a new vertex -- avoids a zero-length final segment when the user's last
	// waypoint already sits at the target. The perch terminal condition
	// (orientation/approach) is applied separately via calcPerchCond below; the
	// existing waypoint's yaw is preserved.
	// if(!future_v.empty()){
	// 	Eigen::VectorXd lastPos;
	// 	future_v[future_v.size()-1].getPos(&lastPos);
	// 	lastPos(0) = target(0,3);
	// 	lastPos(1) = target(1,3);
	// 	lastPos(2) = target(2,3);
	// 	future_v[future_v.size()-1].setPos(lastPos);
	// }
	for (int i =0;i < future_v.size();i++){
		trajectory->push_back(future_v[i]);
	}
        //std::cout << "last point added" <<std::endl;
	if(!trajectory->calcPerchCond(target)){
		std::cout << " could not plan flight: perch terminal condition exceeds "
		          << "the configured horizontal limits" << std::endl;
		trajectory->overideSolve();
		trajectory->vertices = vertices_prev;
		trajectory->coeffSolved = coeffSolved_prev;
		trajectory->segmentTimes = segmentTimes_prev;
		return false;
	}
	prevTarget = target;
	fullStop=0;
	//set Time
        //std::cout << " initially generate segment time " <<std::endl;
	trajectory->autogenTimeSegment();
	// applyMinAltitude uses each segment's exact duration as its constraint
	// window, so it must run after segmentTimes is populated.
	trajectory->applyMinAltitude();
	trajectory->applyHorizontalLimits();
	segmentTimes = trajectory->segmentTimes;
	int count = 0;
	Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
	while (!(trajectory->checkSolved())){
		for(int i = 0; i < trajectory->segmentTimes.size();i++){
			trajectory->segmentTimes[i] +=retryStep;
		}
		Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
		count+=1;
		if(count == retryMax){
			std::cout << " could not plan flight" << std::endl;
			trajectory->overideSolve();
			trajectory->vertices = vertices_prev;
			trajectory->coeffSolved = coeffSolved_prev;
			trajectory->segmentTimes = segmentTimes_prev;
			return false;
		}
	}
	// Minimal execution time: shrink the allocation toward the dynamic limits
	// (peak vel/accel reach v_max/a_max) now that we have a feasible solve.
	// trajectory->minimizeTime(degreeOpt);
	segmentTimes = trajectory->segmentTimes;
	return true;
}



//Call this function to replan using the previous target 
bool ros_replan_utils::replan(int degreeOpt, double t_elap, double t_off){
	/*Eigen::Matrix4d Target;
	Target.setIdentity();*/
	int end = future_v.size()-1;
	waypoint last = future_v[end];
	Eigen::VectorXd pos;
	last.getPos(&pos);
	prevTarget(0,3) = pos(0);
	prevTarget(1,3) = pos(1);
	prevTarget(2,3) = pos(2);
	return replan(degreeOpt, t_elap,t_off, prevTarget);
}


//Overload the function in case you have a target detected
bool ros_replan_utils::replan(int degreeOpt, double t_elap, double t_off, Eigen::Matrix4d Target){
	//std::cout << "t elapsed " << t_elap <<std::endl;
        //std::cout << " start replan " <<std::endl;
	prevTarget = Target;
	//Calculate and set the next point
	if(curr_v == future_v.size()){
		//No need to replan the trajectory
		std::cout << "no need to replan the trajectory, " << std::endl;
		return false;
	}
	//Anticipate your current position 
	//double t_off = 0.0115;
	nav_msgs::msg::Odometry current_heading;
	
	//ros::spinOnce();
	bool use_odom =false; 
	double t0 = rclcpp::Clock().now().seconds() ;
	// The blend below decides how much of the measured tracking error is folded
	// into the new plan's start state -- see setOdomBlend(). anchorOdom==false
	// forces it to zero (pure predictive continuation).
	if(odom_l->getCurrOdom(&current_heading)){
		use_odom = true;
	}
	const double blend = anchorOdom ? odomBlend : 0.0;
	//std::cout << current_heading.pose.pose << std::endl;
	//while(!odom_l.getCurrOdom(&current_heading)){
	//	ros::spinOnce();
	//}
	//ros::Duration(t_elap*0.25).sleep();
	double t_wait  = rclcpp::Clock().now().seconds() - t0 ;
	waypoint start(current_heading);

	Eigen::MatrixXd point_info = trajectory->evalTraj(t_elap+t_wait);
	Eigen::MatrixXd point_info_2 = trajectory->evalTraj(t_elap+t_wait+t_off);
	Eigen::VectorXd pos(4);
	Eigen::VectorXd odom_pos(4);
	Eigen::VectorXd vel(4);
	Eigen::VectorXd odom_vel(4);
	Eigen::VectorXd accel(4);
	Eigen::VectorXd jerk(4);
	Eigen::VectorXd snap(4) ;
	start.getPos(&odom_pos);
	start.getVelo(&odom_vel);
	for(int i =0; i<4;i++){
		if(use_odom){
			// point_info is where the OLD plan expected the vehicle to be right
			// now, so (odom - point_info) is the tracking error. Fold in the
			// configured fraction of it: blend==1 reproduces the original
			// full re-pin onto odometry, blend==0 pure prediction.
			pos[i] = point_info_2(0,i) + blend*(odom_pos(i)-point_info(0,i));
			vel[i] = point_info_2(1,i) + blend*(odom_vel(i)-point_info(1,i));
		}
		else{
			pos[i] = point_info_2(0,i);
			vel[i] = point_info_2(1,i);
		}
		accel[i] = point_info_2(2,i);
		jerk[i] = point_info_2(3,i);
		snap[i] = point_info_2(4,i);
	}

	//std::cout << "Pose " << pos.transpose() <<std::endl;
	start.setPos(pos);
	start.setVel(vel);
	start.setAccel(accel);
	start.setJerk(jerk);
	start.setSnap(snap);
	// Must come after the setters -- each of them re-asserts its status entry.
	if(freeStartJerkSnap){
		relaxStartJerkSnap(&start);
	}
	//Save your Previous Trajectory in case we need to revert.
	std::vector<waypoint> vertices_prev =  trajectory->vertices;
	Eigen::MatrixXd coeffSolved_prev = trajectory->coeffSolved;

    std::vector<double> segmentTimes_prev=trajectory->segmentTimes;

	// Snapshot of THIS class's own bookkeeping (segmentTimes/curr_v), distinct
	// from trajectory's copy above. t_elap is cumulative -- total elapsed time
	// since the trajectory's begin was last set, not time since the last call
	// -- so segmentTimes[curr_v] -= t_elap must only ever be committed once,
	// on a call that ultimately succeeds. Every failure path below must restore
	// both of these, or a run of K consecutive failures (e.g. the same
	// constraint conflict refusing to solve every cycle) subtracts K
	// increasingly-large cumulative t_elap values instead of one, draining
	// segmentTimes[curr_v] far faster than real flight time and hitting
	// minSegTime -- and therefore giving up on replanning for good -- long
	// before the vehicle has actually gotten anywhere near the target.
	// curr_v matters just as much: if it is left incremented after a failed
	// attempt, the NEXT call's curr_v==future_v.size() check at the top of
	// this function fires immediately, so restoring segmentTimes alone would
	// just move where the permanent stop happens, not remove it.
	std::vector<double> member_segmentTimes_prev = segmentTimes;
	int curr_v_prev = curr_v;

	segmentTimes[curr_v]-=(t_elap);

	const double kSegMergeEps = 0.0015; // tiny slack when merging a consumed segment's time
	// Whether the real-time countdown on the LAST segment has run low enough
	// that, left alone, it can no longer support a solve. Historically this
	// meant giving up outright (see the !reallocateTime branch below). Under
	// reallocateTime it instead becomes the trigger for a rescue further down
	// -- once trajectory->vertices/segmentTimes are rebuilt in the structure
	// autogenTimeSegment() needs -- rather than an immediate failure. See
	// setReallocateTime() for why: the countdown only ever knows the ONE
	// straight-line estimate made at initialPlan(), so if the real, constrained
	// path takes longer, it eventually runs out while the vehicle is still far
	// from the target.
	bool lastSegmentLow = (curr_v == future_v.size()-1)&&(segmentTimes[curr_v] < minSegTime);
	if(lastSegmentLow && !reallocateTime){
		curr_v+=1;
		std::cout << "can't replan future_v.size() " << future_v.size() << " and segmentTimes[curr_v] " << segmentTimes[curr_v] << " while minSegTime " <<minSegTime << std::endl;
		segmentTimes = member_segmentTimes_prev;
		curr_v = curr_v_prev;
		return false;
	}

	if(!lastSegmentLow && segmentTimes[curr_v] < minSegTime){
		curr_v+=1;
		//consume  the previous segments time if it is less than the minimum segment time
		segmentTimes[curr_v]+=(segmentTimes[curr_v-1])+kSegMergeEps;
	}

	// FOV sampling must read the trajectory BEFORE trajectory->clearAll() below.
	// clearAll() -> clear_vertices() resets traj_valid to false, and evalTraj()
	// on a not-yet-solved trajectory hits its failure path, returning the
	// constant -1e10 for every field. The FOV block used to call evalTraj()
	// AFTER clearAll() (and before this cycle's own solve()), so pose_fov/
	// accelfov were ALWAYS -1e10 -- every replan, unconditionally -- feeding
	// garbage into genInEqFOV and producing an astronomically-scaled,
	// unsatisfiable bound. That's the actual reason the FOV joint solve always
	// failed, independent of coverage fraction, row count, or rank. Sample the
	// still-valid (previously-solved) trajectory here instead; the constraint
	// rows themselves are built further down, after the rebuild, since they
	// need the NEW segment structure for correct column placement.
	std::vector<double> fov_t_now;
	std::vector<Eigen::Vector4d> fov_pose;
	std::vector<Eigen::Vector3d> fov_accel;
	if(fovEnable){
		int rows = 8;
		float t_now_s = t_elap;
		float fullTime_s = 0.0;
		for(int i=0;i<segmentTimes.size();i++){ fullTime_s += segmentTimes[i]; }
		double coveredTime_s = std::min(fovCoverageFraction * fullTime_s, fullTime_s - 0.05);
		if(coveredTime_s < 0.0){ coveredTime_s = 0.0; }
		for(int k=0;k<rows;k++){
			double incr_time = coveredTime_s/rows;
			t_now_s += incr_time;
			Eigen::MatrixXd replan_pose = trajectory->evalTraj(t_now_s);
			Eigen::Vector4d pose_fov;
			Eigen::Vector3d accelfov;
			for(int i=0;i<4;i++){
				pose_fov[i] = replan_pose(0,i);
				if(i!=3){ accelfov[i] = replan_pose(2,i); }
			}
			fov_t_now.push_back(t_now_s);
			fov_pose.push_back(pose_fov);
			fov_accel.push_back(accelfov);
		}

		// [FOVDIAG][SWEEP] Independent of replan cadence and fov_coverage_fraction:
		// sample the SAME still-valid previous trajectory at a fixed sweep of times
		// from the true segment start (t=0), to see how fast the camera axis drifts
		// off the target as a function of time-since-start alone. t_elap (the floor
		// the coverage-fraction samples above are anchored to) is itself already
		// ~0.05-0.09s due to the replan cadence/lookahead -- this sweep checks
		// whether there's any usable window at all before that floor, i.e. whether
		// shrinking fov_coverage_fraction could ever help, or whether the
		// misalignment is already too far along by the earliest achievable sample.
		Eigen::Vector3d sweepTarget(Target(0,3), Target(1,3), Target(2,3));
		for(double tSweep = 0.0; tSweep <= 0.3 + 1e-9; tSweep += 0.02){
			Eigen::MatrixXd sw = trajectory->evalTraj(tSweep);
			Eigen::Vector4d swPose;
			Eigen::Vector3d swAcc;
			for(int i=0;i<4;i++){
				swPose[i] = sw(0,i);
				if(i!=3){ swAcc[i] = sw(2,i); }
			}
			double ang = trajectory->checkFovAxisAngle(sweepTarget, swPose, swAcc);
			// std::cout << "[FOVDIAG][SWEEP] t=" << tSweep << " angle_deg=" << ang << std::endl;
		}
	}

	//Clear the last trajectories
	trajectory->clearAll();
	trajectory->push_back(start);
	Eigen::Matrix4d targ_heading;
	for (int i = curr_v;i < future_v.size();i++){
		if(i == future_v.size()-1){
			Eigen::Vector4d lastPoint;
			lastPoint(0) = 	Target(0,3) ;
			lastPoint(1) = 	Target(1,3);
			lastPoint(2) =  Target(2,3);
			//std::cout << " Target Used " << lastPoint <<std::endl;
			waypoint last_waypoint(lastPoint);
			future_v[i] = last_waypoint;
		}
		Eigen::VectorXd pos_test;
		future_v[i].getPos(&pos_test);
		trajectory->push_back(future_v[i]);
	}

	trajectory->segmentTimes.clear();
	for (int i = curr_v;i < segmentTimes.size();i++){
		trajectory->segmentTimes.push_back(segmentTimes[i]);
	}
	if(reallocateTime && lastSegmentLow){
		// Rescue path only -- see lastSegmentLow and setReallocateTime() above.
		// The real-time countdown just used to build trajectory->segmentTimes
		// is exact and low-noise, so it stays the default source of truth every
		// cycle, unchanged from before reallocateTime existed. Only when it's
		// about to go infeasible do we ask autogenTimeSegment() for a fresh,
		// distance-based estimate from the anchor's ACTUAL current position --
		// and even then only take the max with the countdown, topping up a
		// failing budget rather than replacing a healthy one. (An earlier
		// version of this called autogenTimeSegment() unconditionally, every
		// cycle: near the target its output is highly sensitive to small
		// distance changes -- several seconds of allocated time per metre at
		// this v_max/a_max -- so ordinary tracking/vision noise made
		// consecutive cycles disagree wildly about how much time was left, and
		// each disagreement re-pinned calcPerchCond's terminal velocity/
		// acceleration match to a different deadline. That produced its own
		// terminal-maneuver whiplash, independent of the running-out-of-budget
		// failure this mechanism exists to fix.)
		std::vector<double> countdown = trajectory->segmentTimes;
		trajectory->autogenTimeSegment();
		for(size_t i = 0; i < trajectory->segmentTimes.size() && i < countdown.size(); i++){
			trajectory->segmentTimes[i] = std::max(countdown[i], trajectory->segmentTimes[i]);
		}
	}
	trajectory->applyMinAltitude();
	trajectory->applyHorizontalLimits();
	if(fullStop ==1){
		trajectory->setFullStop();
	}
	else{
		std::cout << " TARGET " << Target <<std::endl;
		if(!trajectory->calcPerchCond(Target)){
			//revert to previous trajectory, same as the retry-exhaustion path below
			trajectory->overideSolve();
			trajectory->vertices = vertices_prev;
			trajectory->coeffSolved = coeffSolved_prev;
			trajectory->segmentTimes =  segmentTimes_prev;
			// See the member_segmentTimes_prev/curr_v_prev comment above --
			// this class's own bookkeeping needs the same revert.
			segmentTimes = member_segmentTimes_prev;
			curr_v = curr_v_prev;
			std::cout << " could not plan flight: perch terminal condition exceeds "
			          << "the configured horizontal limits" << std::endl;
			return false;
		}
	}

	//SOLVE THE BASE PROBLEM FIRST -- NO JOINT CONSTRAINTS (FOV etc.) YET.
	// Time-allocation infeasibility (the perch band, eq.14, needing more slack
	// than the current segment time gives it) is the dominant failure mode and
	// has nothing to do with FOV. Solving/retrying the base problem first
	// guarantees a time budget that's actually enough for the perch/boundary
	// dynamics BEFORE FOV is layered on, instead of testing FOV at the
	// tightest (least likely to succeed) time allocation and then abandoning
	// it for the rest of the cycle the moment that first attempt fails for an
	// unrelated reason. add_joint_ineq_constr/add_joint_eq_constr are already
	// empty here (clearAll() -> clear_ineq() reset them above), so this
	// dispatches to the fast per-axis MTsolve, not SMsolve.
	int count = 0;
	Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
	while (!trajectory->checkSolved()){
		//std::cout << "REPLAN NEED MORE TIME" <<std::endl;
		// trajectory->segmentTimes here is rebuilt to start at index 0 (segments
		// curr_v..end), so indexing [curr_v] walked out of bounds as curr_v grew
		// -- a heap write past the vector end that intermittently corrupted the
		// trajectory and made it collapse. Add to every remaining segment (0-based).
		for(int i = 0; i < trajectory->segmentTimes.size(); i++){
			trajectory->segmentTimes[i] += retryStep;
		}
		coeffQP =  trajectory->solve(degreeOpt);
		count+=1;
		if(count == retryMax){
			//revert to previous trajectory
			trajectory->overideSolve();
			trajectory->vertices = vertices_prev;
			trajectory->coeffSolved = coeffSolved_prev;
			trajectory->segmentTimes =  segmentTimes_prev;
			// See the member_segmentTimes_prev/curr_v_prev comment above --
			// this class's own bookkeeping needs the same revert.
			segmentTimes = member_segmentTimes_prev;
			curr_v = curr_v_prev;
			std::cout << " could not plan flight" << std::endl;
			return false;
		}
	}
	if(count > 0){
		std::cout << "[replan] retries=" << count
		          << " -- base solve (no joint constraints) needed more time"
		          << std::endl;
	}
	// Base problem is solved and time-feasible. Keep it as the fallback in
	// case FOV (below) can't be satisfied at this same time allocation.
	Eigen::MatrixXd coeffSolved_base = trajectory->coeffSolved;
	std::vector<double> segmentTimes_base = trajectory->segmentTimes;

	//GENERATE FOV CONDITION -- pose/accel samples (fov_pose/fov_accel/fov_t_now)
	//were captured further up, BEFORE clearAll(), from the still-valid previous
	//trajectory. Build the actual constraint rows here, now that vertices/
	//segmentTimes reflect the NEW structure genInEqFOV needs for column placement
	//(and now that segmentTimes reflects the time growth from the base solve
	//above, so the segment/local-time lookup inside genInEqFOV lines up with
	//the time allocation FOV is actually about to be solved against).
	if(fovEnable){
		int rows = fov_t_now.size();
	    int coeffNum = trajectory->getPolyOrder()*(trajectory->numWaypoints() - 1)*trajectory->getDim() ;

		Eigen::VectorXd pose_last;
		future_v[future_v.size()-1].getPos(&pose_last);
		Eigen::Vector3d end_point= pose_last.block<3,1>(0,0);

		// genInEqFOV now returns MULTIPLE rows per sample (the FOV cone row plus
		// the jerk bound and eq.(9) trust-region rows) -- collect them all first
		// so rowsPerSample can be read off the first successful call instead of
		// being duplicated as a magic number here and in TrajBase.cpp (the kind
		// of manual size-sync that has already caused a rank-deficiency bug
		// once in this codebase).
		std::vector<QP_ineq_const> per_sample(rows);
		std::vector<bool> sample_ok(rows, false);
		int rowsPerSample = 0;
		for(int k=0;k<rows;k++){
			// fov_t_now[k] (not t_elap) is the actual time fov_pose[k]/fov_accel[k]
			// were sampled at -- genInEqFOV needs it to place the constraint's
			// basis rows at the matching segment/local-time instead of always
			// segment 0.
			if (trajectory->genInEqFOV(fov_t_now[k],end_point, fov_pose[k], fov_accel[k], &per_sample[k])){
				sample_ok[k] = true;
				rowsPerSample = per_sample[k].d.rows();
			}
		}

		QP_ineq_const full_ineq_constr;
		int totalRows = rows*rowsPerSample;
		full_ineq_constr.C = Eigen::MatrixXd::Zero(totalRows, coeffNum);
		// Sane, non-contradictory default (d <= f) so any row a sample failed to
		// populate stays a trivially-satisfied placeholder instead of an
		// unconditionally infeasible one.
		full_ineq_constr.f = Eigen::VectorXd::Constant(totalRows, 50000);
		full_ineq_constr.d = Eigen::VectorXd::Constant(totalRows, -50000);
		for(int k=0;k<rows;k++){
			if(!sample_ok[k]){ continue; }
			int base = k*rowsPerSample;
			full_ineq_constr.C.block(base, 0, rowsPerSample, coeffNum) = per_sample[k].C;
			full_ineq_constr.d.segment(base, rowsPerSample) = per_sample[k].d;
			full_ineq_constr.f.segment(base, rowsPerSample) = per_sample[k].f;
		}
		trajectory->push_joint_ineq_constr(full_ineq_constr);

		// One shot: try FOV layered on top of the already time-feasible base
		// solve (dispatches to SMsolve since add_joint_ineq_constr is now
		// non-empty). If it doesn't succeed at this segment time, fall back to
		// the base (no-FOV) solution instead of growing time further or
		// silently keeping FOV dropped for the rest of the cycle -- the old
		// behavior, which abandoned FOV the moment the very first (often
		// tightest-time) attempt failed, regardless of whether FOV itself was
		// even the cause.
		Eigen::MatrixXd coeffQP_fov = trajectory->solve(degreeOpt);
		// Remaining segment time at the point of this attempt -- to correlate
		// FOV solve outcome with how much time/distance is left in the flight
		// (e.g. does it succeed earlier with more time left, then start failing
		// as the remaining segment time shrinks near the end).
		double remainingTime = 0.0;
		for(size_t i = 0; i < trajectory->segmentTimes.size(); i++){
			remainingTime += trajectory->segmentTimes[i];
		}
		if(!trajectory->checkSolved()){
			std::cout << "[replan] FOV solve failed at base-feasible time -- "
			          << "falling back to the no-FOV solution"
			          << " remainingTime=" << remainingTime << std::endl;
			trajectory->clear_ineq();
			trajectory->clearCostVector();
			trajectory->overideSolve();
			trajectory->coeffSolved = coeffSolved_base;
			trajectory->segmentTimes = segmentTimes_base;
		}
		else{
			std::cout << "[replan] FOV solve succeeded"
			          << " remainingTime=" << remainingTime << std::endl;
		}
	}

	// Sync back any time growth from the base retry loop. `segmentTimes` (this
	// class's member) is a persistent, ABSOLUTE-indexed array across the whole
	// waypoint list -- curr_v indexes into it every call, and
	// trajectory->segmentTimes is only ever a transient 0-based slice
	// [curr_v..end] of it, rebuilt each call. The retry loop above grows
	// trajectory->segmentTimes but never writes that growth back, so
	// `segmentTimes` silently drifted shorter than what's really flying. Each
	// subsequent replan then computed its time budget (and the
	// `segmentTimes[curr_v] -= t_elap` bookkeeping) from a too-short number,
	// which can force another retry -- and occasionally the resulting
	// polynomial has to swing hard to fit the (wrongly) tight window, i.e. an
	// intermittent "collapse" a cycle or more after any retry happened. Map
	// the committed slice back onto the same absolute positions.
	for(size_t i = 0; i < trajectory->segmentTimes.size(); i++){
		segmentTimes[curr_v + i] = trajectory->segmentTimes[i];
	}
	//std::cout << "successful replanning" <<std::endl;
	return true;
}

void ros_replan_utils::setFOVEnable(bool in){
	fovEnable = in;
}

void ros_replan_utils::setFOVCoverageFraction(double frac){
	fovCoverageFraction = frac;
}

void ros_replan_utils::setReplanParams(double step, int maxRetries, double minSeg){
	retryStep = step;
	retryMax = maxRetries;
	minSegTime = minSeg;
}

void ros_replan_utils::setAnchorOdom(bool in){
	anchorOdom = in;
}

void ros_replan_utils::setOdomBlend(double in){
	odomBlend = in;
}

void ros_replan_utils::setFreeStartJerkSnap(bool in){
	freeStartJerkSnap = in;
}

void ros_replan_utils::setReallocateTime(bool in){
	reallocateTime = in;
}
