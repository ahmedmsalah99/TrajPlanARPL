#include <traj_gen/trajectory/TrajBase.h>
#include <math.h>
#include <algorithm>
using namespace std;

// Inscribed square: |vx|,|vy| <= limit/sqrt(2) guarantees the true horizontal
// magnitude sqrt(vx^2+vy^2) never exceeds the configured limit (equality only
// at the box's corners). Shared by applyHorizontalLimits() (which enforces
// this bound) and calcPerchCond() (which must check against this SAME bound,
// not the raw circular limit, since a pure-axis perch approach can have its
// full horizontal magnitude on one axis alone -- comparing against the wider
// circular limit there would pass a target whose terminal equality then
// directly contradicts this box's inequality on that axis, an empty feasible
// region that made OOQP hang instead of cleanly reporting infeasible).
static const double kInscribedSquareScale = 1.0 / sqrt(2.0);

TrajBase::TrajBase()
{
}



bool TrajBase::condCheck() {
	if(vertices.size() >1){
		return true;
	}
	return false;
}
TrajBase::~TrajBase()
{
}

void TrajBase::setVertices(std::vector<waypoint> w){
	vertices = w;
}

void TrajBase::setTimeSegment(std::vector<double> t)
{
	segmentTimes = t;
}

void TrajBase::equalTimeSegment(double duration)
{
	int count = 0;
	int length = vertices.size()-1;
	double dt = duration/length;
	std::vector<double> times(length);
	while (count < length) {
		count = count + 1;
		times[count-1] = dt;
	}
	segmentTimes = times;
}

float TrajBase::autogenTimeSegment()
{	
	float totalTime = 0.0;
	double v_max = limits[1];
	double a_max = limits[2];
	double magic_fabian_constant = 6.5;
	double yaw_dist = 0.0;
	const double yaw_rate = 2.0;
	int length = vertices.size()-1;
	std::vector<double> times(length);
	//double currTime = 0.0;
	for (int i = 0; i < length; i++) {
		Eigen::VectorXd currPoint;
		Eigen::VectorXd prevPoint;
		vertices[i].getPos(&prevPoint);
		vertices[i+1].getPos(&currPoint);
		//Give enough time for the yaw turns
		double yaw_time = abs(currPoint[3] -prevPoint[3])*2; 
		//Yaw angle does not matter for distance are optimal time
		currPoint[3] =0;
		prevPoint[3]=0;

		double distance = (currPoint - prevPoint).norm();
		double t = 1.1* distance / v_max * 2 *
				   (1.0 + magic_fabian_constant * v_max / a_max *
							  exp(-distance / v_max * 2));
   if(t<yaw_time){
			t = yaw_time;
		}
		times[i] = t;
		std::cout <<  " ALlocated time " << times[i] <<std::endl;
		totalTime+=t;
	  }
	segmentTimes = times;
	return totalTime;
}


void TrajBase::minimizeTime(int degreeOpt, int maxIters)
{
	double v_max = (limits.size() > 1) ? limits[1] : 0.0;
	double a_max = (limits.size() > 2) ? limits[2] : 0.0;
	if(!(v_max > 1e-6) || !(a_max > 1e-6)){
		return; // no dynamic limits configured -> nothing to optimize against
	}
	const int kSamples = 200;
	// The acceleration near the trajectory end can be pinned by a terminal
	// boundary condition (e.g. a perch's ~g free-fall impact accel), which is NOT
	// time-scalable and is allowed to exceed a_max. Ignore that approach window
	// for the a_max check, otherwise it would veto every shrink and the path stays
	// at the conservative autogen allocation. Velocity is checked over the whole
	// trajectory (the terminal velocity is small, so it isn't the peak).
	const double kTermAccelIgnoreFrac = 0.2; // ignore the last 20% of time for accel
	for(int it = 0; it < maxIters; it++){
		if(!checkSolved()){
			return; // need a solved trajectory to measure; keep what we have
		}
		double T = 0.0;
		for(size_t s = 0; s < segmentTimes.size(); s++){ T += segmentTimes[s]; }
		if(T <= 1e-6){ return; }

		// Peak |velocity| (whole trajectory) and |acceleration| (excluding the
		// terminal approach window) over x,y,z.
		double a_cutoff = T * (1.0 - kTermAccelIgnoreFrac);
		double v_peak = 0.0, a_peak = 0.0;
		for(int k = 0; k <= kSamples; k++){
			double t = T * (double)k / (double)kSamples;
			Eigen::MatrixXd st = evalTraj(t);
			double v = st.block(1, 0, 1, 3).norm();
			if(v > v_peak){ v_peak = v; }
			if(t <= a_cutoff){
				double a = st.block(2, 0, 1, 3).norm();
				if(a > a_peak){ a_peak = a; }
			}
		}

		// Uniform time scale s: velocity ~ 1/s, acceleration ~ 1/s^2.
		double s_v = v_peak / v_max;
		double s_a = sqrt(a_peak / a_max);
		double scale = std::max(s_v, s_a);

		// Only SHRINK (speed up) when there is slack under both limits. If the
		// trajectory is already at/over a limit (scale >= ~1 -- e.g. the fixed
		// free-fall acceleration of a perch terminal), leave it: we cannot go
		// faster without violating the limits.
		if(scale > 0.98){
			break;
		}
		scale = std::max(scale, 0.5); // cap the per-iteration shrink for stability

		std::vector<double> prev = segmentTimes;
		for(size_t s = 0; s < segmentTimes.size(); s++){ segmentTimes[s] *= scale; }
		solve(degreeOpt);
		if(!checkSolved()){
			segmentTimes = prev; // the shrink broke feasibility -> revert and stop
			solve(degreeOpt);
			break;
		}
	}
}


void TrajBase::push_back(waypoint w){
	if (w.getDim() != dim){
		//Thrown an exception. Waypoints must match the trajBase dimension
		std::cout << "An exception occurred. Traj dimension and waypoint must have the same dimension " << '\n';
		throw 20;
	}
	/*if(vertices.size()>=1){ yaw reversal code not to be considered for now 
		waypoint waypoint_prev =vertices[vertices.size()-1];
		Eigen::VectorXd last_pos;
		Eigen::VectorXd curr_pos;
		waypoint_prev.getPos(&last_pos);
		w.getPos(&curr_pos);
		while((curr_pos[3] - last_pos[3]) > 3.1415926){
			curr_pos[3] -= 2*3.1415926;
		}
		while((curr_pos[3] - last_pos[3]) < -3.1415926) {
			curr_pos[3]   += 2*3.1415926;
		} 
		w.setPos(curr_pos);      
	}*/
	vertices.push_back(w);
}

int TrajBase::getDim() {
	return dim;
}

waypoint TrajBase::getWaypoint(int t){
	if ( t >= vertices.size()){
		//Thrown an exception. Waypoints must match the trajBase dimension
		std::cout << "An exception occurred. Accessinig index of waypoints bigger than list  "<< '\n';
		std::cout << "num waypoints   "<< vertices.size()<<"Index access " <<t<< '\n';
		throw 20;
	}
	return vertices[t];
}
void TrajBase::setPolyOrder(int p){
	polyOrder = p;
}

int TrajBase::getPolyOrder() {
	return polyOrder;
}

int TrajBase::numWaypoints() {
	return vertices.size();
}

void TrajBase::setFullStop(){
	int numPoint = vertices.size();
	vertices[numPoint-1].setFullStop();
}


void TrajBase::push_ineq_constr(QP_ineq_const constr,int d){
	if(add_eq_constr.size()==0){
		for(int i=0;i<dim;i++){
			QP_ineq_const temp_ineq_constr;
			add_ineq_constr.push_back(temp_ineq_constr);
		}
	}
	QP_ineq_const temp_ineq_constr = add_ineq_constr[d];
	if(temp_ineq_constr.d.rows()!=0){
		//Verticall stack the two objects
		int rows = temp_ineq_constr.d.rows()+constr.d.rows();
		int cols = constr.C.cols();
		Eigen::VectorXd d = Eigen::VectorXd::Zero(rows);
		Eigen::VectorXd f = Eigen::VectorXd::Zero(rows);
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(rows,cols);
		//vertical stacking equality constraints 
		d.head(temp_ineq_constr.d.rows()) = temp_ineq_constr.d;
		d.tail(constr.d.rows()) = constr.d;
		f.head(temp_ineq_constr.f.rows()) = temp_ineq_constr.f;
		f.tail(constr.f.rows()) = constr.f;
		C.block(0,0,temp_ineq_constr.d.rows(),cols) = temp_ineq_constr.C;
		C.block(0,temp_ineq_constr.d.rows(),constr.d.rows(),cols) = constr.C;
		temp_ineq_constr.d = d;
		temp_ineq_constr.f = f;
		temp_ineq_constr.C = C;
	}
	else{
		temp_ineq_constr = constr;
	}
	add_ineq_constr[d] = temp_ineq_constr;
}
void TrajBase::push_eq_constr(QP_constraint constr, int d){
	if(add_eq_constr.size()==0){
		for(int i=0;i<dim;i++){
			QP_constraint temp_eq_constr;
			add_eq_constr.push_back(temp_eq_constr);
		}
	}
	QP_constraint temp_eq_constr = add_eq_constr[d];
	if(temp_eq_constr.b.rows()!=0){
		//Verticall stack the two objects
		int rows = temp_eq_constr.b.rows()+constr.b.rows();
		int cols = constr.a.cols();
		Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows,cols);
		//vertical stacking equality constraints 
		b.head(temp_eq_constr.b.rows()) = temp_eq_constr.b;
		b.tail(constr.b.rows()) = constr.b;
		A.block(0,0,temp_eq_constr.b.rows(),cols) = temp_eq_constr.a;
		A.block(0,temp_eq_constr.b.rows(),constr.b.rows(),cols) = constr.a;
		temp_eq_constr.b = b;
		temp_eq_constr.a = A;
	}
	else{
		temp_eq_constr = constr;
	}
	add_eq_constr[d] = temp_eq_constr;

}

void TrajBase::push_joint_ineq_constr(QP_ineq_const constr){
	add_joint_ineq_constr = constr;
}

void TrajBase::push_joint_eq_constr(QP_constraint constr){
	if(add_joint_eq_constr.b.rows()==0){
		add_joint_eq_constr = constr;
	}
	else{
		int rows = add_joint_eq_constr.b.rows()+constr.b.rows();
		int cols = constr.a.cols();
		Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows,cols);
		//vertical stacking equality constraints 
		b.head(add_joint_eq_constr.b.rows()) = add_joint_eq_constr.b;
		b.tail(constr.b.rows()) = constr.b;
		A.block(0,0,add_joint_eq_constr.b.rows(),cols) = add_joint_eq_constr.a;
		A.block(0,add_joint_eq_constr.b.rows(),constr.b.rows(),cols) = constr.a;
		add_joint_eq_constr.b = b;
		add_joint_eq_constr.a = A;
	
	}
}

void TrajBase::clear_ineq(){
	add_ineq_constr.clear();
	QP_ineq_const temp_ineq_constr;
	add_joint_ineq_constr = temp_ineq_constr;
}

void TrajBase::clear_eq(){
	add_eq_constr.clear();
	QP_constraint temp_eq_constr;
	add_joint_eq_constr = temp_eq_constr;
}

void TrajBase::clear_vertices(){
	for (int i = 0; i < traj_valid.size();i++){
		traj_valid[i] = false;
	}

	vertices.clear();
}

void TrajBase::clearAll(){
	segmentTimes.clear();
	clearCostVector();
	clear_eq();
	clear_ineq();
	clear_vertices();
}

void TrajBase::clearCostVector(){
	useCostVector=false;
}

bool TrajBase::checkSolved(){
	if(traj_valid.size()==0){
		return false;
	}
	for(int i =0;i<traj_valid.size();i++){
		if (!traj_valid[i]){
			return false;
		}
	}
	return true;
}

bool TrajBase::isMinAltitudeZBlocking(){
	if(!minAltitudeEnabled || traj_valid.size() <= 2){
		return false;
	}
	if(traj_valid[2]){
		return false;
	}
	for(size_t i = 0; i < traj_valid.size(); i++){
		if(i != 2 && !traj_valid[i]){
			return false; // some other dimension is also failing -- not purely a floor conflict
		}
	}
	// Two different failure shapes both show up as "only z fails," and they
	// need OPPOSITE responses. If the anchor already satisfies the floor,
	// this is the overshoot case (fixed terminal velocity forces a swing
	// through the floor mid-flight) -- shrinking helps, confirmed in the
	// field. If the anchor is already BELOW the floor, the vehicle
	// genuinely needs real time to climb up to it -- shrinking here only
	// makes an already-insufficient time budget worse. Confirmed in the
	// field: a cycle with anchor z=-1.658 (well below floor=-8.26935) kept
	// failing while segment time was shrunk from 4.86s down to 4.06s,
	// exactly the wrong direction for a genuine climb-time shortage.
	if(vertices.empty()){
		return false;
	}
	Eigen::VectorXd anchorPos;
	if(vertices[0].getPos(&anchorPos) != 1 || anchorPos.rows() <= 2){
		return false;
	}
	return anchorPos(2) <= lastMinAltitudeFloorZ;
}

void TrajBase::overideSolve(){
	for(int i =0;i<traj_valid.size();i++){
		if (traj_valid[i]!=1){
			traj_valid[i] = true;
		}
	}
}

//Given a Homogenous Transform and a forward Velocity how to convert to this system
bool TrajBase::calcPerchCond(Eigen::Matrix4d H){
	int numPoint = vertices.size();

	// Surface normal s3 (outward), in the world frame = 3rd column of the target
	// rotation. e3 is world-up; in NED (z-down) that is (0,0,-1).
	Eigen::Vector3d s3(H(0,2), H(1,2), H(2,2));
	std::cout << "[DIAG] calcPerchCond s3 (goal normal used) = "
            << s3 << std::endl;
	Eigen::Vector3d e3(0.0, 0.0, -1.0);
	double cos_incl = s3.dot(e3);                       // = cos(inclination)
	double sin_incl = sqrt(s3(0)*s3(0) + s3(1)*s3(1));  // horizontal length of s3 = sin(inclination)
	std::cout << "sin_incl " << sin_incl <<std::endl;
	std::cout << "cos_incl " << cos_incl <<std::endl;
	// Guard: an upside-down / overhanging pad (normal points below horizontal)
	// would require thrusting downward, which a quadrotor cannot do. Warn and
	// clamp to a vertical surface rather than emit an infeasible target.
	if(cos_incl < 0.0){
		std::cout << "WARNING calcPerchCond: target normal points downward (s3.e3 < 0); "
		          << "clamping to a vertical surface." << std::endl;
		s3(2) = 0.0;
		double h = s3.head<2>().norm();
		if(h > 1e-9){ s3.head<2>() /= h; }
		cos_incl = 0.0;
		sin_incl = 1.0;
	}

	// Terminal specific-thrust magnitude (eq. 12): scaled from 0 (flat) up to
	// maxInclinationAccel (vertical) by the inclination factor sin(inclination).
	double force = 9.8 + maxInclinationAccel * sin_incl;
	std::cout << "force is " << force << " sin_incl "<< sin_incl << std::endl;
	// Impact velocity built in the surface frame so it generalizes to any
	// orientation: a component INTO the surface (-s3, magnitude impactNormalVel = vS1)
	// plus a component ALONG the surface, down-slope (t_up, magnitude impactSlideVel = vS3).
	Eigen::Vector4d impVel = Eigen::Vector4d::Zero();
	Eigen::VectorXd finalAccel = Eigen::VectorXd::Zero(4);
	double incl_angle = atan2(sin_incl, cos_incl); // inclination angle in [0, pi/2]
	if(incl_angle >= minPitch){
		Eigen::Vector3d t_up = e3 - cos_incl * s3;  // world-up projected into the surface plane
		double tn = t_up.norm();
		if(tn > 1e-9){ t_up /= tn; }
		Eigen::Vector3d v = impactNormalVel * (-s3) + impactSlideVel * t_up;
		impVel(0) = v(0);
		impVel(1) = v(1);
		impVel(2) = v(2);
		impVel(3) = 0.0; // yaw rate

		// Terminal acceleration so that b3 = s3 at contact (eq. 12): xdd = force*s3 - g*e3.
		// Written via e3 so it follows the frame's up-vector (in NED this adds +g on z).
		finalAccel[0] = s3(0) * force - 9.81 * e3(0);
		finalAccel[1] = s3(1) * force - 9.81 * e3(1);
		finalAccel[2] = s3(2) * force - 9.81 * e3(2);
	}
	// else: too flat to need perch-style contact physics (pressing the thrust
	// axis into an inclined surface) -- just arrive level and at rest.
	// finalAccel/impVel both stay zero (hover, not eq.12's free-fall/zero-thrust
	// degenerate case, which is what "force*s3 - g*e3" collapses to as
	// force -> 0 for a flat surface -- that's an artifact of applying the
	// inclined-perch formula outside where it's meant to apply, not an
	// intended "soft landing" behavior).
	std::cout << "final acceleration " << finalAccel[0] << std::endl;
	std::cout << "final acceleration " << finalAccel[1] << std::endl;
	std::cout << "final acceleration " << finalAccel[2] << std::endl;
	std::cout << "final acceleration " << finalAccel << std::endl;

	// Reject up front if the perch physics (fixed by the target geometry and
	// maxInclinationAccel/impactNormalVel/impactSlideVel) demand more horizontal
	// acceleration or impact velocity than applyHorizontalLimits() will actually
	// allow, instead of pushing constraints that fight each other and letting
	// the QP fail on the final segment. Must check against the SAME per-axis
	// inscribed-square bound the box enforces (limit/sqrt(2) on ax and ay
	// independently) -- NOT the wider raw circular limit: a pure-axis perch
	// approach (e.g. s3 = (-1,0,0)) can put its entire horizontal magnitude on
	// one axis alone, which the box constrains to limit/sqrt(2), not limit.
	// Checking the combined magnitude against the raw limit let exactly that
	// case slip through (13.81 < 15 raw, but > 15/sqrt(2)=10.6 per-axis),
	// leaving a terminal equality that directly contradicts the box's
	// inequality on the same axis -- an empty feasible region that made OOQP
	// hang instead of cleanly reporting infeasible.
	double accelBound = horizAccelLimit * kInscribedSquareScale;
	double velBound = horizVelLimit * kInscribedSquareScale;
	if(horizAccelLimit > 0.0 &&
	   (fabs(finalAccel[0]) > accelBound || fabs(finalAccel[1]) > accelBound)){
		std::cout << "[HORIZ_LIMIT] REJECTED plan: perch terminal horizontal acceleration ("
		          << finalAccel[0] << ", " << finalAccel[1]
		          << " m/s^2) exceeds the per-axis bound (+/-" << accelBound
		          << " m/s^2, from horiz_accel_limit=" << horizAccelLimit
		          << "). Raise horiz_accel_limit, lower max_inclination_accel, "
		          << "or use a less steep target." << std::endl;
		return false;
	}
	if(horizVelLimit > 0.0 &&
	   (fabs(impVel(0)) > velBound || fabs(impVel(1)) > velBound)){
		std::cout << "[HORIZ_LIMIT] REJECTED plan: perch impact horizontal velocity ("
		          << impVel(0) << ", " << impVel(1)
		          << " m/s) exceeds the per-axis bound (+/-" << velBound
		          << " m/s, from horiz_vel_limit=" << horizVelLimit
		          << "). Raise horiz_vel_limit or lower "
		          << "impact_normal_vel/impact_slide_vel." << std::endl;
		return false;
	}

	vertices[numPoint-1].setVel(impVel);
	vertices[numPoint-1].setAccel(finalAccel);
	//vertices[numPoint-1].setJerk(Eigen::VectorXd::Zero(4));
	//vertices[numPoint-1].setSnap(Eigen::VectorXd::Zero(4));

	waypoint_ineq_const ineq_const_a;
	Eigen::Vector4d numIneqCon = Eigen::VectorXd::Zero(4);
    std::cout << "done acceleartion " <<std::endl;
	// Eq. (14): hold the acceleration within a (1+q) tolerance band of the
	// terminal target finalAccel = (alpha*s3 - g*e3) over the approach window,
	// so the contact attitude (b3 ~ s3) is largely established before impact
	// (the quadrotor does not bounce off or clip the pad). Applied component-wise
	// on x, y, z (decoupled per axis); yaw (index 3) is left free.
	//   q     : magnitude tolerance / band width                       (eq. 14)
	//   t_k   : window before impact, sampled at dt (= timeOffset)      (eq. 14)
	//   q_eps : small additive slack so the band never collapses to a point when
	//           a component of finalAccel is ~0 (e.g. the lateral axis at a 90 deg
	//           target); this is the generalization of the old hard-coded +/-0.2.
	// q, t_k, q_eps are configurable via setPerchBand().
	double q = perchBandTol;
	double t_k = perchWindow;
	double q_eps = perchBandEps;
	Eigen::Vector4d lower_a = Eigen::VectorXd::Zero(4);
	Eigen::Vector4d upper_a = Eigen::VectorXd::Zero(4);
	for (int k = 0; k < 3; k++) {
		double a0 = finalAccel[k];
		double a1 = (1.0 + q) * finalAccel[k];
		lower_a[k] = std::min(a0, a1) - q_eps;
		upper_a[k] = std::max(a0, a1) + q_eps;
		numIneqCon[k] = 1;
	}

	//std::cout << "Set Accel " << finalAccel <<std::endl;

	ineq_const_a.derivOrder = 2;
	ineq_const_a.timeOffset = t_k;
	ineq_const_a.lower = lower_a;
	ineq_const_a.upper = upper_a;
	ineq_const_a.InEqDim = numIneqCon;
	/*
	std::cout << "Inequality on and off " << numIneqCon <<std::endl;
	
	
	std::cout << "Lower V " << lower_v <<std::endl;
	std::cout << "upper V " << upper_v <<std::endl;
	
	std::cout << "Lower A " << lower_a <<std::endl;
	std::cout << "upper A " << upper_a <<std::endl;
	*/
    std::cout << "PUSH CONSTRAINTS" <<std::endl;
	vertices[numPoint-1].ineq_constraint.push_back(ineq_const_a);
	//if(constrainV){
		//vertices[numPoint-1].ineq_constraint.push_back(ineq_const_v);
	//}
      //  std::cout << " push constraint done" <<std::endl;
	//set no velocity if our pitch angle is low
	return true;
}

//Simple calculation for X aligned perching
bool TrajBase::calcPerchCond(double pitch){
	Eigen::Matrix4d H;
	H(0,2) = -1*sin(pitch);
	H(1,2) = 0;
	H(2,2) = cos(pitch);
	return calcPerchCond(H);
}

void TrajBase::setPerchParams(double maxInclAccel, double normalVel, double slideVel, double minIncl){
	maxInclinationAccel = maxInclAccel;
	impactNormalVel = normalVel;
	impactSlideVel = slideVel;
	minPitch = minIncl;
}

void TrajBase::setPerchBand(double q, double window, double eps){
	perchBandTol = q;
	perchWindow = window;
	perchBandEps = eps;
}

void TrajBase::setFovCamTilt(double tilt){
	fovCamTilt = tilt;
}

void TrajBase::setFovMargin(double m){
	fovMargin = m;
}

void TrajBase::setFovRh(double r_h){
	fovRh = r_h;
}

void TrajBase::setFovTrustRegion(double pos, double acc, double yaw){
	fovTrustPos = pos;
	fovTrustAcc = acc;
	fovTrustYaw = yaw;
}

void TrajBase::setMinAltitude(bool enable, double minAlt, double aboveTarget, double releaseDist, double releaseS){
	minAltitudeEnabled = enable;
	minAltitude = minAlt;
	minAltitudeAboveTarget = aboveTarget;
	minAltitudeReleaseDist = releaseDist;
	minAltitudeReleaseS = releaseS;
}

void TrajBase::applyMinAltitude(){
	if(!minAltitudeEnabled){
		return;
	}
	// Requires segmentTimes to already be sized to the current vertices (one
	// entry per segment): the segment duration is used to size the window below
	// (see the interior-only rationale further down), and keeps
	// genInEqConstraint's row-count estimate (based on the raw, un-clamped
	// timeOffset, not the actual sampled window) proportional to the real
	// segment length. Call this after autogenTimeSegment()/segmentTimes is set
	// for this plan.
	if(segmentTimes.size() != vertices.size() - 1){
		std::cout << "[MIN_ALTITUDE] SKIPPED: segmentTimes not sized to vertices yet"
		          << " (segmentTimes=" << segmentTimes.size()
		          << " vertices=" << vertices.size()
		          << ") -- call after segment times are set for this plan"
		          << std::endl;
		return;
	}

	// Effectively unbounded on the unconstrained dimensions/direction; the QP
	// needs a finite box (d <= Cx <= f), so use a bound far outside any
	// physically reachable position instead of true infinity. Keep this modest
	// (not e.g. 1e6): positions here are O(1-10) m, and mixing a wildly
	// different scale into the same QP as the real O(1-10) constraints
	// ill-conditions the interior-point solver, which can fail to converge
	// even when the true feasible region is fine (matches
	// kEmptyIneqBound=0.1's small-and-scaled convention elsewhere).
	const double kUnbounded = 100.0;

	// NED: altitude above the world origin = -z, so a minimum altitude is an
	// UPPER bound on z (z <= -minAltitude). Only z (index 2) is constrained;
	// x, y, yaw are left free. Each vertex i (i>=1) anchors its constraint to
	// the segment ending at it (segmentTimes[i-1]).
	// Where the floor sits, in NED z (smaller = higher).
	double floorZ = -minAltitude;
	// The final vertex IS the target. Needed both for a target-relative floor
	// (below) and, unconditionally, for the per-segment release logic further
	// down -- so compute it once regardless of minAltitudeAboveTarget.
	Eigen::VectorXd targetPos;
	bool haveTargetPos = (vertices.back().getPos(&targetPos) == 1 && targetPos.rows() > 2);
	if(minAltitudeAboveTarget > 0.0){
		// Relative to the perch target rather than the world origin. A visual
		// target moves between plans, so no absolute figure could track it --
		// this is only discovered at solve time, but the final vertex's
		// position is already known here.
		if(haveTargetPos){
			floorZ = targetPos(2) - minAltitudeAboveTarget;
		}
		else{
			std::cout << "[MIN_ALTITUDE] final waypoint has no position constraint -- "
			          << "falling back to the absolute floor" << std::endl;
		}
	}

	for(size_t i = 1; i < vertices.size(); i++){
		double segT = segmentTimes[i-1];

		// Per-segment, plan-relative release. This used to be a single
		// per-cycle yes/no gate checked against the CURRENT anchor position:
		// if the vehicle started this cycle more than minAltitudeReleaseDist
		// away horizontally, the floor was applied across the WHOLE segment,
		// full stop -- with no relaxation as the segment's own plan carries
		// it closer to the target over time. Since genInEqConstraint only
		// ever leaves a single ~ineqSampleDt gap unconstrained right before
		// the segment's end (see its endOffset handling), that meant the
		// ENTIRE required descent below the floor (e.g. 0.4m, in the field
		// case that motivated this) had to happen inside that one tiny
		// window -- confirmed via [MIN_ALTITUDE_DIAG]: z failed EVERY single
		// retry, regardless of added time or sample rows, because it isn't a
		// tightness problem, it's a shape the solver can never produce.
		//
		// Instead, use this segment's own plan -- a straight-line, constant-
		// rate interpolation between its start and end waypoints, the best
		// estimate available before solving -- to find WHEN along it the
		// horizontal distance to the target first drops to
		// minAltitudeReleaseDist, and only constrain the portion of the
		// segment BEFORE that point. The floor then relaxes gradually as the
		// plan approaches, instead of all at once at the very end.
		double endOffsetForSeg = 0.0; // 0 == floor applies right up to the segment end
		bool skipSeg = false;
		if(minAltitudeReleaseDist > 0.0 && haveTargetPos && segT > 1e-9){
			Eigen::VectorXd startPos, endPos;
			bool haveStart = (vertices[i-1].getPos(&startPos) == 1 && startPos.rows() > 1);
			bool haveEnd = (vertices[i].getPos(&endPos) == 1 && endPos.rows() > 1);
			if(haveStart && haveEnd){
				double sx = startPos(0) - targetPos(0), sy = startPos(1) - targetPos(1);
				double startDist = sqrt(sx*sx + sy*sy);
				if(startDist <= minAltitudeReleaseDist){
					// Already within release distance at the segment's own
					// start -- lift the floor for this whole segment.
					skipSeg = true;
					std::cout << "[MIN_ALTITUDE] segment " << i << " SKIPPED: starts "
					          << startDist << "m horizontally from the target (<= release "
					          << minAltitudeReleaseDist << "m)" << std::endl;
				}
				else{
					double ex = endPos(0) - targetPos(0), ey = endPos(1) - targetPos(1);
					// Horizontal distance-to-target along the straight-line plan,
					// as a function of segment-local time t in [0,segT]:
					// pos(t) = start + (end-start)*t/segT. dist(t)^2 is a
					// quadratic in t -- solve for where it equals
					// minAltitudeReleaseDist^2.
					double dx = (ex - sx) / segT, dy = (ey - sy) / segT; // rate vector
					double a = dx*dx + dy*dy;
					double b = 2.0*(sx*dx + sy*dy);
					double c = sx*sx + sy*sy - minAltitudeReleaseDist*minAltitudeReleaseDist;
					double tCross = -1.0;
					if(a > 1e-9){
						double disc = b*b - 4.0*a*c;
						if(disc >= 0.0){
							double sq = sqrt(disc);
							double t1 = (-b - sq) / (2.0*a);
							double t2 = (-b + sq) / (2.0*a);
							// First entry into the release radius, moving forward in time.
							double tEnter = std::min(t1, t2);
							if(tEnter > 0.0 && tEnter <= segT){
								tCross = tEnter;
							}
						}
					}
					if(tCross >= 0.0){
						endOffsetForSeg = segT - tCross;
						std::cout << "[MIN_ALTITUDE] segment " << i << ": floor applies for the first "
						          << tCross << "s (until ~" << minAltitudeReleaseDist
						          << "m horizontally from the target), released for the remaining "
						          << endOffsetForSeg << "s of " << segT << "s total" << std::endl;
					}
					else{
						std::cout << "[MIN_ALTITUDE] segment " << i << ": never gets within "
						          << minAltitudeReleaseDist << "m horizontally within this segment's own plan -- "
						          << "floor applied across the whole " << segT << "s" << std::endl;
					}
				}
			}
		}
		// Also release the last minAltitudeReleaseS seconds of the FINAL
		// segment unconditionally, regardless of horizontal distance -- the
		// original design, before minAltitudeReleaseDist existed. Distance-
		// only release gives the VERTICAL/climb dimension no relief at all
		// while the vehicle is still far away horizontally: confirmed in the
		// field, an anchor near the ground climbing toward a target ~8m up
		// (still far horizontally) had the floor rigidly enforced across
		// virtually the whole segment with zero terminal give, and the
		// natural climb never got within ~0.35m of the floor at ANY segment
		// duration tried -- not a timing problem, a missing release. Take
		// whichever of the two release conditions gives MORE release (the
		// larger endOffset), so both apply together rather than one
		// replacing the other.
		if(minAltitudeReleaseS > 0.0 && i == vertices.size() - 1 && !skipSeg){
			double timeEndOffset = std::min(minAltitudeReleaseS, segT);
			if(timeEndOffset > endOffsetForSeg){
				std::cout << "[MIN_ALTITUDE] segment " << i << ": also releasing the final "
				          << timeEndOffset << "s unconditionally (min_altitude_release_s="
				          << minAltitudeReleaseS << ")" << std::endl;
				endOffsetForSeg = timeEndOffset;
			}
		}
		if(skipSeg){
			continue;
		}

		waypoint_ineq_const c;
		c.derivOrder = 0; // position
		// spanFromStart, not a frozen timeOffset computed from segmentTimes
		// here -- the retry loop in replan()/initialPlan() grows segmentTimes
		// AFTER this runs, and a frozen timeOffset can't track that (see the
		// struct member comment): the window's width stays fixed to whatever
		// segmentTimes was at THIS instant, so as retries grow the real
		// segment time, the window silently slides forward and stops
		// covering the segment's start instead of actually covering more of
		// it. spanFromStart re-derives the window from the CURRENT segment
		// time every time it's sampled, so growth genuinely extends coverage.
		//
		// endOffset above releases the tail once the plan's own straight-line
		// estimate says the segment is within minAltitudeReleaseDist of the
		// target -- genInEqConstraint's window is also strict-less-than at
		// the very start instant (the previous vertex's own equality
		// constraint, which may legitimately sit below the floor), so both
		// endpoints of every segment end up excluded either way.
		c.spanFromStart = true;
		c.endOffset = endOffsetForSeg;
		c.lower = Eigen::Vector4d::Constant(-kUnbounded);
		c.upper = Eigen::Vector4d::Constant(kUnbounded);
		c.upper(2) = floorZ;
		c.InEqDim = Eigen::Vector4d::Zero();
		c.InEqDim(2) = 1;
		vertices[i].addInEqualityConstraint(c);
	}
	std::cout << "[MIN_ALTITUDE] applied: z <= " << floorZ
	          << (minAltitudeAboveTarget > 0.0 ? " (target-relative)" : " (absolute)")
	          << " across " << (vertices.size() - 1) << " segment(s), release distance "
	          << minAltitudeReleaseDist << "m" << std::endl;
	// Diagnostic only -- see the member comment.
	lastMinAltitudeFloorZ = floorZ;
}

void TrajBase::setHorizontalLimits(double velLimit, double accelLimit, double jerkLimit){
	horizVelLimit = velLimit;
	horizAccelLimit = accelLimit;
	horizJerkLimit = jerkLimit;
}

void TrajBase::applyHorizontalLimits(){
	if(horizVelLimit <= 0.0 && horizAccelLimit <= 0.0 && horizJerkLimit <= 0.0){
		return;
	}
	// Same requirement/rationale as applyMinAltitude(): the window below is
	// sized from each segment's real duration.
	if(segmentTimes.size() != vertices.size() - 1){
		std::cout << "[HORIZ_LIMIT] SKIPPED: segmentTimes not sized to vertices yet"
		          << " (segmentTimes=" << segmentTimes.size()
		          << " vertices=" << vertices.size()
		          << ") -- call after segment times are set for this plan"
		          << std::endl;
		return;
	}

	const double kUnbounded = 100.0; // see applyMinAltitude() for rationale
	// Same interior-only sampling window as applyMinAltitude(): excludes both
	// segment endpoints, which may carry their own equality constraints (e.g.
	// the live start vertex's velocity/acceleration from odom) that this box
	// could otherwise conflict with and make the QP infeasible on every solve.

	// spanFromStart, not a frozen timeOffset computed from segmentTimes here --
	// see the struct member comment (and applyMinAltitude(), which hit this
	// same bug): a timeOffset snapshotted now can't track the retry loop
	// growing segmentTimes afterwards, so the window would silently stop
	// covering the segment's start as retries grow it instead of genuinely
	// covering more of the segment.
	auto pushBox = [&](size_t i, int derivOrder, double limit){
		double boxLimit = limit * kInscribedSquareScale;
		waypoint_ineq_const c;
		c.derivOrder = derivOrder;
		c.spanFromStart = true;
		c.lower = Eigen::Vector4d::Constant(-kUnbounded);
		c.upper = Eigen::Vector4d::Constant(kUnbounded);
		c.lower(0) = -boxLimit; c.upper(0) = boxLimit; // x
		c.lower(1) = -boxLimit; c.upper(1) = boxLimit; // y
		c.InEqDim = Eigen::Vector4d::Zero();
		c.InEqDim(0) = 1; c.InEqDim(1) = 1;
		vertices[i].addInEqualityConstraint(c);
	};

	for(size_t i = 1; i < vertices.size(); i++){
		if(horizVelLimit > 0.0){ pushBox(i, 1, horizVelLimit); }
		if(horizAccelLimit > 0.0){ pushBox(i, 2, horizAccelLimit); }
		if(horizJerkLimit > 0.0){ pushBox(i, 3, horizJerkLimit); }
	}
	std::cout << "[HORIZ_LIMIT] applied: sqrt(vx^2+vy^2)<=" << horizVelLimit
	          << " sqrt(ax^2+ay^2)<=" << horizAccelLimit
	          << " sqrt(jx^2+jy^2)<=" << horizJerkLimit
	          << " across " << (vertices.size() - 1) << " segment(s)" << std::endl;
}

void TrajBase::setIneqSampleDt(double dt){
	ineqSampleDt = dt;
}

/*Virtual Stubs*/



bool TrajBase::genInEqFOV(double t_now, Eigen::Vector3d target, 	Eigen::Vector4d pose,	Eigen::Vector3d accel , QP_ineq_const * constraint)
{
        //std::cout << " strat ineq Fov" <<std::endl;
        int coeffNum = (vertices.size() - 1) *  polyOrder;
	// Row layout: 0 = FOV cone constraint (eq.7-8), 1-3 = jerk bound, 4-6 =
	// eq.(9) position trust region (x,y,z), 7-9 = eq.(9) acceleration trust
	// region (x,y,z), 10 = eq.(9) yaw trust region.
	const int kNumRows = 11;
	QP_ineq_const ineq_const;
	ineq_const.d= Eigen::VectorXd::Zero(kNumRows);
	ineq_const.f= Eigen::VectorXd::Zero(kNumRows);
	ineq_const.C = Eigen::MatrixXd::Zero(kNumRows, coeffNum*dim);
	// 7 rows: [posx,posy,posz,accx,accy,accz,yaw] -- matches derivative_FOV's
	// 7-variable Jacobian (eq.7-8 differentiates w.r.t. position, acceleration,
	// AND yaw; the old 6-variable version left yaw un-optimizable).
	Eigen::MatrixXd const_conv =  Eigen::MatrixXd::Zero(7, coeffNum*dim);
	//std::cout << " fov start " <<std::endl;
        FOV_constraint fov(pose, accel, fovCamTilt, fovRh);
        //std::cout << " start derovative fov " <<std::endl;
	fov_constr constr = fov.derivative_FOV(target);
	//Else we do the calcualtion to add the full inequality constriant
	// FOV constraint: keep the linearized margin g_lin(s) >= fovMargin, i.e. a LOWER
	// bound on grad(g)·s. constr.diff = g(x0) + grad·x0, so g(x0) = constr.diff - grad·x0,
	// and the wanted bound is  grad·s >= grad·x0 - g(x0) + fovMargin.
	Eigen::VectorXd x0_state(7);
	x0_state << pose[0], pose[1], pose[2], accel[0], accel[1], accel[2], pose[3];
	double grad_dot_x0 = (constr.Jacobian * x0_state)(0);
	double g_x0 = constr.diff - grad_dot_x0;            // = diff(x0), the current margin
	ineq_const.d(0) = grad_dot_x0 - g_x0 + fovMargin;   // lower bound
	ineq_const.f(0) = 50000;                            // ~ +inf

	// Locate which segment t_now falls in and the LOCAL time within that
	// segment (same walk evalTraj uses), so the basis rows below are
	// expressed at the same point (pose, accel) was sampled at/linearized
	// around -- instead of always being pinned to segment 0 at whatever raw
	// time was passed in.
	int numSeg = segmentTimes.size();
	int segIdx = 0;
	double localTime = 0.0;
	if(numSeg > 0){
		localTime = std::max(0.0, t_now);
		while(segIdx < numSeg-1 && segmentTimes[segIdx] < localTime){
			localTime -= segmentTimes[segIdx];
			segIdx += 1;
		}
		if(localTime > segmentTimes[segIdx]){
			localTime = segmentTimes[segIdx];
		}
	}

	Eigen::MatrixXd row_pos = basis(localTime, 0).transpose();
	Eigen::MatrixXd row_acc = basis(localTime, 2).transpose();
	Eigen::MatrixXd row_jerk = basis(localTime, 3).transpose();
	for(int i = 0; i < 3; i++){
		// coeffNum*i selects dimension i's block; polyOrder*segIdx then
		// selects the correct SEGMENT within that dimension's block (segment
		// 0 is only correct when t_now happens to land in the first segment).
		int colOff = coeffNum*i + polyOrder*segIdx;
		ineq_const.f(i+1) = 15;//*sum_jacob_frac/sum_jacobian;
		ineq_const.d(i+1) = -15;
		ineq_const.C.block(i+1, colOff,1, polyOrder) = row_jerk;
		const_conv.block(i, colOff,1, polyOrder) = row_pos;
		const_conv.block(i+3, colOff,1, polyOrder) = row_acc;
		//Goal [posx ; posy; posz ; accx; accy; accz]

		// eq.(9) trust region: keep the actual sampled position/acceleration
		// within a per-axis box of this row's linearization point (pose/accel),
		// so the Taylor expansion above (and the FOV row's Jacobian) stays a
		// valid local approximation instead of being extrapolated arbitrarily
		// far, as flagged by Fig.6 of the paper.
		ineq_const.d(i+4) = pose[i]  - fovTrustPos;
		ineq_const.f(i+4) = pose[i]  + fovTrustPos;
		ineq_const.C.block(i+4, colOff,1, polyOrder) = row_pos;

		ineq_const.d(i+7) = accel[i] - fovTrustAcc;
		ineq_const.f(i+7) = accel[i] + fovTrustAcc;
		ineq_const.C.block(i+7, colOff,1, polyOrder) = row_acc;
	}

	// yaw is dimension index 3 (its own coefficient block); reuse the same
	// position-basis row at this local time for both the FOV Jacobian's 7th
	// column and the eq.(9) yaw trust region.
	int yawColOff = coeffNum*3 + polyOrder*segIdx;
	const_conv.block(6, yawColOff, 1, polyOrder) = row_pos;
	ineq_const.d(10) = pose[3] - fovTrustYaw;
	ineq_const.f(10) = pose[3] + fovTrustYaw;
	ineq_const.C.block(10, yawColOff, 1, polyOrder) = row_pos;

	//std::cout <<"constraint Jacobian " << constr.Jacobian.rows() <<std::endl;
	ineq_const.C.block(0, 0,1, coeffNum*dim) = constr.Jacobian *const_conv;
	*constraint = ineq_const ;
	std::cout << "[FOVDIAG] t_now=" << t_now << " segIdx=" << segIdx
	          << " localTime=" << localTime
	          << " pose=[" << pose[0] << "," << pose[1] << "," << pose[2] << "]"
	          << " accel=[" << accel[0] << "," << accel[1] << "," << accel[2] << "]"
	          << " target=[" << target[0] << "," << target[1] << "," << target[2] << "]"
	          << std::endl;
	std::cout << "[FOVDIAG] constr.diff=" << constr.diff
	          << " grad_dot_x0=" << grad_dot_x0 << " g_x0(margin)=" << g_x0
	          << " fovMargin=" << fovMargin
	          << " d(0)=" << ineq_const.d(0) << " f(0)=" << ineq_const.f(0)
	          << " Jacobian=" << constr.Jacobian << std::endl;
	std::cout << "[FOVDIAG] row0 C norm=" << ineq_const.C.row(0).norm()
	          << " row0 C nonzero blocks: dims 0-2 (pos/accel) cols=[0," << coeffNum*3
	          << "), dim 3 (yaw) cols=[" << coeffNum*3 << "," << coeffNum*4 << ")" << std::endl;

	// Yaw diagnostic: now that derivative_FOV's Jacobian includes yaw as a 7th
	// variable (eq.7-8), this print shows whether the linearization point's
	// yaw is close to or far from the bearing to the target -- a large
	// yaw_err means the FOV row's linear correction (bounded by the eq.(9)
	// trust region above) has real ground to cover, not just position/accel.
	double bearing_to_target = std::atan2(target[1] - pose[1], target[0] - pose[0]);
	double yaw_err = bearing_to_target - pose[3];
	while(yaw_err > M_PI){ yaw_err -= 2*M_PI; }
	while(yaw_err < -M_PI){ yaw_err += 2*M_PI; }
	std::cout << "[FOVDIAG][YAW] yaw=" << pose[3]
	          << " bearing_to_target=" << bearing_to_target
	          << " yaw_err=" << yaw_err
	          << " (0=camera faces target, +-pi=camera faces away)" << std::endl;

	// Optical-axis diagnostic: the modeled camera axis (n_proj, built from B2/B3
	// via the Rodrigues rotation) vs. the actual direction to the target. If
	// these are far apart even when yaw_err above is small, the mismatch is in
	// the axis construction itself (thrust-axis/mount-rotation geometry), not
	// in yaw -- this caught a rotation-sign bug (was rotating +[pi/2+camTilt]
	// instead of -[pi/2+camTilt]) that made the camera face ~120 deg away from
	// the target regardless of position/yaw being correct.
	fov_zero_order axisCheck = fov.fov_eval(target);
	Eigen::Vector3d nd = target - Eigen::Vector3d(pose[0], pose[1], pose[2]);
	double nd_norm = nd.norm();
	double axis_angle_deg = 0.0;
	if(nd_norm > 1e-9){
		double cosang = axisCheck.axis.normalized().dot(nd/nd_norm);
		cosang = std::max(-1.0, std::min(1.0, cosang));
		axis_angle_deg = std::acos(cosang) * 180.0 / M_PI;
	}
	std::cout << "[FOVDIAG][AXIS] optical_axis=[" << axisCheck.axis[0] << "," << axisCheck.axis[1] << "," << axisCheck.axis[2] << "]"
	          << " dir_to_target=[" << (nd_norm>1e-9?nd[0]/nd_norm:0) << "," << (nd_norm>1e-9?nd[1]/nd_norm:0) << "," << (nd_norm>1e-9?nd[2]/nd_norm:0) << "]"
	          << " angle_deg=" << axis_angle_deg
	          << " (0=camera axis points at target, 180=points directly away)" << std::endl;
	return true;
}

double TrajBase::checkFovAxisAngle(Eigen::Vector3d target, Eigen::Vector4d pose, Eigen::Vector3d accel){
	FOV_constraint fov(pose, accel, fovCamTilt, fovRh);
	fov_zero_order e = fov.fov_eval(target);
	Eigen::Vector3d nd = target - Eigen::Vector3d(pose[0], pose[1], pose[2]);
	double nd_norm = nd.norm();
	if(nd_norm < 1e-9){ return 0.0; }
	double cosang = e.axis.normalized().dot(nd/nd_norm);
	cosang = std::max(-1.0, std::min(1.0, cosang));
	return std::acos(cosang) * 180.0 / M_PI;
}

