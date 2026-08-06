#include <traj_gen/trajectory/QPpolyTraj.h>
using namespace std;
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>
#include <limits>

using namespace Eigen;

// Named numeric constants (documented; not tuned via config).
static constexpr double kFastSolveRegularization = 1e-3; // Tikhonov term added to Q in fastMTSolve for positive-definiteness
static constexpr double kEmptyIneqBound = 0.1;           // trivial bound for the placeholder inequality row when no constraints exist

void QPpolyTraj::setQpSolveTimeout(double seconds){
	qpSolveTimeoutS = seconds;
}

QPpolyTraj::QPpolyTraj()
{
	dim =4;
	limits = Eigen::VectorXd::Zero(5);
	limits(1)=2.0;
	limits(2)=2.0;
}


QPpolyTraj::QPpolyTraj(int d)
{    	dim = d;
		limits = Eigen::VectorXd::Zero(5);
			limits(1)=2.0;
	limits(2)=2.0;
}

QPpolyTraj::QPpolyTraj(int d,std::vector<double> time)
{
	dim = d;
    segmentTimes = time;
	limits = Eigen::VectorXd::Zero(5);
		limits(1)=2.0;
	limits(2)=2.0;
}




//Fast Solve is a quicker solver at the cost of being not the perfect outcome
Eigen::MatrixXd QPpolyTraj::solve(int minDeriv)
{
	// Declare variables and zero memories
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(1, 4);
		return coeff;
	 }
	 if(add_joint_ineq_constr.d.rows()+add_joint_eq_constr.a.rows()>0){
		 return SMsolve(minDeriv);
	 }
	if(fast){
		return fastMTSolve( minDeriv);
	}
	else{
		return MTsolve( minDeriv);
	}
}


void  thread_FASTQP(int dimension, Eigen::MatrixXd D, int vectorSize, QP_constraint qp,
   Eigen::MatrixXd C, Eigen::VectorXd d,Eigen::VectorXd g0, Eigen::MatrixXd* coeff){
    Eigen::VectorXd sol = Eigen::VectorXd::Zero(vectorSize);
	//Create Copy since the quadprog changes the X&T Q  X matrix each run
	Eigen::MatrixXd A = qp.a, b = qp.b;
	Eigen::MatrixXd Obj = D;
	QP::solve_quadprog(Obj, g0, 
			A.transpose(),-1*b,  
			C.transpose(), d, 
			sol);
	for (int i =0;i<vectorSize;i++){
		coeff->operator()(i,dimension)  = sol[i];
	}
	
}

//EIGEN QP
Eigen::MatrixXd QPpolyTraj::fastMTSolve(int minDeriv)
{
    //Cut the object into 3 vectors
    //We need 3 things for our QP Programming
     // One Q for X^T*Q*X to minimize
     // Two b = A*x constraint to follow
     //For now we consider the constraints
	int coeffNum = (vertices.size() - 1) *  polyOrder;
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
		return coeff;
	 }
	 std::vector<boost::thread> threads;
    int numberSegments = segmentTimes.size();
	// Declare variables and zero memories
	//calculating the number of constraints
	int numConstraint = countEqConstraintRows();
	Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero((numConstraint), coeffNum);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(numConstraint);
   //generate object function
   	Eigen::MatrixXd D = generateObjFun(minDeriv)+kFastSolveRegularization*Eigen::MatrixXd::Identity(coeffNum, coeffNum);
	Eigen::VectorXd g0 = Eigen::VectorXd::Zero(coeffNum);
	ooqpei::OoqpEigenInterface solver();
	for (int j = 0; j < dim; j++){
		QP_constraint qp = genConstraint( j,numConstraint); //each dimension has its unique equality constraint 
		threads.push_back(boost::thread(thread_FASTQP, j,  D, coeffNum, qp,C,d,g0,&coeff));
	}
  for (auto &th : threads) {
    th.join();
  }
 	coeffSolved = coeff;
	//quadprog has no failure as matrix inversion
	  traj_valid[0] = true; 
		traj_valid[1] = true; 
		traj_valid[2] = true; 
		traj_valid[3] = true; 
    return coeff;
}


//SIngle matrix solve
Eigen::MatrixXd QPpolyTraj::SMsolve(int minDeriv)
{
    //Cut the object into 3 vectors
	// Declare variables and zero memories
    int coeffNum = (vertices.size() - 1) *  polyOrder;
		Eigen::MatrixXd coeff(coeffNum, dim);
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
		return coeff;
	 }
	//std::cout << " Generate joint matrix solve " <<std::endl;
	QP_constraint qp_constr =  genJointConstraint();
	//std::cout << " generate inequality joint solve" <<std::endl;
	QP_ineq_const qp_ineq_constr =  genInEqJointConstraint();
	//std::cout << " generate joint objective solve" <<std::endl;
	Eigen::MatrixXd comp = generateJointObjFun(minDeriv);
	//std::cout << " all matrices generated" <<std::endl;
	/*
	std::cout << "A: "<<qp_constr.a <<std::endl;
	std::cout << "b: "<<qp_constr.b <<std::endl;
	std::cout << "C: "<<qp_ineq_constr.C <<std::endl;
	std::cout << "d: "<<qp_ineq_constr.d <<std::endl;
	std::cout << "f: "<<qp_ineq_constr.f <<std::endl;*/
	//There is definitely a problem here Trajectory generation fails a second time for some reason....
	// [SMDIAG] Sanity-check the assembled joint problem before handing it to OOQP,
	// so a failure can be attributed to a specific piece instead of guessing:
	// (1) any d>f box-constraint row (unconditionally infeasible, the same class
	//     of bug as the earlier FOV index bug), (2) whether Ax=b is even
	// consistent on its own (equality system rank vs augmented [A|b] rank --
	// if they differ, no x satisfies Ax=b, independent of any inequality).
	{
		int nBad = 0;
		for(int r = 0; r < qp_ineq_constr.d.rows(); r++){
			if(qp_ineq_constr.d(r) > qp_ineq_constr.f(r)){
				nBad++;
				if(nBad <= 5){
					std::cout << "[SMDIAG] row " << r << " has d(" << qp_ineq_constr.d(r)
					          << ") > f(" << qp_ineq_constr.f(r) << ") -- unconditionally infeasible"
					          << std::endl;
				}
			}
		}
		std::cout << "[SMDIAG] A=" << qp_constr.a.rows() << "x" << qp_constr.a.cols()
		          << " C=" << qp_ineq_constr.C.rows() << "x" << qp_ineq_constr.C.cols()
		          << " badBoxRows=" << nBad << std::endl;
		Eigen::FullPivLU<Eigen::MatrixXd> luA(qp_constr.a);
		int rankA = luA.rank();
		Eigen::MatrixXd Aug(qp_constr.a.rows(), qp_constr.a.cols() + 1);
		Aug.leftCols(qp_constr.a.cols()) = qp_constr.a;
		Aug.rightCols(1) = qp_constr.b;
		int rankAug = Eigen::FullPivLU<Eigen::MatrixXd>(Aug).rank();
		std::cout << "[SMDIAG] rank(A)=" << rankA << " rank([A|b])=" << rankAug
		          << " rows(A)=" << qp_constr.a.rows()
		          << (rankA != rankAug ? "  <-- Ax=b INCONSISTENT (no x satisfies it)" : "")
		          << std::endl;
	}
	Eigen::SparseMatrix<double, Eigen::RowMajor> Obj = comp.sparseView();
	Eigen::SparseMatrix<double, Eigen::RowMajor> AooQP = qp_constr.a.sparseView();
	Eigen::SparseMatrix<double, Eigen::RowMajor> C = qp_ineq_constr.C.sparseView();
    	Eigen::VectorXd sol = Eigen::VectorXd::Zero(coeffNum*dim);
	Eigen::VectorXd g0;
	if(useCostVector){
		g0 = costVector;
	}
	else{
		g0 = Eigen::VectorXd::Zero(coeffNum*dim);
	}
	//Eigen::VectorXd g0  = qp_ineq_constr.C.transpose();
	const bool ignoreUnknownError = false;
  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, and d <= Cx <= f
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
    //std::cout << " qp_constr.b" <<std::endl;
    // std::cout <<  qp_constr.b <<std::endl;
	// OOQP's MA27 backend can throw std::runtime_error ("MA27 cannot factor
	// matrix") on a numerically degenerate problem instead of returning false.
	// This call runs on the caller's own thread (unlike thread_QP's per-axis
	// solves, which run in their own threads and are already guarded) -- an
	// uncaught exception here propagates straight up through solve() into
	// whatever's driving the replanner (traj_manager's main loop), calling
	// std::terminate() and killing the whole node. Catch broadly and treat it
	// as a normal solve failure.
	bool solved = false;
	try {
		solved = ooqpei::OoqpEigenInterface::solve(Obj, g0 ,
	                   AooQP, qp_constr.b,
	                    C,
	                    qp_ineq_constr.d,qp_ineq_constr.f,
						sol,ignoreUnknownError);
	} catch (const std::exception& e) {
		std::cout << "[QP_EXCEPTION] OOQP threw on joint solve: " << e.what()
		          << " -- treating as failed." << std::endl;
	} catch (...) {
		std::cout << "[QP_EXCEPTION] OOQP threw a non-standard exception on "
		          << "joint solve -- treating as failed." << std::endl;
	}
	if(solved){
		// Unlike thread_QP (per-axis), this success path previously printed
		// nothing at all -- only failure was logged. That made it impossible to
		// tell from the log whether the joint FOV solve ever actually succeeded,
		// e.g. earlier in a replan sequence when more segment time/distance is
		// left, before later failing once time shrinks near the end.
		std::cout << "QP successful generation [joint solve, all axes]"
		          << " numIneqRows=" << qp_ineq_constr.d.rows() << std::endl;
		traj_valid[0] = true;
		traj_valid[1] = true;
		traj_valid[2] = true;
		traj_valid[3] = true;
	}
	else{
		std::cout << "QP Failed generation [joint solve, all axes]"
		          << " numIneqRows=" << qp_ineq_constr.d.rows() << std::endl;
		traj_valid[0] = false;
		traj_valid[1] = false;
		traj_valid[2] = false;
		traj_valid[3] = false;
	}
    /*
	ooqpei::OoqpEigenInterface::solve(Obj, g0, 
			AooQP,btotal,  
			l,u, 
			);*/
	for (int j = 0; j < dim; j++){
		for (int i =0;i<coeffNum;i++){
			coeff(i,j) = sol[i+j*coeffNum];
		}
	}
	coeffSolved = coeff;
    return coeff;
}



// coeff and done are shared_ptrs, not raw pointers to MTsolve's locals: if
// this call times out from MTsolve's point of view (see qpSolveTimeoutS), the
// thread is detached and kept running rather than joined, so it can outlive
// the MTsolve stack frame that used to own them. Raw pointers there would be
// dangling writes the moment OOQP eventually did return. done is set true as
// the very last step (including on the exception paths below), so MTsolve's
// polling loop only observes completion once coeff/traj_valid are settled --
// or promptly, rather than waiting out the full timeout, if OOQP throws
// before the deadline.
void  thread_QP(int dimension, Eigen::MatrixXd Qobj, int coeffNum, QP_constraint qp,
   QP_ineq_const ineq_qp, std::shared_ptr<Eigen::MatrixXd> coeff, std::vector<char>* traj_valid,
   std::shared_ptr<std::atomic<bool>> done, Eigen::VectorXd scale){
    static const char* kAxisName[4] = {"x", "y", "z", "yaw"};
    const char* axisName = (dimension >= 0 && dimension < 4) ? kAxisName[dimension] : "?";
	// OOQP's MA27 backend can throw std::runtime_error ("MA27 cannot factor
	// matrix") on a numerically degenerate problem instead of returning false.
	// An uncaught exception on ANY thread -- including this one, possibly
	// running detached well after MTsolve gave up on it -- calls
	// std::terminate() and kills the whole process. Catch broadly and treat it
	// the same as a normal solve failure: traj_valid[dimension] simply never
	// gets set true.
	try {
		const bool ignoreUnknownError = false;
		// `sol` here is solved in the SCALED (normalized-time) variable space,
		// not the physical coefficient space -- see `scale`'s construction in
		// MTsolve() for the derivation. x_i (physical) = scale(i) * y_i
		// (solved). Column-scaling A/C and congruence-scaling Q by `scale` is
		// algebraically identical to building this same QP directly from a
		// basis normalized to segment-local time tau=t/T in [0,1]: since
		// sum_i x_i*t^i = sum_i (scale(i)*y_i)*t^i = sum_i y_i*(t/T)^i when
		// scale(i) = T^-i, this is exactly the change of variables that
		// normalization would produce, without touching genConstraint(),
		// genInEqConstraint(), generateObjFun(), evalTraj(), or basis() at
		// all -- every b/d/f value (physical positions, velocities, etc.)
		// and every row this function receives is completely unchanged;
		// only the linear-algebra objects actually hitting OOQP are rescaled,
		// and the result is un-scaled back to physical coefficients below
		// before being written into coeff. Fixes basis()'s un-normalized
		// t^0..t^9 powers spanning ~20 orders of magnitude within a single
		// multi-second segment, confirmed in the field via
		// [MIN_ALTITUDE_DIAG] conditioning check: cond(objective D)=inf on
		// every failing z solve, cond(equality A) climbing into the 1e6-1e8
		// range as retries grew the segment time.
		Eigen::VectorXd sol = Eigen::VectorXd::Zero(coeffNum);
		Eigen::VectorXd g0 = Eigen::VectorXd::Zero(coeffNum);
		Eigen::MatrixXd Qobj_scaled = scale.asDiagonal() * Qobj * scale.asDiagonal();
		Eigen::MatrixXd A_scaled = qp.a * scale.asDiagonal();
		Eigen::MatrixXd C_scaled = ineq_qp.C * scale.asDiagonal();
		//Create Copy since the quadprog changes the X&T Q  X matrix each run
		Eigen::SparseMatrix<double, Eigen::RowMajor> Obj = Qobj_scaled.sparseView();
		Eigen::MatrixXd A = A_scaled, b = qp.b;
		Eigen::SparseMatrix<double, Eigen::RowMajor> AooQP = A.sparseView();
		Eigen::SparseMatrix<double, Eigen::RowMajor> C = C_scaled.sparseView();
	  /*!
	   * Solve min 1/2 x' Q x + c' x, such that A x = b, and d <= Cx <= f
	   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
	   * @param [in] c a vector (nx1)
	   * @param [in] A a (possibly null) matrices (m_axn)
	   * @param [in] b a vector (m_ax1)
	   * @param [in] C a (possibly null) matrices (m_cxn)
	   * @param [in] d a vector (m_cx1)
	   * @param [in] f a vector (m_cx1)
	   * @param [out] x a vector of variables (nx1)
	   * @return true if successful
	   */
		if(ooqpei::OoqpEigenInterface::solve(Obj, g0 ,
	                   AooQP, b,
	                    C,
	                    ineq_qp.d,ineq_qp.f,
						sol,ignoreUnknownError)){
			std::cout << "QP successful generation [dim=" << dimension << " (" << axisName << ")]" << std::endl;
			traj_valid->operator[](dimension) = true;
			// Only write this dimension's column when the solve actually
			// succeeded. `sol` is zero-initialized above, but OOQP can leave it
			// holding a partial/invalid intermediate iterate on a run that
			// ultimately returns false -- writing that into the shared coeff
			// matrix regardless of success let a failed dimension's garbage
			// leak into coeffSolved (which MTsolve() copies from coeff
			// unconditionally), even though traj_valid correctly marked that
			// dimension as not solved. Confirmed in the field: a replan cycle
			// where z kept failing produced an evaluated position with z at
			// (numerically) exactly zero -- the world origin -- for a
			// trajectory whose real altitude was nowhere near it, exactly
			// what a zero/garbage coefficient column evaluates to. Leaving
			// coeff's column at whatever it already was (its Zero()
			// initialization from MTsolve(), untouched) instead keeps a
			// failed dimension's column a clean, known zero rather than
			// OOQP's unpredictable leftover state.
			for (int i =0;i<coeffNum;i++){
				// Un-scale: sol is in the normalized-time variable space (see
				// the comment above `sol`'s declaration); coeff must hold the
				// physical coefficients basis()/evalTraj() expect.
				coeff->operator()(i,dimension)  = scale(i) * sol[i];
			}
		}
		else{
			std::cout << "QP Failed generation [dim=" << dimension << " (" << axisName << ")]"
			          << " numIneqRows=" << ineq_qp.d.rows() << std::endl;
		}
	} catch (const std::exception& e) {
		std::cout << "[QP_EXCEPTION] OOQP threw on dim=" << dimension << " (" << axisName
		          << "): " << e.what() << " -- treating as failed." << std::endl;
	} catch (...) {
		std::cout << "[QP_EXCEPTION] OOQP threw a non-standard exception on dim=" << dimension
		          << " (" << axisName << ") -- treating as failed." << std::endl;
	}
	done->store(true);
}

// Diagnostic only, no behavior change to the actual solve -- called from
// MTsolve() right after z (dim=2) fails while MIN_ALTITUDE is enabled.
//
// The per-segment release (see applyMinAltitude()) only widens/narrows the
// window near the TARGET end of the segment. If z keeps failing even after
// that window was genuinely relaxed (confirmed via row-count logs dropping),
// the remaining cause could be one of two very different things:
//   (a) the required descent still doesn't fit in the released window --
//       tighten/rework the release further, or
//   (b) with only a handful of free coefficients left once the boundary
//       conditions (odom start: pos/vel/accel/jerk/snap; perch-conditioned
//       target: pos/vel/accel) are pinned, the polynomial's NATURAL shape
//       -- driven purely by those boundary values and the min-snap-style
//       cost, nothing to do with the target -- may dip through the floor
//       somewhere in the MIDDLE of the segment. No amount of releasing time
//       near the target can fix that.
// This re-solves z with ONLY the MIN_ALTITUDE inequality rows removed (every
// other constraint on z, e.g. the perch accel band, stays exactly as in the
// real solve) and samples the resulting z(t) across the whole segment
// against the floor, to show directly which case this is.
void QPpolyTraj::diagnoseMinAltitudeShape(int minDeriv, const Eigen::MatrixXd& D, int numConstraint){
	const int dimension = 2; // z
	int coeffNum = (vertices.size() - 1) * polyOrder;

	// MIN_ALTITUDE's own rows are uniquely identified by how applyMinAltitude()
	// builds them: derivOrder==0 (position), spanFromStart==true, InEqDim(2)==1.
	// Strip just those from a scratch copy of vertices, leaving every other
	// constraint (e.g. calcPerchCond's accel band, also on z) untouched.
	std::vector<waypoint> savedVertices = vertices;
	for(size_t i = 0; i < vertices.size(); i++){
		std::vector<waypoint_ineq_const> kept;
		for(size_t j = 0; j < vertices[i].ineq_constraint.size(); j++){
			const waypoint_ineq_const& ic = vertices[i].ineq_constraint[j];
			bool isMinAltitude = (ic.derivOrder == 0 && ic.spanFromStart && ic.InEqDim(2) == 1);
			if(!isMinAltitude){
				kept.push_back(ic);
			}
		}
		vertices[i].ineq_constraint = kept;
	}

	QP_constraint qp = genConstraint(dimension, numConstraint);
	QP_ineq_const ineq_qp = genInEqConstraint(dimension);
	vertices = savedVertices; // restore immediately, before anything else can observe it

	// Same normalized-time change of variables as thread_QP's real solve
	// (see its comment on `sol`) -- without this, this diagnostic solves on
	// the same catastrophically ill-conditioned raw-time basis the real
	// solve used to, and its answer (including whether it solves at all)
	// isn't representative of what the actual, now-normalized solve path
	// produces. `sol` below comes back in the normalized (y) variable space
	// and is un-scaled to physical coefficients immediately after a
	// successful solve, before anything samples it.
	Eigen::VectorXd scale = buildTimeNormalizationScale();
	Eigen::VectorXd sol = Eigen::VectorXd::Zero(coeffNum);
	Eigen::VectorXd g0 = Eigen::VectorXd::Zero(coeffNum);
	Eigen::MatrixXd Qobj_scaled = scale.asDiagonal() * D * scale.asDiagonal();
	Eigen::MatrixXd A_scaled = qp.a * scale.asDiagonal();
	Eigen::MatrixXd C_scaled = ineq_qp.C * scale.asDiagonal();
	Eigen::SparseMatrix<double, Eigen::RowMajor> Obj = Qobj_scaled.sparseView();
	Eigen::MatrixXd A = A_scaled, b = qp.b;
	Eigen::SparseMatrix<double, Eigen::RowMajor> AooQP = A.sparseView();
	Eigen::SparseMatrix<double, Eigen::RowMajor> C = C_scaled.sparseView();
	bool ok = false;
	try {
		ok = ooqpei::OoqpEigenInterface::solve(Obj, g0, AooQP, b, C, ineq_qp.d, ineq_qp.f, sol, false);
	} catch (const std::exception& e) {
		std::cout << "[MIN_ALTITUDE_DIAG] shape-without-floor solve threw: " << e.what()
		          << " -- skipping" << std::endl;
		return;
	} catch (...) {
		std::cout << "[MIN_ALTITUDE_DIAG] shape-without-floor solve threw a non-standard "
		          << "exception -- skipping" << std::endl;
		return;
	}
	if(!ok){
		std::cout << "[MIN_ALTITUDE_DIAG] z still fails even WITHOUT the min-altitude floor -- "
		          << "the floor is not the (sole) blocker here" << std::endl;
		return;
	}
	// Un-scale back to physical coefficients -- see the comment above `scale`.
	for(int i = 0; i < coeffNum; i++){ sol(i) *= scale(i); }

	double T = 0.0;
	for(size_t s = 0; s < segmentTimes.size(); s++){ T += segmentTimes[s]; }
	const int kSamples = 20;
	int numSeg = segmentTimes.size();
	bool anyViolation = false;
	std::cout << "[MIN_ALTITUDE_DIAG] shape without floor (floor=" << lastMinAltitudeFloorZ << "):";
	for(int k = 0; k <= kSamples; k++){
		double t = T * (double)k / (double)kSamples;
		// Locate segment/local time exactly as evalTraj() does.
		double localT = t;
		int seg = 0;
		while(seg < numSeg - 1 && segmentTimes[seg] < localT){
			localT -= segmentTimes[seg];
			seg += 1;
		}
		if(localT > segmentTimes[seg]){ localT = segmentTimes[seg]; }
		Eigen::VectorXd power = basis(localT, 0);
		double z = 0.0;
		for(int p = 0; p < polyOrder; p++){
			z += power(p) * sol(seg*polyOrder + p);
		}
		bool violates = (z > lastMinAltitudeFloorZ);
		anyViolation = anyViolation || violates;
		std::cout << " [t=" << t << " z=" << z << (violates ? " VIOLATES" : "") << "]";
	}
	std::cout << std::endl;
	if(!anyViolation){
		std::cout << "[MIN_ALTITUDE_DIAG] shape without floor never actually violates it -- "
		          << "the real solve's failure is likely an interaction with another "
		          << "sampled constraint on z (e.g. the perch accel band), not min-altitude "
		          << "itself" << std::endl;
	}
}

// Diagnostic only, no behavior change. basis() (see its definition) builds
// every constraint/objective row from RAW, un-normalized powers of time --
// t[i] = pow(time, i) for i up to polyOrder-1 (9) -- rather than a segment-
// duration-normalized fraction. For a several-second segment that spans an
// enormous dynamic range within one matrix: e.g. t^9 at t=0.05s is ~2e-12,
// while at t=9s it is ~4e8 -- about 20 orders of magnitude apart, deep into
// the range where double-precision (~15-17 significant digits) can no
// longer represent both scales in the same linear system without severe
// loss of precision. diagnoseMinAltitudeShape() found the unconstrained
// shape's overshoot getting WORSE (not better) as retries grew the segment
// time -- the opposite of what a genuine kinematic/DOF tightness would do,
// but exactly what worsening numerical conditioning would do. This computes
// the actual condition numbers (largest/smallest singular value) of the
// EXACT objective (D) and equality-constraint (A) matrices this failing
// solve fed to OOQP, plus the raw basis dynamic range directly, so a field
// test can confirm or rule this out with real numbers instead of estimates.
void QPpolyTraj::logSolveConditioning(int dimension, const Eigen::MatrixXd& D, int numConstraint){
	double segT = 0.0;
	for(size_t s = 0; s < segmentTimes.size(); s++){ segT += segmentTimes[s]; }
	double basisRatio = (ineqSampleDt > 1e-9)
	    ? pow(segT, polyOrder - 1) / pow(ineqSampleDt, polyOrder - 1)
	    : std::numeric_limits<double>::infinity();

	auto condOf = [](const Eigen::MatrixXd& M) -> double {
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M);
		Eigen::VectorXd sv = svd.singularValues();
		return (sv.size() > 0 && sv(sv.size() - 1) > 1e-300)
		    ? sv(0) / sv(sv.size() - 1) : std::numeric_limits<double>::infinity();
	};

	double condD_raw = condOf(D);
	QP_constraint qp = genConstraint(dimension, numConstraint);
	double condA_raw = condOf(qp.a);

	// Same normalized-time change of variables thread_QP's real solve
	// applies (see its comment on `sol`) -- reported alongside the raw
	// numbers so a field test directly confirms the fix on the ACTUAL solve
	// path, instead of inferring it indirectly from solve latency. Before
	// this, this function reported only condD_raw/condA_raw -- the same
	// numbers it always had, regardless of thread_QP's fix, which made a
	// still-fixed solve look identically broken in this log line.
	Eigen::VectorXd scale = buildTimeNormalizationScale();
	double condD_scaled = condOf(scale.asDiagonal() * D * scale.asDiagonal());
	double condA_scaled = condOf(qp.a * scale.asDiagonal());

	std::cout << "[MIN_ALTITUDE_DIAG] conditioning check -- segT=" << segT
	          << " ineqSampleDt=" << ineqSampleDt
	          << " raw basis magnitude ratio (t^" << (polyOrder - 1) << " at segT vs at dt)="
	          << basisRatio
	          << " cond(objective D) raw=" << condD_raw << " normalized=" << condD_scaled
	          << " cond(equality A) raw=" << condA_raw << " normalized=" << condA_scaled
	          << std::endl;
}

// Per-coefficient scale for the normalized-time change of variables x_i =
// scale(i)*y_i (see thread_QP's comment on `sol` for the full derivation):
// column i of segment s's polyOrder-wide block gets scale = segmentTimes[s]^-p,
// where p is that column's power within its own segment. Shared by MTsolve's
// real per-dimension solve AND both z-failure diagnostics
// (diagnoseMinAltitudeShape, logSolveConditioning) so all three interpret
// coefficients in exactly the same normalized space -- a single source of
// truth. Before this was factored out, the two diagnostics each built their
// own solve directly from the raw (un-normalized) D/A/C, so they kept
// reporting the pre-fix numbers (e.g. cond(D)=inf) even after MTsolve's real
// solve had already moved to the normalized ones, which looked like the fix
// hadn't done anything when it had.
Eigen::VectorXd QPpolyTraj::buildTimeNormalizationScale(){
	int coeffNum = (vertices.size() - 1) * polyOrder;
	int numberSegments = segmentTimes.size();
	Eigen::VectorXd scale = Eigen::VectorXd::Ones(coeffNum);
	for(int s = 0; s < numberSegments; s++){
		double T = segmentTimes[s];
		for(int p = 0; p < polyOrder; p++){
			scale(s*polyOrder + p) = (T > 1e-9) ? pow(T, -p) : 1.0;
		}
	}
	return scale;
}

Eigen::MatrixXd QPpolyTraj::MTsolve(int minDeriv)
{
    //Cut the object into 3 vectors
    //We need 3 things for our QP Programming
     // One Q for X^T*Q*X to minimize
     // Two b = A*x constraint to follow
     //For now we consider the constraints
    int numberSegments = segmentTimes.size();
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
		return coeff;
	 }
	 std::vector<boost::thread> threads;
	// Declare variables and zero memories
	int numConstraint = countEqConstraintRows();
	// Heap-allocated and shared (not a local Eigen::MatrixXd) because a timed-out
	// thread below is detached rather than joined, so it can keep running -- and
	// writing to this -- after MTsolve returns and its stack frame is gone. Each
	// spawned thread holds its own copy of the shared_ptr, keeping the matrix
	// alive for as long as that thread runs, however long that ends up being.
	auto coeff = std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Zero(coeffNum, dim));
	// One completion flag per dimension, heap-allocated for the same reason as
	// coeff above (a detached thread must be able to set it after MTsolve has
	// already returned). Polled below instead of using boost::thread's
	// chrono-based try_join_for, which pulls in libboost_chrono/libboost_system
	// as an extra link dependency this project doesn't otherwise need --
	// std::chrono/std::atomic/std::this_thread are header-only/standard-library.
	std::vector<std::shared_ptr<std::atomic<bool>>> done(dim);
   //generate object function
   	Eigen::MatrixXd D = generateObjFun(minDeriv);
	// See buildTimeNormalizationScale()'s comment for the derivation --
	// makes solving for y (in thread_QP) equivalent to using a basis
	// normalized to segment-local time tau=t/T in [0,1], instead of
	// basis()'s raw seconds t^0..t^9 -- the source of the catastrophic
	// ill-conditioning confirmed via [MIN_ALTITUDE_DIAG]'s conditioning
	// check (cond(objective D)=inf on every failing solve).
	Eigen::VectorXd scale = buildTimeNormalizationScale();
	for (int j = 0; j < dim; j++){
		QP_constraint qp = genConstraint( j,numConstraint); //each dimension has its unique equality constraint
		QP_ineq_const ineq_qp = genInEqConstraint(j);
		// thread_QP only ever sets traj_valid[j] TRUE (on success); reset here so a
		// dimension that times out below (or genuinely fails) below isn't left
		// stale-true from an earlier, unrelated solve.
		traj_valid[j] = false;
		done[j] = std::make_shared<std::atomic<bool>>(false);
		threads.push_back(boost::thread(thread_QP, j,  D, coeffNum, qp,ineq_qp,coeff,&traj_valid,done[j],scale));
	}
	// Bounded join: OOQP occasionally fails to converge/terminate on a
	// numerically tight problem (see the qpSolveTimeoutS member comment in
	// QPpolyTraj.h). An unbounded join here previously hung the whole node
	// forever with no further log output the moment that happened. Poll each
	// dimension's completion flag with a short sleep instead of a blocking
	// join call, so we can bound the wait ourselves.
	static const char* kAxisName[4] = {"x", "y", "z", "yaw"};
	const auto pollInterval = std::chrono::milliseconds(10);
	for (size_t j = 0; j < threads.size(); j++) {
		auto deadline = std::chrono::steady_clock::now() +
		                std::chrono::duration<double>(qpSolveTimeoutS);
		while (!done[j]->load() && std::chrono::steady_clock::now() < deadline) {
			std::this_thread::sleep_for(pollInterval);
		}
		if (done[j]->load()) {
			threads[j].join(); // already finished -- returns immediately
		} else {
			const char* axisName = (j < 4) ? kAxisName[j] : "?";
			std::cout << "[QP_TIMEOUT] OOQP did not return within " << qpSolveTimeoutS
			          << "s on dim=" << j << " (" << axisName << ") -- treating as failed "
			          << "and moving on. OOQP gives no safe way to cancel an in-progress "
			          << "solve, so the thread is detached and left running in the "
			          << "background; this dimension's coefficients are not used."
			          << std::endl;
			traj_valid[j] = false;
			threads[j].detach();
		}
	}
	// [MIN_ALTITUDE_DIAG] Diagnostic only, no behavior change. z (dim=2) has
	// been failing intermittently in the field with no obvious trigger --
	// suspected cause is the anchor's real altitude sitting close enough to
	// the MIN_ALTITUDE floor that ordinary tracking noise occasionally puts it
	// on the wrong side of the bound. Log the anchor's actual z/vz next to the
	// floor whenever z's solve fails, so a field test can confirm or rule this
	// out directly instead of inferring it from a smoothed post-hoc plot.
	if(minAltitudeEnabled && traj_valid.size() > 2 && !traj_valid[2] && !vertices.empty()){
		Eigen::VectorXd anchorPos, anchorVel;
		double az = (vertices[0].getPos(&anchorPos) == 1 && anchorPos.rows() > 2)
		            ? anchorPos(2) : std::numeric_limits<double>::quiet_NaN();
		double avz = (vertices[0].getVelo(&anchorVel) == 1 && anchorVel.rows() > 2)
		             ? anchorVel(2) : std::numeric_limits<double>::quiet_NaN();
		std::cout << "[MIN_ALTITUDE_DIAG] z solve failed -- anchor z=" << az
		          << " vz=" << avz << " floor=" << lastMinAltitudeFloorZ
		          << " (anchor is " << (az <= lastMinAltitudeFloorZ ? "above/at" : "BELOW")
		          << " the floor)" << std::endl;
		// Distinguishes "the required descent genuinely can't fit in the
		// released window near the target" from "the natural, unconstrained
		// shape (given the boundary conditions alone) dips through the floor
		// somewhere in the MIDDLE of the segment, which no amount of
		// releasing time near the target can fix" -- see diagnoseMinAltitudeShape().
		diagnoseMinAltitudeShape(minDeriv, D, numConstraint);
		// Checks whether the mid-segment overshoot diagnoseMinAltitudeShape()
		// shows is actually a numerical-conditioning artifact of basis()'s
		// un-normalized, raw-seconds time powers (t^0..t^9), rather than a
		// genuine kinematic/DOF infeasibility -- see logSolveConditioning().
		logSolveConditioning(2, D, numConstraint);
	}
	// Snapshot copy: any detached, still-running thread from the timeout branch
	// above only ever writes its own timed-out dimension's column, and that
	// dimension is already marked invalid (traj_valid[j]=false), so a possible
	// torn read of that one column here doesn't affect anything actually
	// consumed downstream -- calculateCurrentPt() requires all four dims valid.
	coeffSolved = *coeff;
    return *coeff;
}





Eigen::VectorXd QPpolyTraj::calculateCurrentPt( int Order, double time){
    bool lastPoint = false;
    Eigen::VectorXd current_pt_(dim);
    //Calculate the length of the segment inefficinet figure it out
//CONDITION CHECK THAT THERE WAS A SUCCESFUL WORK ON ALL 4 AXISES
    if (!(traj_valid[0]&&traj_valid[1]&&traj_valid[2] &&traj_valid[3])){
//IF any of the above is false
		std::cout << "FAILURE GENERATING PATH" <<std::endl;
		return Eigen::VectorXd::Zero(dim);
    }

    int Segment_idx =0;
    while (segmentTimes[Segment_idx] < time){
		time -= segmentTimes[Segment_idx];
		Segment_idx++;
		if (Segment_idx == segmentTimes.size()){
           lastPoint =true;
			break;
		}
    }
    if(lastPoint){
		Segment_idx=segmentTimes.size()-1;
		time = segmentTimes[Segment_idx];
     }
    //Normalize the time based on the fulltime
    //std::cout << "Updating power basis" << std::endl;
    Eigen::VectorXd power = basis(time,Order);
    double culsum = 0;
    for (int d = 0; d < dim; d++) {
		int count = 0;
		culsum = 0;
//std::cout << "Updating dimension: " << d << std::endl;
		for (int p = (polyOrder*Segment_idx); p < polyOrder*(Segment_idx+1); p++) {
			culsum = culsum + power(count) * coeffSolved(p, d);
			count = count +1;
		}
	//std::cout << "The sum is: " << culsum << std::endl;
		current_pt_(d) = culsum;
	//std::cout << "The current_pt_(d) is: " << current_pt_ << std::endl;
	}
    return current_pt_;
}


Eigen::MatrixXd QPpolyTraj::evalTraj(double time){	
    int numSeg = segmentTimes.size();
	//Calculates the fulltimes of each segment
	//Example if you have 2 segmes 1,0.5
	//Fultime would be 0, 1, 1.5
	//Calculate the length of the segment inefficinet figure it out
	//CONDITION CHECK THAT THERE WAS A SUCCESFUL WORK ON ALL 4 AXISES
	if (!(traj_valid[0]&&traj_valid[1]&&traj_valid[2] &&traj_valid[3])){
		//IF any of the above is false
		std::cout << "FAILURE GENERATING PATH" <<std::endl;
		return Eigen::MatrixXd::Constant(5,dim,-1e10);
	}
	if(numSeg < 1){
		return Eigen::MatrixXd::Constant(5,dim,-1e10);
	}
	int k=0;
	if(time < 0.0){
		time = 0.0;
	}
	// Find the segment containing `time`, accumulating local time. Stop at the
	// last segment so a request at/after the trajectory end never walks k past
	// coeffSolved's rows (which triggers an Eigen out-of-range assertion).
    while(k < numSeg-1 && segmentTimes[k] < time) {
		time -= segmentTimes[k];
		k+=1;
    }
	// If the request is beyond this segment's duration (e.g. floating-point
	// overshoot at the trajectory end), clamp to the segment endpoint rather
	// than extrapolating the polynomial.
	if(time > segmentTimes[k]){
		time = segmentTimes[k];
	}
	//std::cout << "segment : " << k << std::endl;
	Eigen::MatrixXd point(5,dim);
	for(int Order=0; Order < 5;Order++){
		//Normalize the time based on the fulltime
		Eigen::VectorXd power = basis(time,Order);
		for (int d = 0; d < dim; d++) {
			int count = 0;
			double culsum = 0;
			for (int p = (polyOrder*k); p < polyOrder*(k+1); p++) {
				culsum = culsum + power(count) * coeffSolved(p,d);
				count = count +1;
				}
			point(Order,d)= culsum;
		}
	}	
	return point;
}



bool QPpolyTraj::calcAngVBound(int segNum, float bounds){
	int derivOrder = 3;
	float accel = 5;
	Eigen::VectorXd temp_V;
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<4;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		if(d!=3){
			derivOrder=3;
		}
		else{
			derivOrder=1;
		}
		for(int i =derivOrder; i < polyOrder;i++){
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
		}
		if(d!=3){
			temp_V = (2.5/(3*accel*accel))*rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		}
		else{
			temp_V = (0.5)*rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		}
		ConPoly = ConPoly + temp_V;
	}
	// Calculate a polynomial X^2+Y^2+Z^2 < V^2 -> 
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds*bounds;
//	std::cout << ConPoly <<std::endl; 
	//Evaluate at beginning of polynomial T=0;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	//Check the conditions on end points
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}




bool QPpolyTraj::calcThrRatBound(int segNum, float bounds){
	int derivOrder = 3;
	float factor = 1/(5*5);
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<3;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		for(int i =derivOrder; i < polyOrder;i++){
			//Calculate the constant i.e x^5 derivative twice is 5*4 x^3 
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
		}
		Eigen::VectorXd temp_V = rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		ConPoly = ConPoly + temp_V;
	}
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds*bounds;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	//Check the conditions on end points
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}



bool QPpolyTraj::calcThrBound(int segNum, float bounds){
	int derivOrder = 2;
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<3;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		for(int i =derivOrder; i < polyOrder;i++){
			//Calculate the constant i.e x^5 derivative twice is 5*4 x^3 
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			 //std::cout <<factor <<std::endl;
			//Polynomial class is reversed highest order first lowest order last compared to ours.
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
			//Last vlue gravity
			if((d==2)&&(i==polyOrder-1)){
				temp_dim(polyOrder-i-1) -=9.81;
			}
		}
		Eigen::VectorXd temp_V = rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		ConPoly = ConPoly + temp_V;
	}
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds*bounds;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}


bool QPpolyTraj::calcGlobalBound(int segNum, int derivOrder){
	if(limits[derivOrder]==0){
		return true;
	}
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<3;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		for(int i =derivOrder; i < polyOrder;i++){
			//Calculate the constant i.e x^5 derivative twice is 5*4 x^3 
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			 //std::cout <<factor <<std::endl;
			//Polynomial class is reversed highest order first lowest order last compared to ours.
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
		}
		//std::cout << temp_dim << std::endl;
		//if((derivOrder ==2)&&(d==2)){
		//	temp_dim(polyOrder-derivOrder-1)-=9.81;
		//}
		Eigen::VectorXd temp_V = rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		//std::cout << "Dimension : " << d << std::endl;
		ConPoly = ConPoly + temp_V;
	}
	int bounds = limits[derivOrder]*limits[derivOrder];
	// Calculate a polynomial X^2+Y^2+Z^2 < V^2 -> 
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds;
//	std::cout << ConPoly <<std::endl; 
	//Evaluate at beginning of polynomial T=0;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	//Check the conditions on end points
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}

Eigen::MatrixXd QPpolyTraj::generateQ(int minDeriv, double time) {
    Eigen::MatrixXd Q( polyOrder,  polyOrder);
    Eigen::VectorXd poly =  basis(time, minDeriv);
    double powerOrder = 0.0;
    for (int i = 0; i <  polyOrder; i++) {
        for (int j = 0; j <  polyOrder; j++) {
            powerOrder = ((double)j + (double)i) - ((double)minDeriv * 2) + 1.0;
            if (powerOrder < 1) {
                powerOrder = 1;
            }
            Q(i, j) = poly(j) * poly(i) * time / powerOrder;
        }
    }
    return Q;
}


Eigen::MatrixXd QPpolyTraj::generateObjFun(int minDeriv)
{
    int numberSegments = segmentTimes.size();
    //Eigen::MatrixXd startQ= Eigen::MatrixXd::Zero( polyOrder,  polyOrder);
    Eigen::MatrixXd endQ( polyOrder,  polyOrder);
    Eigen::MatrixXd Qtotal = Eigen::MatrixXd::Zero(polyOrder*numberSegments, polyOrder * numberSegments);
    for (int i = 0; i < numberSegments; i++) {
        endQ = generateQ(minDeriv, segmentTimes[i]);
        //COPY Matrix Code
        Qtotal.block(i *  polyOrder, i *  polyOrder,polyOrder,  polyOrder) = endQ ;
        //startQ = endQ;
    }
    return Qtotal;
}


QP_ineq_const QPpolyTraj::genInEqConstraint( int dimension)
{
	std::cout << "entered genInEqConstraint " << std::endl;
	double dt = ineqSampleDt;
	int numConst =0;
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	QP_ineq_const  ineq_const;
	for (int i = 1; i < vertices.size();i++){
		//Count the number of inequality constraints you have
		for(int j =0; j < vertices[i].ineq_constraint.size(); j++){
			// Upper bound on the rows the sampling loop below can emit. The
			// window it actually walks starts at time-timeOffset and stops
			// at time-endOffset, so subtract the released tail here too or the
			// estimate stops being tight. It must stay an OVER-estimate.
			// spanFromStart constraints don't have a meaningful timeOffset --
			// re-derive the same width the sampling loop below will use, from
			// the CURRENT segmentTimes[i-1] (see the struct member comment).
			double toff;
			if(vertices[i].ineq_constraint[j].spanFromStart){
				toff = segmentTimes[i-1] - vertices[i].ineq_constraint[j].endOffset - dt;
			}
			else{
				toff = vertices[i].ineq_constraint[j].timeOffset
				     - vertices[i].ineq_constraint[j].endOffset;
			}
			if(toff < 0.0){ toff = 0.0; }
			numConst += (vertices[i].ineq_constraint[j].InEqDim(dimension)*toff/dt+1);
		}
	}
	int add_constr_num = 0;
	QP_ineq_const temp_ineq_constr;
	if(add_ineq_constr.size()!=0){
		temp_ineq_constr = add_ineq_constr[dimension];
		add_constr_num = temp_ineq_constr.d.rows();
		numConst +=add_constr_num;
	}

	//no inequality constraint just send the inequality constraint empty
	// Empty means that the QP optimization has no ineq_const
	//std::cout <<"num constraints: " << numConst << std::endl;
	if(numConst==0){
		ineq_const.d= Eigen::VectorXd::Zero(1);
		ineq_const.d(0) = -kEmptyIneqBound;
		ineq_const.f= Eigen::VectorXd::Zero(1);
		ineq_const.f(0) = kEmptyIneqBound;
		ineq_const.C =  Eigen::MatrixXd::Zero(1, coeffNum);
		return ineq_const;
	}
	ineq_const.d= Eigen::VectorXd::Zero(numConst);
	ineq_const.f= Eigen::VectorXd::Zero(numConst);
	ineq_const.C =  Eigen::MatrixXd::Zero(numConst, coeffNum);
	int rowNum = 0;
	for(int i = 1; i < vertices.size(); i++){
		for(int j =0; j < vertices[i].ineq_constraint.size(); j++){// Check if it is a valid constraint
			if(vertices[i].ineq_constraint[j].InEqDim(dimension)==1){
				double time = segmentTimes[i-1]; //modify to be any waypoints
				waypoint_ineq_const pon_ineq = vertices[i].ineq_constraint[j];
				// spanFromStart: open the window at a fixed, always-valid offset
				// from the segment's OWN start (dt) instead of computing it from
				// a frozen timeOffset against the CURRENT (possibly retry-grown)
				// time -- see the struct member comment for why the latter
				// silently loses coverage as segmentTimes grows.
				double toff = pon_ineq.spanFromStart ? dt : (time - pon_ineq.timeOffset);
				//Don't sample before the segment starts if the window is longer than the segment
				if(toff < 0){ toff = 0; }
				//Close the window early when a tail has been released (see
				//endOffset). tEnd <= 0 releases the constraint entirely.
				double tEnd = time - pon_ineq.endOffset;
				if(tEnd < 0){ tEnd = 0; }
				//std::cout << "End time " << time << std::endl;
				while(toff < tEnd){
					
					Eigen::VectorXd row = basis(toff, pon_ineq.derivOrder);
					ineq_const.d(rowNum) = pon_ineq.lower(dimension);
					ineq_const.f(rowNum) = pon_ineq.upper(dimension);
					ineq_const.C.block(rowNum, (i -1)*  polyOrder,1, polyOrder) = row.transpose(); 
					rowNum +=1;
					toff+=dt;
				}	
			}
		}
	}
	if(add_ineq_constr.size()!=0){
		ineq_const.d.tail(add_constr_num) = temp_ineq_constr.d;
		ineq_const.C.block(rowNum, 0,add_constr_num, coeffNum) = temp_ineq_constr.C;
		ineq_const.f.tail(add_constr_num) = temp_ineq_constr.f;
	}
	return ineq_const ;
}


int QPpolyTraj::countEqConstraintRows(){
	// Mirrors genConstraint()'s row-writing exactly (see the three branches
	// there: i==0, i==last, interior) so the pre-allocated A/b size always
	// matches what actually gets written -- no trailing all-zero rows.
	int count = 0;
	Eigen::VectorXd point;
	int n = vertices.size();
	for(int i = 0; i < n; i++){
		if(i == 0){
			count += 5; // pos + vel/accel/jerk/snap: always written (unset -> forced 0)
		}
		else if(i == n-1){
			count += 1; // pos: always
			for(int j = 1; j < 5; j++){
				if(vertices[i].getConstraint(&point, j) == 1){ count += 1; }
			}
		}
		else{
			count += 2; // position continuity (prev-end + next-start): always
			for(int j = 1; j < 5; j++){
				if(vertices[i].getConstraint(&point, j) == 1){ count += 2; }
			}
		}
	}
	return count;
}

QP_constraint QPpolyTraj::genConstraint( int dimension, int numConstraint)
{
    //do the fixed first
    //We have  polyOrder coefficient polynomials
   //#As a result, we need 20 coefficients for 2 segments.
	int add_constr_num = 0;
	QP_constraint temp_constr;
	if(add_eq_constr.size()!=0){
		temp_constr = add_eq_constr[dimension];
		add_constr_num = temp_constr.a.rows();
		numConstraint +=add_constr_num;
	}
    int coeffNum = (vertices.size() - 1) *  polyOrder;
    //#Constant conditions go through each way point and start/end with no velocity/acceleration
    Eigen::VectorXd row;
    Eigen::VectorXd point;
	//Keeps track if you fixed the derivative at the point or not
    Eigen::MatrixXd fixedDeriv = Eigen::MatrixXd::Zero(segmentTimes.size(), 5);
	Eigen::MatrixXd b = Eigen::VectorXd::Zero(numConstraint);
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numConstraint, coeffNum); //create the equality constraint 
    double time = 0;
    int rowindex = 0;
	if (numConstraint > coeffNum){
		QP_constraint qp;
		qp.a = A;
		qp.b = b;
		return qp;
	}
    for (int i = 0; i < vertices.size(); i++) {
        if (i == 0) {
            //We start at time 0 make sure your first possition is correct
            row = basis( time, 0);
            A.block(rowindex, i *  polyOrder,1, polyOrder) = row.transpose();
            vertices[i].getPos(&point);
            b(rowindex) =point(dimension);
            rowindex++;
            //Make sure that all the velocity acceleration snap jerk are 0
            for (int j = 1; j < 5; j++) {
				//Check if there is a constraint here
				if(vertices[i].getConstraint(&point,j)==1){
					// if true set the dimension
					//std::cout << "axes: " << dimension << std::endl;
					//std::cout << "DerivOrder: " << j << std::endl;
					//std::cout << "Vertex i:  " << i << std::endl;
					//std::cout << "Constraint: " << point << std::endl;
					b(rowindex) =point(dimension);//*normalFactor;
				}
				else{
						b(rowindex) = 0;
				}
				row =  basis( time,j);
				A.block(rowindex, i *  polyOrder,1, polyOrder) = row.transpose();
				rowindex++;
            }
        }
        if (i == (vertices.size() - 1)) {
			//We start at the end time make sure your possition is correct
	        time = segmentTimes[segmentTimes.size() - 1];
            row =  basis( time,0);
            A.block(rowindex,( i-1) *  polyOrder,1,  polyOrder) = row.transpose();
            vertices[i].getPos(&point);
            b(rowindex) = point(dimension);
            rowindex++;	
	        //Make sure that all the velocity acceleration snap jerk are 0
			for (int j = 1; j < 5; j++) {
				row =  basis( time,j);
				if(vertices[i].getConstraint(&point,j)==1){
					A.block(rowindex, (i - 1) *  polyOrder,1,  polyOrder) =  row.transpose();
					// if true set the dimension
					//std::cout << "Dimension constraint i: " << j <<std::endl;
					b(rowindex) = point(dimension);
					//std::cout << "axes: " << dimension << std::endl;
				//	std::cout << "DerivOrder: " << j << std::endl;
					//std::cout << "Vertex i:  " << i << std::endl;
					//std::cout << "Constraint: " << point << std::endl;
					rowindex++;
				}
				//else{
				//	b(rowindex) = 0;
				//}
			}
        }
        if((i != 0)&&(i != (vertices.size() - 1))) {
            time = segmentTimes[i - 1];
            row =  basis(time,0);
            vertices[i].getPos(&point);
            //Make sure both the prev segment ends in the location
            //And the prev segment starts in the location
            A.block(rowindex, (i - 1) *  polyOrder,1,  polyOrder) = row.transpose();
            b(rowindex) = point(dimension);
            rowindex++;
            //Making sure the next segment starts in the right area.
            row =  basis(0,0);
            A.block(rowindex, i  *  polyOrder,1,  polyOrder) = row.transpose();
            b(rowindex) = point(dimension);
            rowindex++;
			//Check to see if we have set constraints at each waypoint aside from position
            for (int j = 1; j < 5; j++) {
				if(vertices[i].getConstraint(&point,j)==1){
					row =  basis( time,j);
					// if true set the dimension at the order
					A.block(rowindex, (i - 1) *  polyOrder,1,  polyOrder) =  row.transpose();
					b(rowindex) = point(dimension);
					rowindex++;
					//Making sure the next segment starts in the right area.
					row =  basis(0,j);
					A.block(rowindex, i  *  polyOrder,1,  polyOrder) =row.transpose();
					b(rowindex) = point(dimension);
					rowindex++;
					// sets a flag to let us know we can skip the bottom continuity 
					fixedDeriv(i,j) = 1;
					/*
					std::cout << "axes: " << dimension << std::endl;
					std::cout << "DerivOrder: " << j << std::endl;
					std::cout << "Vertex i:  " << i << std::endl;
					std::cout << "Constraint: " << point << std::endl;*/
				}
            }
        }
    }
    //Continuous constraints i.e. ensure contiunity of velocity and acceleration.
    for (int i = 1; i < segmentTimes.size(); i++) {
        for (int j = 1; j < 5; j++) {
			// You only need to enfroce continuity constraints if at waypoint N a number isn't set
			//i.e at point 1 segment 0 velocity = 1 then segment 1 velcoity also = 1, and as a result 
			//s0v = 1 ;s1v =1;  =>s0v = s1v; this is no longer needed. to expliclitly state it
			if (fixedDeriv(i,j)==0){
				//endpoint of last segment
				row =  basis( segmentTimes[i - 1], j);
				A.block(rowindex, (i-1) *  polyOrder,1,  polyOrder) = row.transpose();
				//Start of Current Segment. 
				row =  basis( 0, j);
				A.block(rowindex, i *  polyOrder,1,  polyOrder) = -1*row.transpose();
				b(rowindex) = 0;
				rowindex++;
			}
        }
    }
	if(add_eq_constr.size()!=0){
		b.block(rowindex, 0,add_constr_num, 1) = temp_constr.b;
		A.block(rowindex, 0,add_constr_num, coeffNum) = temp_constr.a;
	}

	QP_constraint qp;
	qp.a = A;
	qp.b = b;
	//std::cout << A <<std::endl;
	//std::cout << b <<std::endl;
	//Eigen::MatrixXd A_squared = A* A.transpose();
	//std::cout << "Condition Number " << A_squared.norm()*A_squared.inverse().norm() << std::endl;
	return qp;
}


Eigen::MatrixXd QPpolyTraj::generateJointObjFun(int minDeriv){
   	Eigen::MatrixXd D = generateObjFun(minDeriv);
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	Eigen::MatrixXd comp  = Eigen::MatrixXd::Zero(coeffNum*dim,coeffNum*dim);
	for (int j = 0; j < dim; j++){
		//Create Copy since the quadprog changes the X&T Q  X matrix each run
        comp.block(j *  coeffNum, j *  coeffNum,coeffNum,  coeffNum) = D;
	}
	return comp;
}

QP_constraint QPpolyTraj::genJointConstraint(){
	QP_constraint joint_qp;
	std::vector<int> numConstrDim;
    	int coeffNum = (vertices.size() - 1) *  polyOrder;
	int sumConstraint = 0;
	 //Don't have pure 0's on the derivative of the last object
	int numConstraint = countEqConstraintRows();
	for(int i=0;i<dim;i++){
		int add_constr=0;
		if(add_eq_constr.size()!=0){
			add_constr = add_eq_constr[i].b.rows();
		}
		numConstrDim.push_back(add_constr+numConstraint);
		sumConstraint += numConstrDim[i];
	}	
	sumConstraint+=add_joint_eq_constr.b.rows();
	Eigen::MatrixXd btotal = Eigen::VectorXd::Zero(sumConstraint );
	Eigen::MatrixXd Atotal  = Eigen::MatrixXd::Zero(sumConstraint , coeffNum*dim ); //create the equality constraint 
	int currentRow =0;
	for (int j = 0; j < dim; j++){
		QP_constraint qp = genConstraint(j,numConstraint ); //each dimension has its unique equality constraint
		Eigen::MatrixXd A = qp.a, b = qp.b;		
		Atotal.block(currentRow,coeffNum*j,numConstrDim[j], coeffNum) = A;
		btotal.block(currentRow,0,numConstrDim[j],1) = b;
		//std::cout << "Block dim inserted: " << j << std::endl;
		currentRow+=numConstrDim[j];
	}
	//Add the joint constraints
	//std::cout << "Number of rows a: " << add_joint_eq_constr.a.rows() <<std::endl;
	if(add_joint_eq_constr.a.rows() > 0){
		Atotal.block(currentRow,0,add_joint_eq_constr.b.rows(), coeffNum*dim) = add_joint_eq_constr.a;
		btotal.block(currentRow,0,add_joint_eq_constr.b.rows(),1) = add_joint_eq_constr.b;
	}
	joint_qp.a = Atotal;
	joint_qp.b = btotal;
	return joint_qp;
}

QP_ineq_const QPpolyTraj::genInEqJointConstraint(){
	QP_ineq_const  joint_ineq_const;
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	std::vector<int> numConstrDim;
	int sumConstraint = 0;
	for(int k=0;k<dim;k++){
		int numConst = 0;
		for (int i = 1; i < vertices.size();i++){
			//Count the number of inequality constraints you have
			for(int j =0; j < vertices[i].ineq_constraint.size(); j++){
				// MUST stay identical to genInEqConstraint()'s counter: the block
				// write below is sized from numConstrDim[j] but filled with the
				// matrix genInEqConstraint(j) allocated from ITS count. If the two
				// disagree by even one row, assigning the smaller matrix into the
				// larger block trips Eigen's DenseBase::resize() assertion.
				// That means matching both the sampling step (ineqSampleDt), the
				// released tail (endOffset), AND spanFromStart's re-derivation.
				double toff;
				if(vertices[i].ineq_constraint[j].spanFromStart){
					toff = segmentTimes[i-1] - vertices[i].ineq_constraint[j].endOffset - ineqSampleDt;
				}
				else{
					toff = vertices[i].ineq_constraint[j].timeOffset
					     - vertices[i].ineq_constraint[j].endOffset;
				}
				if(toff < 0.0){ toff = 0.0; }
				numConst += (vertices[i].ineq_constraint[j].InEqDim(k)*toff/ineqSampleDt+1);
			}
		}
		if(add_ineq_constr.size()!=0){
			numConst +=add_ineq_constr[k].d.rows();
		}
		numConstrDim.push_back(numConst) ;
		sumConstraint += numConstrDim[k];
	}	
	sumConstraint+=add_joint_ineq_constr.d.rows();	
	if(sumConstraint ==0){
		joint_ineq_const.d= Eigen::VectorXd::Zero(1);
		joint_ineq_const.d(0) = -kEmptyIneqBound;
		joint_ineq_const.f= Eigen::VectorXd::Zero(1);
		joint_ineq_const.f(0) = kEmptyIneqBound;
		joint_ineq_const.C =  Eigen::MatrixXd::Zero(1, coeffNum*dim);
		return joint_ineq_const;		
	}
	joint_ineq_const.C = Eigen::MatrixXd::Zero(sumConstraint , coeffNum*dim );
	joint_ineq_const.d = Eigen::VectorXd::Zero(sumConstraint);
	joint_ineq_const.f = Eigen::VectorXd::Zero(sumConstraint);	
	int currentRow =0;
	for (int j = 0; j < dim; j++){
		if(numConstrDim[j]!=0){
			QP_ineq_const temp_ineq_qp = genInEqConstraint(j ); //each dimension has its unique equality constraint
			joint_ineq_const.C.block(currentRow,coeffNum*j,numConstrDim[j], coeffNum) = temp_ineq_qp.C;
			joint_ineq_const.d.block(currentRow,0,numConstrDim[j],1) = temp_ineq_qp.d;
			joint_ineq_const.f.block(currentRow,0,numConstrDim[j],1) = temp_ineq_qp.f;
			currentRow+=numConstrDim[j];
		}
	}
	if(add_joint_ineq_constr.d.rows()!=0){
		//Add the joint constraints
		joint_ineq_const.C.block(currentRow,0,add_joint_ineq_constr.d.rows(), coeffNum*dim) = add_joint_ineq_constr.C;
		joint_ineq_const.d.block(currentRow,0,add_joint_ineq_constr.d.rows(),1) = add_joint_ineq_constr.d;
		joint_ineq_const.f.block(currentRow,0,add_joint_ineq_constr.d.rows(),1) = add_joint_ineq_constr.f;
	}
	return joint_ineq_const;
}

Eigen::MatrixXd QPpolyTraj::calculateTrajectory( int Order, double dt){	
    int numSeg = segmentTimes.size();
	//Calculates the fulltimes of each segment
	//Example if you have 2 segmes 1,0.5
	//Fultime would be 0, 1, 1.5
	Eigen::VectorXd fullTime(numSeg+1);
	//Calculate the length of the segment inefficinet figure it out
	int length = 0;
    for (int k = 0; k < segmentTimes.size(); k++) {
        for (int j = 0; j < segmentTimes[k]  /0.01; j++) {
			length++;				
		}	
    }
    int startTime = 0;
	//CONDITION CHECK THAT THERE WAS A SUCCESFUL WORK ON ALL 4 AXISES
	if (!(traj_valid[0]&&traj_valid[1]&&traj_valid[2] &&traj_valid[3])){
		//IF any of the above is false
		// std::cout << "FAILURE GENERATING PATH" <<std::endl;
		return Eigen::MatrixXd::Zero(length,dim);
	}
	Eigen::MatrixXd trajectory(length,dim);
	int rownum = 0;
	double culsum = 0;
    for (int k = 0; k < segmentTimes.size(); k++) {
        for (int j = 0; j < segmentTimes[k]  /dt; j++) {
			double time = j*dt;
			//Normalize the time based on the fulltime
			Eigen::VectorXd power = basis(time,Order);
			for (int d = 0; d < dim; d++) {
				int count = 0;
				culsum = 0;
				for (int p = (polyOrder*k); p < polyOrder*(k+1); p++) {
					culsum = culsum + power(count) * coeffSolved(p,d);
					count = count +1;
					}
				trajectory(rownum,d)= culsum;
			}
			rownum++;				
		}	
    }
	return trajectory;
}

Eigen::VectorXd QPpolyTraj::basis(double time, int derivative) {
    if (derivative > 4) {
        throw 10;
    }
    // compute all the powers of time
    std::vector<double> t(polyOrder);
	//create a list of powers temporarily so you don't have to recalculate each time
    for (int i = 0; i <polyOrder; i++) {
        t[i] = pow(time, i);
    }
	// Alocate coefficitions of your basis
    Eigen::VectorXd b(polyOrder);
	// Calculate coefficients 
    for (int i = 0; i < polyOrder; i++) {
		double power = i - derivative;
		double coeff = 1;
		//if your power < derivative it must be 0 i.e. x'' = 0; or x^2''' = 0;
		if (power <0){
			b[i] = 0;
		}
		else{
			//claculate n!/(n-derivative)!
			 for (int j = i; j > i-derivative; j--) {
				coeff = coeff*j;
			 }
			b[i] = coeff*t[power];
		}
    }
    return b;
}
