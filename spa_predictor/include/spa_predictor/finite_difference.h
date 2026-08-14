#ifndef _spa_predictor_finite_difference_h
#define _spa_predictor_finite_difference_h

// Simple backward finite difference of a scalar signal against
// irregularly-timed samples -- shared building block for turning a
// measured signal into its own time-derivative, e.g. for feeding
// SpaPredictor::addMeasurement()'s optional `accel` argument:
//   spa_axis_node.cpp:   one instance, velocity -> acceleration (position
//                        is already measured, VehicleOdometry.velocity is
//                        already measured, only ONE differentiation needed)
//   spa_angle_node.cpp:  two CHAINED instances, angle -> rate -> accel
//                        (there is no directly-measured Euler angle rate
//                        field to difference just once -- VehicleOdometry
//                        only carries body-frame angular_velocity, which
//                        is NOT the Euler rate outside the small-angle/
//                        single-axis case)
//
// No ROS dependency -- pure arithmetic, same reasoning as SpaPredictor
// itself.
class FiniteDifference
{
public:
	// Writes the derivative estimate of `value` at time `t` into
	// *derivative_out IF one is available (needs a valid previous sample --
	// i.e. not the first call, and dt > 0); otherwise *derivative_out is
	// left UNTOUCHED, so callers should pre-set it to a sentinel (e.g.
	// std::numeric_limits<double>::quiet_NaN()) to signal "not available
	// yet" downstream -- exactly the sentinel SpaPredictor::addMeasurement()
	// itself expects for its optional accel argument.
	void Update(double t, double value, double* derivative_out)
	{
		if(have_){
			double dt = t - lastT_;
			if(dt > 1e-9){
				*derivative_out = (value - lastValue_) / dt;
			}
		}
		lastValue_ = value;
		lastT_ = t;
		have_ = true;
	}

private:
	bool have_ = false;
	double lastValue_ = 0.0;
	double lastT_ = 0.0;
};

#endif
