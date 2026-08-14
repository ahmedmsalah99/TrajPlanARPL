std_msgs/Header header

# False until the predictor has completed its first mode-detection pass and
# has a state to propagate from -- all fields below are meaningless while
# false (predicted == filtered == 0, sigma == -1 for every bin).
bool initialized

# The time (seconds, SAME clock/epoch as the INPUT measurements fed to
# addMeasurement() -- e.g. PX4 timestamp-derived seconds, NOT necessarily
# ROS time) that horizon_s/predicted_value/predicted_velocity are relative
# to: SpaPredictor::lastMeasurementTime() at the moment this message was
# built. Deliberately separate from header.stamp, which is just "when this
# ROS message happened to be published" (up to ~1/publish_rate_hz of ROS
# wall-clock lag behind the last actual measurement, and on a clock that
# may not even agree with the measurement timestamps' epoch/rate if
# use_sim_time or a PX4/ROS time offset is in play). Anything computing
# t_target = made_at_t + horizon_s to align a prediction against later
# ground truth (e.g. an offline evaluation script) MUST use this field,
# not header.stamp -- using header.stamp there silently introduces a
# timing error of unknown, possibly time-varying size.
float64 made_at_t

# SpaPredictor::predict(0.0, ...): the Kalman-filtered CURRENT value, i.e.
# the SPA used purely as a smoother with no lead time. Distinct from the raw
# measurement -- see spa_predictor.h's class comment.
float64 filtered_value

# The RAW acceleration measurement fed to addMeasurement() alongside the
# most recent position/angle sample (see spa_predictor.h's class comment
# on the estimator's second correction channel) -- an INPUT the estimator
# was corrected against, not a model output (unlike predicted_value/
# predicted_velocity below; do not confuse the two). NaN whenever no
# acceleration reading was available for that sample (e.g. still
# bootstrapping the finite-difference chain -- see
# spa_axis_node.cpp/spa_angle_node.cpp's FiniteDifference members).
# Published purely for visibility into what the estimator is actually
# being corrected against -- this value is derived by finite-differencing
# an already-noisy measured signal (one differentiation for x/y/heave,
# two CHAINED differentiations for roll/pitch), so expect it to be
# considerably noisier than filtered_value/predicted_value.
float64 measured_accel

# Currently tracked modes (see SpaPredictor::currentModes()), largest
# amplitude first. Parallel arrays, same length.
float64[] mode_freq_hz
float64[] mode_amplitude
float64[] mode_phase_rad

# Static-offset state (SpaPredictor::currentOffset()) -- the slow-drift/DC
# component the mode search deliberately excludes (see f_min_hz).
float64 offset

# Predictions and their empirical accuracy, at the horizons the publishing
# node requested via predictAndAssess() -- parallel arrays, same length.
# sigma_s is NaN wherever that horizon bin has no resolved self-assessment
# samples yet; treat NaN as "unknown", not "zero error".
float64[] horizon_s
float64[] predicted_value
# The signal's time-derivative at each horizon (e.g. heave rate, not just
# heave) -- from the SAME propagation as predicted_value, see
# SpaPredictor::predictWithVelocity()'s comment. Needed for AHC-style
# relative impact velocity (v_impact = v_land - v_target), not just
# position matching. No separate self-assessment -- see sigma_s's caveat,
# which applies to predicted_value only; there is no independent velocity
# measurement to check this against.
float64[] predicted_velocity
float64[] sigma_s

# Diagnostic: count of measurements dropped for arriving out of order (see
# SpaPredictor::droppedSamples()). Persistently nonzero suggests an upstream
# timestamping problem, not something this node can fix.
int32 dropped_samples
