std_msgs/Header header

# False until the predictor has completed its first mode-detection pass and
# has a state to propagate from -- all fields below are meaningless while
# false (predicted == filtered == 0, sigma == -1 for every bin).
bool initialized

# SpaPredictor::predict(0.0, ...): the Kalman-filtered CURRENT value, i.e.
# the SPA used purely as a smoother with no lead time. Distinct from the raw
# measurement -- see spa_predictor.h's class comment.
float64 filtered_value

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
float64[] sigma_s

# Diagnostic: count of measurements dropped for arriving out of order (see
# SpaPredictor::droppedSamples()). Persistently nonzero suggests an upstream
# timestamping problem, not something this node can fix.
int32 dropped_samples
