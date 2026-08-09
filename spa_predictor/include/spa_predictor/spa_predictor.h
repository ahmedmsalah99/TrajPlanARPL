#ifndef _spa_predictor_h
#define _spa_predictor_h
#include <Eigen/Eigen>
#include <vector>
#include <deque>

// Signal Prediction Algorithm (SPA), per Abujoub, McPhee & Irani, "Methodologies
// for Landing Autonomous Aerial Vehicles on Maritime Vessels" (2020), Sec. 3.1.
//
// Predicts a single scalar periodic-ish signal (e.g. pad heave) forward by an
// arbitrary, continuously-adjustable horizon. Three stages, matching the paper:
//   1. Mode detection (periodic, every t_fft seconds): FFT the recent window,
//      peak-pick up to max_modes dominant frequencies.
//   2. Estimation (every sample): a bank of harmonic-oscillator states, one
//      per detected mode, plus a static-offset state, tracked with a steady-
//      state Kalman predictor (paper eq. 6).
//   3. Prediction (on demand, any horizon): closed-form propagation of the
//      oscillator states -- exact for a sum of sinusoids, so predict() is
//      smooth and well-defined for any horizon, not just a sample-grid step.
//      This is the "flexible prediction horizon" property the whole design
//      exists for.
//
// No ROS/rclcpp dependency -- feed it (time, value) pairs from whatever
// source (a ROS subscription callback, a test harness, a log file) via
// addMeasurement(); the caller owns timestamps and units. Time is an
// arbitrary monotonic seconds value (e.g. rclcpp::Time(...).seconds());
// only differences matter, never an absolute epoch.
//
// Deliberately restricted to ONE scalar signal per instance -- the paper
// runs "two SPAs in parallel" for roll and pitch; a third instance covers
// heave. Instantiate one SpaPredictor per signal.
class SpaPredictor {
public:
	struct Config {
		// --- Mode detection ---
		// Observation window for the FFT (s). Must span several periods of
		// the slowest mode you want to resolve -- frequency resolution is
		// 1/t_fft. Re-run at this cadence.
		double t_fft = 25.0;
		// Uniform resample grid used ONLY for the FFT's frequency search (s).
		// Amplitude/phase are refit afterwards against the raw, unresampled
		// buffer (see detectModes()'s comment), so this only needs to be fine
		// enough to avoid aliasing f_max, not fine enough for the final fit.
		double fft_grid_dt = 0.1;
		// Mode search band (Hz). f_min excludes near-DC content -- WITHOUT
		// this, slow drift in the upstream position estimate (e.g. vision
		// re-estimating the target as the vehicle approaches) gets picked up
		// as a spurious "mode" and confidently extrapolated, rather than
		// being absorbed by the offset state where it belongs. f_max guards
		// against aliasing given fft_grid_dt.
		double f_min_hz = 0.03;
		double f_max_hz = 2.0;
		// Peak acceptance threshold, as a fraction of the largest in-band
		// peak's magnitude (paper's "peak detection sensitivity mu").
		double peak_sensitivity = 0.15;
		// Upper bound on simultaneously-tracked modes -- bounds the
		// observer's state size (2*max_modes + 1) and the cost of the
		// Riccati solve run at each mode-set rebuild.
		int max_modes = 4;

		// --- Estimation (steady-state Kalman predictor, paper eq. 6) ---
		// Process noise, per oscillator state (position- and velocity-like
		// halves of each mode's 2x2 block). Larger = the observer trusts
		// fresh measurements more and adapts amplitude/phase faster;
		// smaller = smoother, slower-adapting.
		double process_noise_osc = 1e-4;
		// Process noise on the static-offset state. Deliberately larger than
		// process_noise_osc by default -- this is what lets the offset track
		// slow drift (a moving mean) instead of forcing that drift to alias
		// into the oscillator modes.
		double process_noise_offset = 5e-3;
		// Measurement noise variance (sensor units^2).
		double measurement_noise = 0.01;
		// Nominal sample period (s) the steady-state gain is solved for. See
		// the .cpp's stepEstimator() comment for why a per-sample Riccati
		// solve isn't done: the same fixed gain is reused for every sample's
		// correction, while the STATE PROPAGATION itself always uses the
		// sample's true, possibly-irregular dt. Set close to the expected
		// measurement rate (e.g. 1/30 for a 30 Hz AprilTag feed).
		double nominal_dt = 1.0 / 30.0;
		int riccati_max_iter = 500;
		double riccati_tol = 1e-10;

		// --- Self-assessment ---
		// Upper bound on the horizons tracked by sigmaAtHorizon() (s).
		double sigma_max_horizon = 8.0;
		// Bin width for the horizon axis (s).
		double sigma_bin_s = 0.5;
		// EWMA smoothing factor for each bin's mean squared error, applied
		// per NEW SAMPLE landing in that bin (not per unit time) -- so it
		// naturally adapts faster for densely-sampled horizons (short h, many
		// predictions land there) and slower for sparse ones (long h).
		double sigma_ewma_alpha = 0.05;
	};

	struct Mode {
		double freq_hz;
		double amplitude;
		double phase_rad; // s(t) contribution = amplitude*sin(2*pi*freq_hz*t + phase_rad)
	};

	// Split into two overloads (rather than one ctor with `= Config()`) --
	// a default argument referencing a NESTED class's implicit default
	// constructor isn't usable yet at its point of declaration inside the
	// ENCLOSING class's own body (Config's default member initializers
	// aren't "complete" there), even though Config itself is already fully
	// defined above. See SpaPredictor()'s definition in the .cpp, which
	// delegates to the Config& overload entirely outside this class body,
	// where both types are unambiguously complete.
	SpaPredictor();
	explicit SpaPredictor(const Config& cfg);

	// Feed one measurement at absolute time t (seconds, monotonic, caller's
	// clock). Samples must arrive in non-decreasing t; a sample with
	// t <= lastMeasurementTime() is dropped (logged via droppedSamples()).
	void addMeasurement(double t, double value);

	// True once at least one mode-detection pass has completed and the
	// estimator has a state to propagate from. predict()/currentModes() are
	// meaningless before this.
	bool initialized() const { return initialized_; }

	// Predict the signal's value at (lastMeasurementTime() + horizon_s).
	// horizon_s may be negative (recent past) for self-checks, zero (the
	// filtered current estimate -- this IS the smoothing use case, see the
	// class comment), or any positive lead time; the closed-form propagation
	// is exact for any real horizon_s, not just multiples of a sample step.
	// Returns false (value_out untouched) if !initialized().
	bool predict(double horizon_s, double* value_out) const;

	// Convenience: predict at every horizon in horizons_s, in order. Also
	// records each prediction for self-assessment (see sigmaAtHorizon()) --
	// this is the ONLY way predictions enter the self-assessment buffer, so
	// call this (not repeated predict()) if you want sigmaAtHorizon() to
	// reflect the horizons you actually care about.
	std::vector<double> predictAndAssess(const std::vector<double>& horizons_s);

	// Empirical RMS error of past predictions made at ~this horizon,
	// measured by comparing each recorded prediction (see
	// predictAndAssess()) against the measurement that later landed near its
	// target time. NaN if this horizon bin has no resolved samples yet --
	// treat NaN as "unknown", not "zero error"; callers requiring a
	// confidence bound before acting (e.g. a Go decision) must handle it
	// explicitly rather than let it silently compare true.
	double sigmaAtHorizon(double horizon_s) const;

	// Diagnostics.
	std::vector<Mode> currentModes() const { return modes_; }
	double currentOffset() const { return xOff(); }
	double lastMeasurementTime() const { return lastT_; }
	int droppedSamples() const { return droppedSamples_; }

private:
	Config cfg_;
	bool initialized_ = false;
	double lastT_ = 0.0;
	int droppedSamples_ = 0;

	// Raw (t, value) ring buffer covering the last cfg_.t_fft seconds --
	// input to BOTH mode detection (via a resampled copy) and the
	// least-squares amplitude/phase refit (directly, unresampled -- see the
	// .cpp). Pruned to the window on every addMeasurement().
	std::deque<double> bufT_;
	std::deque<double> bufV_;
	double nextModeDetectT_ = -1.0; // lastT_ at/after which the next FFT pass runs

	// Observer state: [x_{1,1}, x_{1,2}, ..., x_{N,1}, x_{N,2}, x_off],
	// size 2*modes_.size()+1. x_{i,1} = amplitude*sin(theta_i) (the mode's
	// direct contribution to the signal); x_{i,2} = its time derivative.
	Eigen::VectorXd state_;
	Eigen::VectorXd gainL_; // steady-state predictor gain (paper eq. 6's L)
	std::vector<Mode> modes_; // frequencies this state_/gainL_ were built for

	double xOff() const { return state_.size() > 0 ? state_(state_.size() - 1) : 0.0; }

	// -- mode detection --
	void maybeRunModeDetection();
	std::vector<Mode> detectModes() const;
	// Rebuilds state_/gainL_/modes_ for a new mode set, warm-starting state_
	// from the modes' own (amplitude, phase) at the current time so the
	// estimator doesn't restart from zero every t_fft interval.
	void rebuildObserver(const std::vector<Mode>& newModes);

	// -- estimation --
	// Advances state_ by dt via the exact closed-form per-block propagation,
	// then applies gainL_'s correction against `value` (paper eq. 6). If
	// haveMeasurement is false, propagates only (handles measurement
	// dropouts without corrupting the state with a fabricated correction).
	void stepEstimator(double dt, bool haveMeasurement, double value);

	// Exact 2x2 continuous-time state transition for one oscillator block at
	// angular frequency w, elapsed time dt (see .cpp for the derivation --
	// this is the closed-form rotation-like exponential of [[0,1],[-w^2,0]]).
	static Eigen::Matrix2d oscillatorTransition(double w, double dt);

	// Value contribution of state_ at the CURRENT state (i.e. horizon 0):
	// sum of each mode's x_{i,1} plus the offset state.
	double sampleState(const Eigen::VectorXd& s) const;

	// Propagates a COPY of state_ forward by horizon_s (exact, closed-form,
	// any real horizon) and returns its sampled value -- the shared
	// implementation behind predict()/predictAndAssess().
	double propagateAndSample(double horizon_s) const;

	// -- self-assessment --
	struct PendingPrediction {
		double madeAtT;
		double targetT;
		double predictedValue;
	};
	std::deque<PendingPrediction> pending_;
	// Per-horizon-bin EWMA of squared error and a resolved-sample count
	// (the latter only to distinguish "never resolved" -> NaN from "resolved
	// with ~zero error").
	std::vector<double> sigmaBinsMse_;
	std::vector<int> sigmaBinsCount_;
	int horizonBin(double horizon_s) const;
	void resolvePending(double t, double value);
};

#endif
