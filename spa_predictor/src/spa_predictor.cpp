#include <spa_predictor/spa_predictor.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
}

SpaPredictor::SpaPredictor(const Config& cfg) : cfg_(cfg)
{
	int nBins = std::max(1, static_cast<int>(std::ceil(cfg_.sigma_max_horizon / cfg_.sigma_bin_s)) + 1);
	sigmaBinsMse_.assign(nBins, 0.0);
	sigmaBinsCount_.assign(nBins, 0);
}

void SpaPredictor::addMeasurement(double t, double value)
{
	if(initialized_ && t <= lastT_){
		droppedSamples_ += 1;
		return;
	}

	// Self-assessment: check this measurement against any pending
	// predictions whose target time has now arrived, BEFORE folding it into
	// the estimator -- this is purely a comparison against the past, must
	// not influence the update below.
	resolvePending(t, value);

	// Raw buffer, used by both mode detection (resampled copy) and the
	// amplitude/phase refit (direct). Keep exactly the last t_fft seconds.
	bufT_.push_back(t);
	bufV_.push_back(value);
	while(!bufT_.empty() && (t - bufT_.front()) > cfg_.t_fft){
		bufT_.pop_front();
		bufV_.pop_front();
	}

	if(!initialized_){
		// First-ever sample: nothing to propagate from yet. Seed a
		// zero-mode observer (pure offset = this measurement) so predict()
		// is at least well-defined immediately (constant-value prediction)
		// while the buffer fills toward the first real mode-detection pass.
		lastT_ = t;
		nextModeDetectT_ = t + cfg_.t_fft;
		rebuildObserver({}); // state_ becomes size-1 ([x_off]); warm-started below
		state_(0) = value;
		initialized_ = true;
		return;
	}

	double dt = t - lastT_;
	lastT_ = t;
	stepEstimator(dt, /*haveMeasurement=*/true, value);

	maybeRunModeDetection();
}

// ---------------------------------------------------------------------------
// Estimation
// ---------------------------------------------------------------------------

Eigen::Matrix2d SpaPredictor::oscillatorTransition(double w, double dt)
{
	// Closed-form exp(A*dt) for A = [[0,1],[-w^2,0]] -- the state matrix of a
	// simple harmonic oscillator x1'' = -w^2 x1 written as a first-order
	// system (x1, x2=x1'). For x1(t) = a*sin(w*t+phi), x2(t) = a*w*cos(w*t+phi),
	// propagating by dt is exactly a rotation of (x1, x2/w) by angle w*dt:
	//   x1' = cos(w dt) x1 + sin(w dt)/w * x2
	//   x2' = -w sin(w dt) x1 + cos(w dt) x2
	// (substitute x1=a sin(theta), x2=a w cos(theta), theta=w t+phi, and use
	// the angle-sum identities to confirm x1' = a sin(theta + w dt), etc.)
	// w is always bounded away from 0 here (modes are rejected below
	// cfg_.f_min_hz > 0), but guard the division defensively anyway.
	double w_safe = (std::fabs(w) > 1e-9) ? w : 1e-9;
	double c = std::cos(w * dt);
	double s = std::sin(w * dt);
	Eigen::Matrix2d T;
	T << c, s / w_safe,
	     -w * s, c;
	return T;
}

double SpaPredictor::sampleState(const Eigen::VectorXd& s) const
{
	// Sum of each mode's x_{i,1} (its direct signal contribution) plus the
	// offset state (always the last entry).
	double v = 0.0;
	int n = static_cast<int>(modes_.size());
	for(int i = 0; i < n; i++){
		v += s(2 * i);
	}
	v += s(s.size() - 1);
	return v;
}

void SpaPredictor::stepEstimator(double dt, bool haveMeasurement, double value)
{
	int n = static_cast<int>(modes_.size());
	int sz = 2 * n + 1;
	if(state_.size() != sz){
		// Defensive -- should only happen if rebuildObserver() wasn't called
		// after a modes_ change, which would be a bug elsewhere in this file.
		state_ = Eigen::VectorXd::Zero(sz);
	}

	// Propagate every block by the sample's TRUE dt (exact, regardless of
	// how irregular the sampling is -- e.g. an AprilTag feed with jittery
	// arrival times).
	Eigen::VectorXd propagated(sz);
	for(int i = 0; i < n; i++){
		double w = kTwoPi * modes_[i].freq_hz;
		Eigen::Vector2d block = state_.segment<2>(2 * i);
		propagated.segment<2>(2 * i) = oscillatorTransition(w, dt) * block;
	}
	propagated(sz - 1) = state_(sz - 1); // offset state has no dynamics

	if(!haveMeasurement){
		state_ = propagated;
		return;
	}

	// Paper eq. 6: x_{k+1} = Psi*x_k + L*(s_k - C*x_k). This is the
	// PREDICTOR form (next state as a function of the CURRENT, not
	// current-corrected, estimate) -- see rebuildObserver()'s comment for
	// the matching Riccati/gain derivation. The innovation is measured
	// against C*x_k (sampleState(state_), i.e. BEFORE this step's
	// propagation), not against the just-propagated state.
	double innovation = value - sampleState(state_);
	state_ = propagated + gainL_ * innovation;
}

// ---------------------------------------------------------------------------
// Mode detection
// ---------------------------------------------------------------------------

void SpaPredictor::maybeRunModeDetection()
{
	if(lastT_ < nextModeDetectT_){
		return;
	}
	nextModeDetectT_ = lastT_ + cfg_.t_fft;

	// Need enough history to actually resolve f_min_hz -- otherwise skip
	// this pass and try again next interval (the buffer keeps growing
	// meanwhile via addMeasurement()'s prune-to-window logic).
	if(bufT_.empty() || (bufT_.back() - bufT_.front()) < 0.5 * cfg_.t_fft){
		return;
	}

	std::vector<Mode> newModes = detectModes();
	// Only rebuild when this pass actually found something. An empty result
	// (no peak cleared peak_sensitivity, or the grid was degenerate) could
	// mean "genuinely calm right now", but it could just as easily be one
	// noisy/unlucky window -- and rebuildObserver({}) would silently
	// collapse straight back to a pure-offset model, discarding whatever
	// modes were already converged. Keeping the current mode set on an
	// inconclusive pass is the safer default: a real, sustained transition
	// to calm just means later passes keep coming back empty too, and
	// nothing here currently acts on that (no forced decay of stale modes)
	// -- worth revisiting (e.g. require several consecutive empty passes
	// before actually resetting) if a real calm period needs to be detected
	// as such, not just ridden out on stale modes.
	if(!newModes.empty()){
		rebuildObserver(newModes);
	}
}

std::vector<SpaPredictor::Mode> SpaPredictor::detectModes() const
{
	// Step 1: resample the raw buffer onto a uniform grid at fft_grid_dt via
	// linear interpolation, purely to run a DFT for FREQUENCY search
	// (amplitude/phase are refit against the raw buffer afterwards -- see
	// below -- so this resampling only needs to be good enough to locate
	// peaks, not to preserve amplitude).
	double tStart = bufT_.front();
	double tEnd = bufT_.back();
	int M = static_cast<int>(std::floor((tEnd - tStart) / cfg_.fft_grid_dt)) + 1;
	if(M < 8){
		return {};
	}
	std::vector<double> grid(M);
	{
		size_t srcIdx = 0;
		for(int k = 0; k < M; k++){
			double tk = tStart + k * cfg_.fft_grid_dt;
			while(srcIdx + 1 < bufT_.size() && bufT_[srcIdx + 1] < tk){ srcIdx++; }
			if(srcIdx + 1 >= bufT_.size()){
				grid[k] = bufV_.back();
				continue;
			}
			double t0 = bufT_[srcIdx], t1 = bufT_[srcIdx + 1];
			double v0 = bufV_[srcIdx], v1 = bufV_[srcIdx + 1];
			double a = (t1 > t0) ? (tk - t0) / (t1 - t0) : 0.0;
			a = std::clamp(a, 0.0, 1.0);
			grid[k] = v0 + a * (v1 - v0);
		}
	}

	double mean = 0.0;
	for(double v : grid){ mean += v; }
	mean /= M;

	// Hann window, to reduce spectral leakage in the peak search (amplitude
	// is NOT read off this windowed spectrum -- only frequency location --
	// so the window's coherent-gain amplitude bias doesn't matter here).
	std::vector<double> windowed(M);
	for(int k = 0; k < M; k++){
		double w = 0.5 - 0.5 * std::cos(kTwoPi * k / (M - 1));
		windowed[k] = (grid[k] - mean) * w;
	}

	// Direct DFT (real cosine/sine correlation), magnitude spectrum only,
	// over the frequency band of interest. M is at most a few hundred here
	// (t_fft ~ tens of seconds at fft_grid_dt ~0.1s), so this O(M * #bins)
	// direct sum is negligible cost run once per t_fft interval -- no FFT
	// library dependency needed.
	double dtGrid = cfg_.fft_grid_dt;
	double fResolution = 1.0 / (M * dtGrid);
	int kMin = std::max(1, static_cast<int>(std::floor(cfg_.f_min_hz / fResolution)));
	int kMax = std::min(M / 2, static_cast<int>(std::ceil(cfg_.f_max_hz / fResolution)));
	if(kMax - kMin < 2){
		return {};
	}
	std::vector<double> mag(kMax - kMin + 1, 0.0);
	for(int k = kMin; k <= kMax; k++){
		double w = kTwoPi * k * fResolution;
		double xc = 0.0, xs = 0.0;
		for(int n = 0; n < M; n++){
			double phase = w * (n * dtGrid);
			xc += windowed[n] * std::cos(phase);
			xs += windowed[n] * std::sin(phase);
		}
		mag[k - kMin] = std::hypot(xc, xs);
	}

	// Step 2: peak-pick local maxima above cfg_.peak_sensitivity * (largest
	// in-band peak), take up to max_modes by magnitude, and refine each
	// peak's frequency via parabolic interpolation across its 3 neighboring
	// bins -- raw bin resolution alone (1/t_fft) is too coarse: at a 25s
	// window that is 0.04 Hz, which over a multi-second prediction horizon
	// integrates to tens of degrees of phase error from frequency
	// quantization alone. Parabolic interpolation recovers sub-bin accuracy
	// essentially for free.
	double maxMag = 0.0;
	for(double m : mag){ maxMag = std::max(maxMag, m); }
	if(maxMag <= 0.0){
		return {};
	}
	struct Peak { int idx; double mag; };
	std::vector<Peak> peaks;
	for(int i = 1; i < static_cast<int>(mag.size()) - 1; i++){
		if(mag[i] > mag[i - 1] && mag[i] > mag[i + 1] && mag[i] >= cfg_.peak_sensitivity * maxMag){
			peaks.push_back({i, mag[i]});
		}
	}
	std::sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b){ return a.mag > b.mag; });
	if(static_cast<int>(peaks.size()) > cfg_.max_modes){
		peaks.resize(cfg_.max_modes);
	}

	std::vector<double> freqs;
	for(const auto& p : peaks){
		double yL = mag[p.idx - 1], yC = mag[p.idx], yR = mag[p.idx + 1];
		double denom = (yL - 2.0 * yC + yR);
		double delta = (std::fabs(denom) > 1e-12) ? 0.5 * (yL - yR) / denom : 0.0;
		delta = std::clamp(delta, -0.5, 0.5);
		double kRefined = (p.idx + kMin) + delta;
		freqs.push_back(kRefined * fResolution);
	}

	// Step 3: refit amplitude/phase/offset for ALL selected frequencies
	// SIMULTANEOUSLY via linear least squares against the RAW (unresampled,
	// unwindowed) buffer -- avoids the Hann window's amplitude/coherent-gain
	// correction entirely and is more accurate than reading amplitude/phase
	// off the DFT bin, since the buffer's true sample times are used
	// directly. Model: value(t) = offset + sum_i [a_i*cos(w_i t) + b_i*sin(w_i t)],
	// solved for (a_i, b_i, offset), then amplitude = hypot(a_i,b_i),
	// phase = atan2(a_i, b_i) so that a_i*cos+b_i*sin = amplitude*sin(w t + phase)
	// (matches this class's sin-based convention throughout).
	int n = static_cast<int>(freqs.size());
	if(n == 0){
		return {};
	}
	int P = static_cast<int>(bufT_.size());
	Eigen::MatrixXd design(P, 2 * n + 1);
	Eigen::VectorXd rhs(P);
	for(int r = 0; r < P; r++){
		double t = bufT_[r];
		for(int i = 0; i < n; i++){
			double w = kTwoPi * freqs[i];
			design(r, 2 * i) = std::cos(w * t);
			design(r, 2 * i + 1) = std::sin(w * t);
		}
		design(r, 2 * n) = 1.0;
		rhs(r) = bufV_[r];
	}
	Eigen::VectorXd sol = design.colPivHouseholderQr().solve(rhs);

	std::vector<Mode> result;
	for(int i = 0; i < n; i++){
		double a = sol(2 * i), b = sol(2 * i + 1);
		double amplitude = std::hypot(a, b);
		double phase = std::atan2(a, b); // a*cos+b*sin = amplitude*sin(w t + phase)
		result.push_back({freqs[i], amplitude, phase});
	}
	// Largest amplitude first -- cosmetic/diagnostic ordering only, doesn't
	// affect the observer (state_ layout follows this order, consistently
	// rebuilt each time).
	std::sort(result.begin(), result.end(), [](const Mode& a, const Mode& b){
		return a.amplitude > b.amplitude;
	});
	return result;
}

void SpaPredictor::rebuildObserver(const std::vector<Mode>& newModes)
{
	double prevOffset = (state_.size() > 0) ? xOff() : 0.0;

	modes_ = newModes;
	int n = static_cast<int>(modes_.size());
	int sz = 2 * n + 1;

	// Warm-start: seed each mode's state directly from its just-identified
	// (amplitude, phase) evaluated at the current time (paper: "used to
	// initialize an observer model with a new set of parameters"), so
	// rebuilding the mode set doesn't reset the estimator to zero and lose
	// however many samples of convergence it already had.
	Eigen::VectorXd newState(sz);
	for(int i = 0; i < n; i++){
		double w = kTwoPi * modes_[i].freq_hz;
		double theta = w * lastT_ + modes_[i].phase_rad;
		newState(2 * i) = modes_[i].amplitude * std::sin(theta);
		newState(2 * i + 1) = modes_[i].amplitude * w * std::cos(theta);
	}
	newState(sz - 1) = prevOffset;
	state_ = newState;

	// Steady-state predictor-form Kalman gain (paper eq. 6's L), solved via
	// fixed-point iteration on the associated discrete Riccati equation at
	// the CONFIGURED nominal sample period, not the true per-sample dt.
	// Deliberate approximation: re-solving a Riccati equation on every
	// irregular-dt sample would be needlessly expensive for a predictor
	// gain that only needs to be roughly right; state PROPAGATION (in
	// stepEstimator()) always uses the sample's true dt regardless, so this
	// only affects how aggressively each correction is weighted, not the
	// dynamics themselves. Revisit if the real feed's jitter turns out to
	// be large relative to nominal_dt.
	//
	// System (per current mode set): Psi = blockdiag(oscillatorTransition_i,
	// ..., 1), C = [1,0, 1,0, ..., 1] (picks each mode's x_{i,1} plus the
	// offset state), scalar output.
	Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(sz, sz);
	for(int i = 0; i < n; i++){
		double w = kTwoPi * modes_[i].freq_hz;
		Psi.block<2, 2>(2 * i, 2 * i) = oscillatorTransition(w, cfg_.nominal_dt);
	}
	Psi(sz - 1, sz - 1) = 1.0;

	Eigen::MatrixXd C = Eigen::MatrixXd::Zero(1, sz);
	for(int i = 0; i < n; i++){ C(0, 2 * i) = 1.0; }
	C(0, sz - 1) = 1.0;

	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(sz, sz) * cfg_.process_noise_osc;
	Q(sz - 1, sz - 1) = cfg_.process_noise_offset;
	double R = cfg_.measurement_noise;

	// Fixed-point iteration for the steady-state a-priori error covariance P:
	//   S = C P C^T + R
	//   K = P C^T / S            (filter-form gain, an intermediate)
	//   P_next = Psi (P - K C P) Psi^T + Q
	// then the PREDICTOR-form gain is L = Psi P C^T / S (the extra Psi
	// reflects eq. 6 producing x_{k+1} directly from x_k, not x_{k|k}).
	Eigen::MatrixXd P = Q;
	for(int iter = 0; iter < cfg_.riccati_max_iter; iter++){
		double S = (C * P * C.transpose())(0, 0) + R;
		Eigen::MatrixXd K = (P * C.transpose()) / S;
		Eigen::MatrixXd Pnext = Psi * (P - K * C * P) * Psi.transpose() + Q;
		Pnext = 0.5 * (Pnext + Pnext.transpose()); // numerical symmetry hygiene
		double delta = (Pnext - P).norm();
		P = Pnext;
		if(delta < cfg_.riccati_tol){
			break;
		}
	}
	double S = (C * P * C.transpose())(0, 0) + R;
	gainL_ = (Psi * P * C.transpose()) / S;
}

// ---------------------------------------------------------------------------
// Prediction
// ---------------------------------------------------------------------------

double SpaPredictor::propagateAndSample(double horizon_s) const
{
	int n = static_cast<int>(modes_.size());
	int sz = state_.size();
	Eigen::VectorXd s(sz);
	for(int i = 0; i < n; i++){
		double w = kTwoPi * modes_[i].freq_hz;
		s.segment<2>(2 * i) = oscillatorTransition(w, horizon_s) * state_.segment<2>(2 * i);
	}
	s(sz - 1) = state_(sz - 1);
	return sampleState(s);
}

bool SpaPredictor::predict(double horizon_s, double* value_out) const
{
	if(!initialized_){
		return false;
	}
	*value_out = propagateAndSample(horizon_s);
	return true;
}

std::vector<double> SpaPredictor::predictAndAssess(const std::vector<double>& horizons_s)
{
	std::vector<double> out;
	out.reserve(horizons_s.size());
	if(!initialized_){
		return out;
	}
	for(double h : horizons_s){
		double v = propagateAndSample(h);
		out.push_back(v);
		pending_.push_back({lastT_, lastT_ + h, v});
	}
	// Bound the pending buffer so a prediction that never resolves (e.g.
	// measurements stop arriving) doesn't grow it unboundedly. Generous
	// margin over sigma_max_horizon since a request can be for a horizon
	// longer than that.
	double maxAge = cfg_.sigma_max_horizon + cfg_.t_fft;
	while(!pending_.empty() && (lastT_ - pending_.front().madeAtT) > maxAge){
		pending_.pop_front();
	}
	return out;
}

int SpaPredictor::horizonBin(double horizon_s) const
{
	double clamped = std::clamp(horizon_s, 0.0, cfg_.sigma_max_horizon);
	int bin = static_cast<int>(std::round(clamped / cfg_.sigma_bin_s));
	return std::clamp(bin, 0, static_cast<int>(sigmaBinsMse_.size()) - 1);
}

void SpaPredictor::resolvePending(double t, double value)
{
	// A pending prediction "resolves" once a real measurement arrives at or
	// after its target time. Tolerance is half the nominal sample period --
	// close enough that comparing against `value` is a fair test of that
	// prediction, without waiting for an exact timestamp match that
	// irregular sampling will rarely produce.
	double tol = 0.5 * cfg_.nominal_dt;
	while(!pending_.empty() && pending_.front().targetT <= t + tol){
		const PendingPrediction& p = pending_.front();
		if(p.targetT >= t - cfg_.t_fft){ // ignore stale entries far in the past
			double horizon = p.targetT - p.madeAtT;
			double err = value - p.predictedValue;
			int bin = horizonBin(horizon);
			if(sigmaBinsCount_[bin] == 0){
				sigmaBinsMse_[bin] = err * err;
			}
			else{
				double a = cfg_.sigma_ewma_alpha;
				sigmaBinsMse_[bin] = (1.0 - a) * sigmaBinsMse_[bin] + a * (err * err);
			}
			sigmaBinsCount_[bin] += 1;
		}
		pending_.pop_front();
	}
}

double SpaPredictor::sigmaAtHorizon(double horizon_s) const
{
	int bin = horizonBin(horizon_s);
	if(sigmaBinsCount_[bin] == 0){
		return std::numeric_limits<double>::quiet_NaN();
	}
	return std::sqrt(sigmaBinsMse_[bin]);
}
