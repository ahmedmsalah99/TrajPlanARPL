# spa_predictor

Signal Prediction Algorithm (SPA) -- a scalar-signal predictor, per Abujoub,
McPhee & Irani, "Methodologies for Landing Autonomous Aerial Vehicles on
Maritime Vessels" (2020), Sec. 3.1: DFT-based mode detection, a steady-state
Kalman predictor over a bank of harmonic-oscillator states, and empirical
self-assessment (RMS error vs. prediction horizon).

`SpaPredictor` (`include/spa_predictor/spa_predictor.h`) is pure Eigen, no
ROS dependency -- one instance per scalar signal (the paper runs one SPA per
degree of freedom; this repo currently instantiates one for pad heave, with
roll/pitch predictors planned as a follow-up once this is validated).

`spa_heave_node` wraps it around a `px4_msgs/VehicleOdometry` heave source
and publishes `spa_predictor/SpaPrediction` (predictions + self-assessment).
Deliberately standalone -- not linked into `ros_traj_gen_utils`' `traj_exe`
yet; this is Phase 2 of the SPA rollout (validate against a known/simulated
signal before anything consumes its output for real).

## Build & run

```bash
colcon build --packages-select spa_predictor
source install/setup.bash
ros2 run spa_predictor spa_heave_node
```

Feed it a heave source -- see the `pad_motion_gazebo` package for a
simulated one (ground-truth pad heave via a gz-sim plugin), or point
`input_topic` at a real vision-derived source once available.

## Parameters (spa_heave_node)

| Param | Default | Meaning |
|---|---|---|
| `input_topic` | `/pad/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` source; `position[2]` (NED) is used |
| `output_topic` | `/pad/spa/heave_prediction` | `spa_predictor/SpaPrediction` |
| `publish_rate_hz` | `10.0` | how often predictions are computed/published |
| `horizons_s` | `[0, 0.5, 1, 1.5, 2, 3, 4, 5]` | horizons requested each cycle; also what `sigma_s` is assessed at |
| `t_fft_s` | `25.0` | mode-detection window; must span several periods of the slowest mode |
| `f_min_hz` / `f_max_hz` | `0.03` / `2.0` | mode search band -- `f_min_hz` excludes near-DC drift from being fit as a spurious mode |
| `peak_sensitivity` | `0.15` | peak acceptance threshold, fraction of the largest in-band peak |
| `max_modes` | `4` | cap on simultaneously-tracked modes |
| `process_noise_osc` / `process_noise_offset` / `measurement_noise` | `1e-4` / `5e-3` / `0.01` | Kalman tuning -- see `spa_predictor.h`'s `Config` comments |
| `nominal_dt_s` | `1/30` | sample period the steady-state gain is solved for (propagation always uses the true per-sample dt regardless) |
| `sigma_max_horizon_s` / `sigma_bin_s` | `8.0` / `0.5` | self-assessment binning range/resolution |

See `spa_predictor.h`'s `Config` struct for the authoritative documentation
of every tunable -- the table above is a quick reference, not a substitute.

## Known limitation

If one mode-detection pass finds no peaks (a genuinely calm moment, or just
one noisy/unlucky window), the current mode set is kept rather than reset to
zero -- avoids discarding a converged model on a single inconclusive pass,
but means there's no mechanism yet to actually detect and act on a real,
sustained transition to calm. See `maybeRunModeDetection()`'s comment.
