# spa_predictor

Signal Prediction Algorithm (SPA) -- a scalar-signal predictor, per Abujoub,
McPhee & Irani, "Methodologies for Landing Autonomous Aerial Vehicles on
Maritime Vessels" (2020), Sec. 3.1: DFT-based mode detection, a steady-state
Kalman predictor over a bank of harmonic-oscillator states, and empirical
self-assessment (RMS error vs. prediction horizon).

`SpaPredictor` (`include/spa_predictor/spa_predictor.h`) is pure Eigen, no
ROS dependency -- one instance per scalar signal (the paper runs one SPA per
degree of freedom). Besides `predict()` (value at a horizon), it also
exposes `predictWithVelocity()` -- the signal's time-derivative at that
same horizon, from the same propagated state, at no extra cost -- needed
for AHC-style relative impact velocity, not just position matching.

`spa_axis_node` wraps it around one axis of a `px4_msgs/VehicleOdometry`
position source (selected via the `axis` parameter: 0/1/2 -> NED
North/East/Down, i.e. x/y/heave) and publishes `spa_predictor/SpaPrediction`
(predictions + self-assessment). Run one instance per axis you want
predicted -- e.g. three instances (axis 0, 1, 2) for a pad that translates
in x/y as well as heaving, each with its own independent mode detection
(no assumption that different axes share frequencies). Roll/pitch
predictors (separate SpaPredictor instances over angle, not position) are
a planned follow-up once this is validated.

Deliberately standalone -- not linked into `ros_traj_gen_utils`' `traj_exe`
yet; this is Phase 2 of the SPA rollout (validate against a known/simulated
signal before anything consumes its output for real).

## Build & run

```bash
colcon build --packages-select spa_predictor
source install/setup.bash
ros2 run spa_predictor spa_axis_node                                    # heave (axis=2, default)
ros2 run spa_predictor spa_axis_node --ros-args -p axis:=0              # x/North
ros2 run spa_predictor spa_axis_node --ros-args -p axis:=1              # y/East
```

Feed it a position source -- see the `pad_motion_gazebo` package for a
simulated one (ground-truth pad pose via a gz-sim plugin), or point
`input_topic` at a real vision-derived source once available. All three
instances above read the SAME `input_topic` by default (each just looks at
a different `position[]` index), and each defaults its `output_topic` from
`axis` (`/pad/spa/x_prediction`, `/pad/spa/y_prediction`,
`/pad/spa/heave_prediction`) so running multiple instances doesn't collide
without extra flags.

## Parameters (spa_axis_node)

| Param | Default | Meaning |
|---|---|---|
| `axis` | `2` | which NED position component to predict: 0=x/North, 1=y/East, 2=heave/Down |
| `input_topic` | `/pad/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` source; `position[axis]` is used |
| `output_topic` | `/pad/spa/<x\|y\|heave>_prediction` | `spa_predictor/SpaPrediction`, defaulted from `axis` |
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

`SpaPrediction.msg`'s `predicted_velocity` is a parallel array to
`predicted_value` (same `horizon_s` indexing) -- the signal's own
time-derivative at each horizon. It has no `sigma_s`-style self-assessment:
only position/value is ever measured (`addMeasurement()`), so there's no
independent velocity measurement to check a velocity prediction against.

## Known limitation

If one mode-detection pass finds no peaks (a genuinely calm moment, or just
one noisy/unlucky window), the current mode set is kept rather than reset to
zero -- avoids discarding a converged model on a single inconclusive pass,
but means there's no mechanism yet to actually detect and act on a real,
sustained transition to calm. See `maybeRunModeDetection()`'s comment.
