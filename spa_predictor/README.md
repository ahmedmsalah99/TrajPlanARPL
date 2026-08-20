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

`addMeasurement()` also optionally accepts an ACCELERATION reading
alongside the position/value one, for a genuinely better ESTIMATE, not
(currently) a predicted output: a harmonic oscillator's acceleration is
already an exact linear function of its own position state
(`x1'' = -w^2 x1`), so no new state was needed -- just a second row in the
correction step's output/measurement matrix, and a second steady-state
Kalman gain (`gainL_`, used when an acceleration reading is available that
sample; `gainLPosOnly_`, the original single-channel gain, otherwise) --
see `rebuildObserver()`/`stepEstimator()`. Pass `NaN` (the default) for
`accel` on samples where no acceleration reading is available yet (e.g. a
finite-difference chain still bootstrapping) -- falls back to position-only
correction transparently for that one sample.

`spa_axis_node` wraps it around one axis of a `px4_msgs/VehicleOdometry`
position source (selected via the `axis` parameter: 0/1/2 -> NED
North/East/Down, i.e. x/y/heave) and publishes `spa_predictor/SpaPrediction`
(predictions + self-assessment). Run one instance per axis you want
predicted -- e.g. three instances (axis 0, 1, 2) for a pad that translates
in x/y as well as heaving, each with its own independent mode detection
(no assumption that different axes share frequencies). Also feeds the
estimator's acceleration channel: one finite difference of the
already-measured `VehicleOdometry.velocity[axis]` (`include/spa_predictor/
finite_difference.h`'s `FiniteDifference`, shared with `spa_angle_node`).

`spa_angle_node` is the sibling for tilt: wraps `SpaPredictor` around roll
or pitch (selected via the `angle` parameter: 0/1 -> roll/pitch), extracted
from `VehicleOdometry.q` (aerospace ZYX/NED-FRD convention -- see
`spa_angle_node.cpp`'s `quatToRollPitch()`). `predicted_velocity` here is
the predicted angular RATE at each horizon, from the same
free-with-`predictWithVelocity()` mechanism as linear velocity. Predicts
Euler roll/pitch, not the surface-normal components (`s3x`/`s3y`) that
`TrajBase::calcPerchCond()` actually consumes -- the two agree to within
~1% below this system's perch tilt ceiling (~10-25 deg), and Euler angles
are simpler to interpret/verify for this validation step; reconstructing
`s3` from predicted roll/pitch is a well-defined follow-up when wiring this
into `calcPerchCond()`, not implemented here. Also feeds the estimator's
acceleration channel, but via TWO chained `FiniteDifference` instances
(angle -> rate -> accel), since -- unlike `spa_axis_node`'s linear axes --
there's no directly-measured Euler angle rate to difference just once
(`VehicleOdometry.angular_velocity` is a BODY-frame rate, not the Euler
rate, outside the small-angle/single-axis case).

Deliberately standalone -- not linked into `ros_traj_gen_utils`' `traj_exe`
yet; this is Phase 2 of the SPA rollout (validate against a known/simulated
signal before anything consumes its output for real).

`spa_landing_gate_node` is a Go/NoGo gate built on top of the five
predictors' own published outputs (it does not touch `SpaPredictor`
directly). At `publish_rate_hz` it looks up each of x/y/heave/roll/pitch's
`SpaPrediction` at a fixed `horizon_s` (default 0.5s) and publishes
`spa_predictor/LandingGate` with `go = true` only when ALL of:

- **velocity**: predicted `|vx|`, `|vy|`, `|vz|` (from `predicted_velocity`)
  are all `<= velocity_threshold`.
- **inclination**: the pad's predicted surface normal `s3`, reconstructed
  from PREDICTED roll/pitch and the pad's current MEASURED yaw (yaw isn't
  predicted -- out of SPA's scope; assumed to change negligibly over a
  ~0.5s horizon), isn't tilted more than a configured max angle off dead
  level: `s3 . world_up >= min_inclination_cos`. Reuses
  `TrajBase::calcPerchCond()`'s own `s3`/`world_up = (0,0,-1)` (NED)
  convention exactly, rather than re-deriving a new sign convention that
  could silently disagree with it.
- **direction**: the pad's HEADING (yaw only -- not the `s3` tilt above;
  roll/pitch are already covered by the inclination condition) puts the
  drone somewhere in its rear half relative to that heading -- astern, or
  behind-left/behind-right out to (and including) abeam, not only dead
  astern. Trivially satisfied (not a failure) when either the pad has no
  meaningful net horizontal motion (below `static_speed_threshold_mps` --
  an anchored/hovering pad's yaw has no coherent "forward" to measure a
  stern arc from) or the drone is too close horizontally to the pad to
  define a meaningful bearing (`direction_epsilon_m`).

`go` is forced `false` (and every other field left stale/meaningless) via
`inputs_ready` whenever any of the five `SpaPrediction` topics hasn't
published yet, doesn't have an entry at `horizon_s` (within
`horizon_tol_s`), or the drone/pad `VehicleOdometry` sources haven't been
heard from. This is a coarse, forward-looking sanity filter using SPA's
predictions -- it does NOT replace `calcPerchCond()`'s own precise,
azimuth-dependent terminal-condition rejection test at touchdown itself.

## Build & run

```bash
colcon build --packages-select spa_predictor
source install/setup.bash

# All five signals (x, y, heave, roll, pitch) + the CSV logger + the Go/NoGo
# landing gate, recommended:
ros2 launch spa_predictor spa_axes.launch.py
# Override the shared input source (default: pad_motion_gazebo's topic):
ros2 launch spa_predictor spa_axes.launch.py input_topic:=/some/other/vehicle_odometry
# Override the drone's own odometry source the landing gate needs (default:
# matches ros_traj_gen_utils/traj_manager.cpp's own convention):
ros2 launch spa_predictor spa_axes.launch.py drone_odom_topic:=/some/other/vehicle_odometry
# Tune mode-detection GROUPED as horizontal (x+y) / heave / angular
# (roll+pitch), rather than per-signal -- see the launch file's own header
# comment for why. Any TUNABLE_PARAMS entry left unset keeps that node's
# own Config default (see spa_predictor.h).
ros2 launch spa_predictor spa_axes.launch.py horizontal_peak_sensitivity:=0.08 heave_t_fft_s:=15.0 angular_max_modes:=6

# Or run a single node by hand:
ros2 run spa_predictor spa_axis_node                                    # heave (axis=2, default)
ros2 run spa_predictor spa_axis_node --ros-args -p axis:=0              # x/North
ros2 run spa_predictor spa_axis_node --ros-args -p axis:=1              # y/East
ros2 run spa_predictor spa_angle_node                                   # roll (angle=0, default)
ros2 run spa_predictor spa_angle_node --ros-args -p angle:=1            # pitch
```

Feed it a position/attitude source -- see the `pad_motion_gazebo` package
for a simulated one (ground-truth pad pose via a gz-sim plugin, optionally
with synthetic sensor noise -- see its `PadMotionPlugin.cc`'s class
comment for the `position_noise_std`/`velocity_noise_std`/
`attitude_noise_std`/`angular_velocity_noise_std`/`noise_seed` SDF
parameters), or point `input_topic` at a real vision-derived source once
available. All five instances above read the SAME `input_topic` by
default, and each defaults its `output_topic` from its own axis/angle
selector (`/pad/spa/x_prediction`, `.../y_prediction`,
`.../heave_prediction`, `.../roll_prediction`, `.../pitch_prediction`) so
running multiple instances doesn't collide without extra flags.

## Parameters (spa_axis_node / spa_angle_node)

Both nodes share the same `SpaPredictor::Config`-derived parameters below
(`publish_rate_hz` onward) -- they only differ in how the input signal is
selected and what it's called:

| Param | Default | Meaning |
|---|---|---|
| `axis` (spa_axis_node only) | `2` | which NED position component to predict: 0=x/North, 1=y/East, 2=heave/Down |
| `angle` (spa_angle_node only) | `0` | which attitude component to predict: 0=roll, 1=pitch |
| `input_topic` | `/pad/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` source; `position[axis]` (spa_axis_node) or `q` (roll/pitch extracted, spa_angle_node) is used |
| `output_topic` | `/pad/spa/<x\|y\|heave\|roll\|pitch>_prediction` | `spa_predictor/SpaPrediction`, defaulted from `axis`/`angle` |
| `publish_rate_hz` | `10.0` | how often predictions are computed/published |
| `horizons_s` | `[0, 0.5, 1, 1.5, 2, 3, 4, 5]` | horizons requested each cycle; also what `sigma_s` is assessed at |
| `t_fft_s` | `25.0` | mode-detection window; must span several periods of the slowest mode |
| `f_min_hz` / `f_max_hz` | `0.03` / `2.0` | mode search band -- `f_min_hz` excludes near-DC drift from being fit as a spurious mode |
| `peak_sensitivity` | `0.10` | peak acceptance threshold, fraction of the largest in-band peak |
| `max_modes` | `4` | cap on simultaneously-tracked modes |
| `process_noise_osc` / `process_noise_offset` | `1e-4` / `5e-3` | Kalman process-noise tuning -- see `spa_predictor.h`'s `Config` comments |
| `measurement_noise_pos` / `measurement_noise_accel` | `0.01` / `1.0` | Kalman measurement noise for the position and acceleration correction channels respectively -- the latter's default is NOT calibrated to any signal (finite-differencing sharply amplifies noise; roll/pitch need two chained differences vs. one for x/y/heave), expect to tune per signal group |
| `nominal_dt_s` | `1/30` | sample period the steady-state gain is solved for (propagation always uses the true per-sample dt regardless) |
| `sigma_max_horizon_s` / `sigma_bin_s` | `8.0` / `0.5` | self-assessment binning range/resolution |

See `spa_predictor.h`'s `Config` struct for the authoritative documentation
of every tunable -- the table above is a quick reference, not a substitute.

`SpaPrediction.msg`'s `predicted_velocity` is a parallel array to
`predicted_value` (same `horizon_s` indexing) -- the signal's own
time-derivative at each horizon. It has no `sigma_s`-style self-assessment:
only position/value is ever measured (`addMeasurement()`), so there's no
independent velocity measurement to check a velocity prediction against.

Don't confuse this with the acceleration correction channel above --
they're unrelated. `predicted_velocity` is a model OUTPUT (the estimator's
own velocity state, sampled at a horizon); `derived_accel` (published for
visibility -- see its `.msg` comment) is a model INPUT, the value used to
correct the SAME position-only state the model already had, NOT a
prediction. Named "derived", not "measured" -- there is no accelerometer
anywhere in this pipeline; it's computed by finite-differencing
already-published data entirely outside `SpaPredictor` (one differentiation
of the already-published velocity for x/y/heave, two chained
differentiations, angle -> rate -> accel, for roll/pitch), so treat it as
considerably noisier than a real measurement would be. Neither node
exposes a `predicted_acceleration` OUTPUT -- deliberately out of scope for
now, see `spa_predictor.h`'s class comment. `derived_accel` is logged as
its own column by `spa_eval_logger.py`.

## Parameters (spa_landing_gate_node)

| Param | Default | Meaning |
|---|---|---|
| `x_topic` / `y_topic` / `heave_topic` / `roll_topic` / `pitch_topic` | `/pad/spa/<x\|y\|heave\|roll\|pitch>_prediction` | the five `SpaPrediction` sources -- matches the axis/angle nodes' own default `output_topic`s |
| `pad_odom_topic` | `/pad/fmu/out/vehicle_odometry` | pad's own odometry, only used for its MEASURED yaw (see above) |
| `drone_odom_topic` | `/fmu/out/vehicle_odometry` | the drone's own odometry, only used for its current x/y position |
| `output_topic` | `/pad/landing_gate` | `spa_predictor/LandingGate` |
| `publish_rate_hz` | `10.0` | how often the gate is evaluated/published |
| `horizon_s` | `0.5` | fixed horizon every predicted quantity is evaluated at -- must match (within `horizon_tol_s`) an entry already in each `SpaPrediction`'s own `horizon_s[]` |
| `horizon_tol_s` | `0.01` | matching tolerance for the lookup above |
| `velocity_threshold` | `0.3` | m/s, see velocity condition above -- not calibrated to any particular vehicle/pad, tune to what your controller can track through touchdown |
| `min_inclination_cos` | `cos(30 deg) ~= 0.866` | see inclination condition above -- deliberately looser than `calcPerchCond()`'s own precise tilt ceiling |
| `max_direction_cos` | `0.0` | see direction condition above -- `0.0` admits the pad's entire rear half (astern through abeam); more negative narrows toward dead-astern-only, more positive admits part of the forward half too |
| `static_speed_threshold_mps` | `0.05` | below this pad horizontal speed (m/s, predicted at `horizon_s`), the pad is considered STATIC and the direction condition is trivially satisfied -- see above |
| `direction_epsilon_m` | `0.05` | below this horizontal magnitude (m), the direction condition is trivially satisfied instead of computed -- see above |

## Offline accuracy evaluation

Two diagnostic-only tools (`scripts/`, not part of the C++ library/node
above), for measuring prediction accuracy vs. horizon precisely rather than
by eye off a live plot:

- `spa_eval_logger.py`: a small rclpy node that records ground-truth pad
  odometry and every `SpaPrediction` message to two CSVs (`pad_truth.csv`,
  `spa_predictions.csv`) under `/tmp/spa_eval` by default (each run
  overwrites the last -- by design, not a bug: this is diagnostic data for
  the current run, not a history). `spa_predictions.csv` already carries
  `t_target = made_at_t + horizon_s` per horizon -- the "shift" that lines
  a prediction up in time with the ground-truth sample it was actually
  about. **Launched automatically by `spa_axes.launch.py`** -- no separate
  step needed.
- `spa_eval_analyze.py`: a pure-Python (no ROS dependency), NOT ROS-installed
  script -- run it straight from the source tree, not via `ros2 run`. It
  loads the two CSVs, interpolates ground truth at each `t_target`, and
  reports mean/median/RMSE/max error per horizon, plus three plots per
  axis: shifted predictions overlaid on ground truth, error over time, and
  error-vs-horizon (the direct, precise answer to "does accuracy degrade
  with horizon, and by how much").

```bash
# Brings up the predictor(s) AND the CSV logger together:
ros2 launch spa_predictor spa_axes.launch.py
# ... Ctrl-C once you have enough data ...

# Analyze (needs pandas/numpy/matplotlib -- pip install if missing). No
# args needed -- defaults to /tmp/spa_eval, matching the logger's default:
python3 spa_predictor/scripts/spa_eval_analyze.py
```

`spa_eval_analyze.py`'s `--axis` defaults to `all` (x, y, heave, roll,
pitch); pass `--show` to also open interactive plot windows instead of
only saving PNGs; `--truth`/`--predictions`/`--out-dir` override the
`/tmp/spa_eval` default paths. See each script's own docstring for the
full CSV schema and every CLI/parameter option.

## Known limitation

If one mode-detection pass finds no peaks (a genuinely calm moment, or just
one noisy/unlucky window), the current mode set is kept rather than reset to
zero -- avoids discarding a converged model on a single inconclusive pass,
but means there's no mechanism yet to actually detect and act on a real,
sustained transition to calm. See `maybeRunModeDetection()`'s comment.
