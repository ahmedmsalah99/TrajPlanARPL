# pad_motion_gazebo

A gz-sim (Harmonic/Fortress) `Model` plugin, `PadMotionPlugin`, that
kinematically animates a perch pad's heave motion and publishes its
GROUND-TRUTH pose/velocity as `px4_msgs/VehicleOdometry` -- the same message
type the drone's own odometry already uses elsewhere in this workspace, so
consumers (starting with the `spa_predictor` package's `spa_heave_node`)
need no new parsing code.

Scope: **heave only**. The pad's attitude never changes; roll/pitch motion
is a planned follow-up (see `PadMotionPlugin.cc`'s class comment for exactly
what that needs).

## Why ground truth, not simulated vision

This validates the SPA predictor's math (mode detection, the oscillator/
Kalman estimator, accuracy-vs-horizon) against a known signal, with zero
detector noise in the loop -- before pointing it at a real, noisier,
vision-derived heave estimate.

## Build & run

```bash
colcon build --packages-select pad_motion_gazebo spa_predictor
source install/setup.bash

# terminal 1: gz-sim with the heaving pad
ros2 launch pad_motion_gazebo pad_motion_sim.launch.py

# terminal 2: the SPA heave predictor
ros2 run spa_predictor spa_heave_node

# terminal 3: watch it converge
ros2 topic echo /pad/spa/heave_prediction
```

`sigma_s` (per `horizon_s`) starts as NaN (no self-assessment data yet) and
should start reporting real numbers once predictions made `t_fft` seconds
ago start resolving against fresh measurements -- give it at least one full
`t_fft` window (default 25s) before judging accuracy.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/pad/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | published by `PadMotionPlugin` | GROUND TRUTH, not from a real vehicle; NED, position\[2\] carries heave, identity attitude |
| `/pad/spa/heave_prediction` | `spa_predictor/SpaPrediction` | published by `spa_heave_node` | predictions + self-assessment |

## Standalone world vs. drop-in model

- `worlds/pad_motion_test.sdf`: self-contained world with just the pad (no
  drone) -- what `pad_motion_sim.launch.py` runs.
- `models/wave_pad/`: the same pad as an includable model
  (`<include><uri>model://wave_pad</uri></include>`) for dropping into an
  existing world (e.g. a real PX4 SITL world), once that world's
  `GZ_SIM_RESOURCE_PATH` includes this package's `models/` directory.

## Configuring the motion

Edit the `<heave_component>` elements in the SDF (world or model, they're
independent copies) -- each is `amplitude` (m) / `period_s` (s) /
`phase_rad`, and any number can be given for a multi-tone signal. Defaults
to a two-tone, deliberately non-harmonic 4.0s/1.7s signal so mode detection
has more than a single trivial tone to find.

## Assumptions worth checking before trusting the numbers

- **ENU world frame** (X-East, Y-North, Z-Up) -- matches PX4's own
  `gz_bridge` and standard ROS practice, but is a convention this package
  assumes, not something Gazebo enforces. Confirm before dropping
  `wave_pad` into a world you didn't author yourself.
- **`<static>true</static>` is required** on the pad model -- the plugin
  drives its pose directly every `PreUpdate`, which conflicts with letting
  the physics engine integrate it.
- Position/velocity in the published odometry are analytically exact (the
  plugin computes them from the same closed-form sinusoid it uses to move
  the model, not by reading Gazebo's own integrated state back) -- so this
  really is ground truth, not a numerically-differentiated approximation.
