# traj_gen_test

Dummy data publisher to exercise the `ros_traj_gen_utils` `traj_exe` node without
the full flight stack. It publishes the inputs `traj_exe` consumes:

| Topic | Type | Notes |
|---|---|---|
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | best-effort; a hover by default (so the planner has a start state and the apriltag odom buffer fills) |
| `<device>/waypoints` | `nav_msgs/Path` | published once (or periodically) to trigger planning |
| `/tags_features_extractor/tag_pose` | `geometry_msgs/PoseStamped` | optional (`publish_tag:=true`) for the visual/FOV path |

## Build & run

```bash
colcon build --packages-select traj_gen_test
source install/setup.bash

# terminal 1: the planner
ros2 launch ros_traj_gen_utils traj_plan.launch.py

# terminal 2: the dummy inputs
ros2 launch traj_gen_test dummy_test.launch.py
# or directly:
ros2 run traj_gen_test dummy_publisher --ros-args -p publish_tag:=true
```

Then watch the outputs:
```bash
ros2 topic echo /quadrotor/position_cmd
ros2 topic echo /quadrotor/trackers_manager/qp_tracker/qp_trajectory_pos
```

## RViz / TF

The planner publishes its `Path`/`PositionCommand` in several frames (`odom`, `world`,
`simulator`) and the dummy's waypoints in `odom`. RViz needs a TF tree to place them,
so this node also broadcasts:

- static identity transforms `fixed_frame` → {`odom`/`frame_id`, `world`, `simulator`, `mocap`}
- dynamic `odom` → `base_link` (the drone pose; NED→ENU of `odom_ned`)
- static `base_link` → `camera` (for the tag pose)

In RViz set **Fixed Frame = `map`** (the default `fixed_frame`). Then add Path displays
for the waypoint/trajectory topics, a PoseStamped display for the tag, and a TF display
to see the drone/camera frames. Set `publish_tf:=false` if you provide TF elsewhere.

## Parameters

| Param | Default | Meaning |
|---|---|---|
| `device` | `/quadrotor` | vehicle namespace; waypoints go to `<device>/waypoints` |
| `frame_id` | `odom` | frame of the published waypoints |
| `odom_rate_hz` | `50.0` | odometry publish rate |
| `publish_tag` | `true` | also publish a dummy tag pose |
| `tag_rate_hz` | `20.0` | tag publish rate |
| `waypoint_period_s` | `0.1` | `0` = publish once after a 2 s delay; `>0` = repeat |
| `odom_ned` | `[0,0,0]` | hover position reported, in PX4 NED |
| `waypoints` | a 3-point path | flat `[x,y,z,yaw, ...]` in the world frame |
| `tag pose` | x,y,z,roll,pitch,yaw | in the camera frame |

Note: `traj_exe` also calls the `trackers_manager/transition` and `mav_services/hover`
services asynchronously; this package does not provide those servers, which is
harmless (the calls are fire-and-forget). The `device` must match the planner's
`device` config.
