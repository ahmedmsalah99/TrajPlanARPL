std_msgs/Header header

# The Go/NoGo decision -- true only when EVERY sub-condition below holds
# (see velocity_ok, inclination_ok, direction_ok) AND inputs_ready is true.
# Matches the paper's (Abujoub, McPhee & Irani, Sec. 3.1.1) Go-NoGo
# terminology -- this is the perch analogue of it, built from the SPA
# predictors' own predicted state rather than a raw instantaneous reading.
bool go

# False whenever any of the five upstream SpaPrediction topics (x, y,
# heave, roll, pitch) hasn't published an initialized message yet, or
# doesn't have an entry matching horizon_s (within tolerance) in its own
# horizon_s[] array, or the drone/pad VehicleOdometry sources haven't been
# heard from yet. go is forced false whenever this is false -- every field
# below is meaningless/stale in that case, not just go.
bool inputs_ready

# Which horizon (s) every predicted_* quantity below was evaluated at --
# fixed at node startup (see spa_landing_gate_node's horizon_s parameter),
# not per-message-configurable; carried here so a consumer/logger doesn't
# have to hardcode it separately.
float64 horizon_s

# -- Velocity condition: predicted x/y/heave velocity at horizon_s, from
# spa_axis_node's predicted_velocity (NED, m/s). --
float64 vx
float64 vy
float64 vz
float64 velocity_threshold  # as configured, for reference/logging
bool velocity_ok            # |vx|, |vy|, |vz| all <= velocity_threshold

# -- Inclination condition (tilt only -- see direction_cos below for the
# separate, yaw-only heading condition): the pad's predicted surface
# normal s3 at horizon_s, reconstructed from PREDICTED roll/pitch
# (spa_angle_node) and the pad's most recently MEASURED yaw (yaw is not
# predicted -- out of SPA's current scope; assumes yaw changes negligibly
# over horizon_s, which is reasonable for the ~0.5s horizons this gate is
# meant for, not necessarily for longer ones). Same s3 convention
# TrajBase::calcPerchCond() already uses (3rd column of the full rotation
# matrix; NED, world "up" is (0,0,-1)) -- this gate does NOT replace
# calcPerchCond()'s own precise terminal-condition rejection test, it's a
# coarser, earlier sanity filter using SPA's forward-looking predictions
# instead of the current instant.
float64 s3_x
float64 s3_y
float64 s3_z

# Cosine of the INCLINATION angle -- the angle between s3 and world_up
# (world_up = (0,0,-1) in NED), i.e. how far the pad's surface is tilted
# off dead-level. 1.0 = 0 deg (perfectly level, ideal), cos(30 deg) ~= 0.866
# = tilted 30 deg off level, 0.0 = 90 deg (a vertical wall), negative =
# tilted past vertical (overhanging/upside-down -- a quadrotor cannot
# thrust downward to hold against that).
float64 inclination_cos
bool inclination_ok  # inclination_cos >= min_inclination_cos (as configured) -- i.e. tilted no more than the configured max angle off level

# -- Direction condition: the pad's HEADING (yaw only, NOT the s3 tilt
# above -- roll/pitch are already covered by the inclination condition)
# vs. the drone's bearing FROM the pad. Cosine of the angle between the
# pad's bow direction and that bearing: -1.0 = drone dead astern
# (favorable), 0.0 = drone abeam (left or right, still favorable), +1.0 =
# drone dead ahead of the bow (unfavorable). Favors an approach from
# anywhere in the pad's rear half, not only dead astern. NaN whenever
# either the pad has no meaningful net horizontal motion (STATIC mode --
# see static_speed_threshold_mps, an anchored/hovering pad's yaw has no
# coherent "forward" to measure a stern arc from) or the drone is too
# close horizontally to the pad to define a meaningful bearing at all
# (direction_epsilon_m) -- direction_ok is trivially true in either case,
# see the node's comment.
float64 direction_cos

# Cosine of the angle between the pad's RIGHT/starboard direction (90 deg
# clockwise from the bow, viewed from above) and the drone's bearing FROM
# the pad: positive = drone is on the pad's right/starboard side,
# negative = left/port side, 0.0 = dead ahead or dead astern (on the
# boundary). Whether this side matches the single active side
# (allow_right) gates direction_ok outright this cycle -- see the node's
# comment. Same NaN cases as direction_cos above.
float64 side_cos

bool direction_ok  # (side_cos >= 0) == allow_right, AND direction_cos <= max_direction_cos (default 0.0 -- the whole rear half, astern through abeam), or the NaN special cases above
