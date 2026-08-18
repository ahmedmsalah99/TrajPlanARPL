std_msgs/Header header

# The Go/NoGo decision -- true only when EVERY sub-condition below holds
# (see velocity_ok, upward_ok, direction_ok) AND inputs_ready is true.
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

# -- Attitude condition: the pad's predicted surface normal s3 at
# horizon_s, reconstructed from PREDICTED roll/pitch (spa_angle_node) and
# the pad's most recently MEASURED yaw (yaw is not predicted -- out of
# SPA's current scope; assumes yaw changes negligibly over horizon_s,
# which is reasonable for the ~0.5s horizons this gate is meant for, not
# necessarily for longer ones). Same s3 convention TrajBase::calcPerchCond()
# already uses (3rd column of the full rotation matrix; NED, world "up" is
# (0,0,-1)) -- this gate does NOT replace calcPerchCond()'s own precise
# terminal-condition rejection test, it's a coarser, earlier sanity filter
# using SPA's forward-looking predictions instead of the current instant.
float64 s3_x
float64 s3_y
float64 s3_z

# s3 . world_up (world_up = (0,0,-1) in NED) -- 1.0 = perfectly upward,
# 0.0 = perfectly horizontal (a wall), negative = overhanging/downward.
float64 upward_cos
bool upward_ok  # upward_cos >= min_upward_cos (as configured)

# Cosine between s3's HORIZONTAL direction and the pad's horizontal offset
# from the drone (pad_xy - drone_xy): -1.0 = s3 points exactly opposite
# that offset (the surface leans back toward the drone's approach --
# favorable), +1.0 = s3 points exactly along it (the surface leans away,
# unfavorable). NaN when either vector's horizontal magnitude is too small
# to define a meaningful direction (pad nearly flat, or drone nearly
# directly overhead) -- direction_ok is trivially true in that case, see
# the node's comment.
float64 direction_cos
bool direction_ok  # direction_cos <= max_direction_cos (as configured), or the NaN special case above
