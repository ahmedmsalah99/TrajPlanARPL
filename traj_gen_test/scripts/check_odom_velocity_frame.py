#!/usr/bin/env python3
"""Check whether the logged odometry velocity is in the frame the planner assumes.

traj_manager.cpp's vehicleOdometryToRosOdometry copies px4_msgs/VehicleOdometry's
velocity[] straight into the ROS twist and treats it as NED. But VehicleOdometry
carries a velocity_frame field which may be NED, FRD, or BODY_FRD, and nothing
checks it. If PX4 is publishing body-frame velocity, then vx/vy are rotated by
the vehicle's yaw relative to the NED position -- while vz passes through nearly
unchanged, since body-down and NED-down coincide in level flight.

That matters well beyond the logs: this velocity is the initial condition every
initialPlan()/replan() solve anchors to.

The position in the same message is unambiguously NED, so differentiating it
gives a ground-truth NED velocity to compare against:

    NED hypothesis:   v_reported ~= d(pos)/dt
    body hypothesis:  v_reported ~= Rz(yaw)^T * d(pos)/dt

Whichever residual is smaller wins. Needs the roll/pitch/yaw columns.

Usage:
    python3 check_odom_velocity_frame.py [actual_trajectory.csv]
"""
import argparse
import csv
import math


def load(path):
    cols = {}
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames:
            cols[name] = []
        for row in reader:
            for name in reader.fieldnames:
                cols[name].append(float(row[name]))
    return cols


def central_diff(t, v):
    """d(v)/dt by central difference; endpoints one-sided. None where dt == 0."""
    n = len(v)
    out = [None] * n
    for i in range(n):
        lo = max(0, i - 1)
        hi = min(n - 1, i + 1)
        dt = t[hi] - t[lo]
        if dt > 1e-9:
            out[i] = (v[hi] - v[lo]) / dt
    return out


def rms(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return float('nan')
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('actual_csv', nargs='?', default='/tmp/actual_trajectory.csv')
    parser.add_argument('--min-speed', type=float, default=0.05,
                        help='ignore samples slower than this (m/s) -- yaw rotation is '
                             'unobservable when the vehicle is barely moving')
    args = parser.parse_args()

    c = load(args.actual_csv)
    if 'yaw' not in c:
        print('No yaw column in %s -- this log predates roll/pitch/yaw logging.'
              % args.actual_csv)
        return

    t = c['t_rel']
    dx, dy, dz = (central_diff(t, c[k]) for k in ('x', 'y', 'z'))

    res_ned_x, res_ned_y, res_body_x, res_body_y, res_z = [], [], [], [], []
    for i in range(len(t)):
        if dx[i] is None:
            continue
        speed = math.hypot(dx[i], dy[i])
        res_z.append(c['vz'][i] - dz[i])
        if speed < args.min_speed:
            continue
        # NED: reported velocity should equal the differentiated NED position.
        res_ned_x.append(c['vx'][i] - dx[i])
        res_ned_y.append(c['vy'][i] - dy[i])
        # Body: reported velocity should equal that same NED velocity rotated
        # into the body frame by -yaw.
        cy, sy = math.cos(c['yaw'][i]), math.sin(c['yaw'][i])
        res_body_x.append(c['vx'][i] - ( cy * dx[i] + sy * dy[i]))
        res_body_y.append(c['vy'][i] - (-sy * dx[i] + cy * dy[i]))

    if not res_ned_x:
        print('No samples above --min-speed %.3f m/s; nothing to compare.' % args.min_speed)
        return

    ned = rms(res_ned_x + res_ned_y)
    body = rms(res_body_x + res_body_y)
    yaws = [abs(y) for y in c['yaw']]

    print('samples compared : %d of %d' % (len(res_ned_x), len(t)))
    print('|yaw|  max/mean  : %.3f / %.3f rad' % (max(yaws), sum(yaws) / len(yaws)))
    print()
    print('horizontal residual RMS (m/s), lower is the frame actually in use:')
    print('  as NED   : %.4f' % ned)
    print('  as body  : %.4f' % body)
    print()
    print('vz residual RMS  : %.4f m/s' % rms(res_z))
    print('   (sanity check -- small under either hypothesis, since body-down and')
    print('    NED-down coincide in level flight. That is exactly why z tracks')
    print('    cleanly while x/y do not when the frame is wrong.)')
    print()

    if max(yaws) < 0.05:
        print('VERDICT: |yaw| never exceeds 0.05 rad, so the two hypotheses are')
        print('         indistinguishable in this log. Fly with some yaw and re-run.')
    elif ned < body * 0.5:
        print('VERDICT: velocity is NED, as the code assumes. The horizontal')
        print('         mismatch is something else.')
    elif body < ned * 0.5:
        print('VERDICT: velocity is BODY-frame, but traj_manager.cpp treats it as')
        print('         NED. vx/vy are rotated by yaw -- in the logs, and in the')
        print('         initial condition every solve is anchored to.')
    else:
        print('VERDICT: inconclusive -- neither hypothesis clearly wins. Check')
        print('         VehicleOdometry.velocity_frame directly:')
        print('         ros2 topic echo /fmu/out/vehicle_odometry --field velocity_frame')


if __name__ == '__main__':
    main()
