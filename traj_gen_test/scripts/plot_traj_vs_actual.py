#!/usr/bin/env python3
"""Plot the actual flown trajectory against a handful of planned-trajectory
snapshots, for the traj_manager.cpp [TRAJ_LOG] capture -- position and
velocity side by side.

traj_manager.cpp writes two CSVs while running (see planned_traj_log_path/
actual_traj_log_path params). Both start recording, and both time axes are
zeroed, the moment offboard is enabled (start_replan is called) -- t=0 is
that instant, not node startup.

  planned_trajectories.csv: traj_id,gen_time_rel,t_local,x,y,z,vx,vy,vz
    One block of rows per solve that became the active trajectory (throttled
    by traj_save_period_s, so a fast replan cadence doesn't flood the file
    with near-identical snapshots), sampled across that trajectory's own
    duration (t_local, seconds since that solve's own t=0). gen_time_rel is
    seconds since offboard was enabled.

  actual_trajectory.csv: t_rel,x,y,z,vx,vy,vz
    Continuous actual vehicle state from /fmu/out/vehicle_odometry, t_rel in
    seconds since offboard was enabled.

Since both are already relative to the same t=0, a planned sample's time
since offboard-enable is just gen_time_rel + t_local -- no alignment step
needed.

Usage:
    python3 plot_traj_vs_actual.py [planned.csv] [actual.csv] [--num-planned N]
"""
import argparse
import csv
from collections import defaultdict

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d projection)


def load_actual(path):
    t, x, y, z, vx, vy, vz = [], [], [], [], [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            t.append(float(row['t_rel']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            z.append(float(row['z']))
            vx.append(float(row['vx']))
            vy.append(float(row['vy']))
            vz.append(float(row['vz']))
    return {'t': t, 'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}


def load_planned(path):
    """Returns {traj_id: {'gen_time_rel': float, 't': [...], 'x': [...], ..., 'vz': [...]}}."""
    empty = lambda: {'gen_time_rel': None, 't': [], 'x': [], 'y': [], 'z': [],
                      'vx': [], 'vy': [], 'vz': []}
    trajs = defaultdict(empty)
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            traj_id = int(row['traj_id'])
            entry = trajs[traj_id]
            if entry['gen_time_rel'] is None:
                entry['gen_time_rel'] = float(row['gen_time_rel'])
            entry['t'].append(float(row['t_local']))
            for key in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
                entry[key].append(float(row[key]))
    return trajs


def pick_equally_spaced(sorted_ids, n):
    """n indices spread evenly across sorted_ids (endpoints included)."""
    count = len(sorted_ids)
    if count <= n:
        return sorted_ids
    if n <= 1:
        return [sorted_ids[0]]
    step = (count - 1) / (n - 1)
    return [sorted_ids[round(i * step)] for i in range(n)]


def plot_3d(ax, actual, planned, chosen_ids, keys, labels, title):
    kx, ky, kz = keys
    ax.plot(actual[kx], actual[ky], actual[kz], '-', color='tab:orange',
             linewidth=2, label='actual')

    colors = plt.cm.viridis([i / max(1, len(chosen_ids) - 1) for i in range(len(chosen_ids))])
    for color, traj_id in zip(colors, chosen_ids):
        entry = planned[traj_id]
        ax.plot(entry[kx], entry[ky], entry[kz], '--', color=color,
                 label='planned @ t=%.1fs' % entry['gen_time_rel'])

    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_zlabel(labels[2])
    ax.set_title(title)
    ax.legend(fontsize='small')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('planned_csv', nargs='?', default='/tmp/planned_trajectories.csv')
    parser.add_argument('actual_csv', nargs='?', default='/tmp/actual_trajectory.csv')
    parser.add_argument('--num-planned', type=int, default=10,
                         help='how many equally-spaced planned snapshots to overlay')
    args = parser.parse_args()

    actual = load_actual(args.actual_csv)
    planned = load_planned(args.planned_csv)

    sorted_ids = sorted(planned.keys())
    if not sorted_ids:
        print('No planned trajectories found in %s' % args.planned_csv)
        return
    chosen_ids = pick_equally_spaced(sorted_ids, args.num_planned)

    fig = plt.figure(figsize=(16, 8))
    ax_pos = fig.add_subplot(121, projection='3d')
    ax_vel = fig.add_subplot(122, projection='3d')

    plot_3d(ax_pos, actual, planned, chosen_ids, ('x', 'y', 'z'),
            ('x (N)', 'y (E)', 'z (D)'),
            'Position: actual vs %d planned snapshots' % len(chosen_ids))
    plot_3d(ax_vel, actual, planned, chosen_ids, ('vx', 'vy', 'vz'),
            ('vx (N/s)', 'vy (E/s)', 'vz (D/s)'),
            'Velocity: actual vs %d planned snapshots' % len(chosen_ids))

    fig.suptitle('t=0 at offboard enable (NED)')
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
