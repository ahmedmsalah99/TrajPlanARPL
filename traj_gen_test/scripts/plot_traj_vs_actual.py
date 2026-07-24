#!/usr/bin/env python3
"""Plot the actual flown trajectory against a handful of planned-trajectory
snapshots, for the traj_manager.cpp [TRAJ_LOG] capture.

traj_manager.cpp writes two CSVs while running (see planned_traj_log_path/
actual_traj_log_path params):

  planned_trajectories.csv: traj_id,gen_time_abs,t_local,x,y,z,vx,vy,vz
    One block of rows per solve that became the active trajectory (throttled
    by traj_save_period_s, so a fast replan cadence doesn't flood the file
    with near-identical snapshots), sampled across that trajectory's own
    duration (t_local, seconds since that solve's own t=0).

  actual_trajectory.csv: t_abs,x,y,z,vx,vy,vz
    Continuous actual vehicle state from /fmu/out/vehicle_odometry.

Both files timestamp with the SAME absolute clock (node->now()), so a
planned sample's absolute time is just gen_time_abs + t_local -- no
alignment step needed.

Usage:
    python3 plot_traj_vs_actual.py [planned.csv] [actual.csv] [--num-planned N]
"""
import argparse
import csv
from collections import defaultdict

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d projection)


def load_actual(path):
    t, x, y, z = [], [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            t.append(float(row['t_abs']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            z.append(float(row['z']))
    return t, x, y, z


def load_planned(path):
    """Returns {traj_id: {'gen_time_abs': float, 't': [...], 'x': [...], 'y': [...], 'z': [...]}}."""
    trajs = defaultdict(lambda: {'gen_time_abs': None, 't': [], 'x': [], 'y': [], 'z': []})
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            traj_id = int(row['traj_id'])
            entry = trajs[traj_id]
            if entry['gen_time_abs'] is None:
                entry['gen_time_abs'] = float(row['gen_time_abs'])
            entry['t'].append(float(row['t_local']))
            entry['x'].append(float(row['x']))
            entry['y'].append(float(row['y']))
            entry['z'].append(float(row['z']))
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


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('planned_csv', nargs='?', default='/tmp/planned_trajectories.csv')
    parser.add_argument('actual_csv', nargs='?', default='/tmp/actual_trajectory.csv')
    parser.add_argument('--num-planned', type=int, default=5,
                         help='how many equally-spaced planned snapshots to overlay')
    args = parser.parse_args()

    actual_t, actual_x, actual_y, actual_z = load_actual(args.actual_csv)
    planned = load_planned(args.planned_csv)

    sorted_ids = sorted(planned.keys())
    if not sorted_ids:
        print('No planned trajectories found in %s' % args.planned_csv)
        return
    chosen_ids = pick_equally_spaced(sorted_ids, args.num_planned)

    t0 = actual_t[0] if actual_t else 0.0

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(actual_x, actual_y, actual_z, '-', color='tab:orange', linewidth=2, label='actual')

    colors = plt.cm.viridis([i / max(1, len(chosen_ids) - 1) for i in range(len(chosen_ids))])
    for color, traj_id in zip(colors, chosen_ids):
        entry = planned[traj_id]
        gen_t_rel = entry['gen_time_abs'] - t0
        ax.plot(entry['x'], entry['y'], entry['z'], '--', color=color,
                label='planned @ t=%.1fs' % gen_t_rel)

    ax.set_xlabel('x (N)')
    ax.set_ylabel('y (E)')
    ax.set_zlabel('z (D)')
    ax.set_title('Actual flight vs %d planned snapshots (NED)' % len(chosen_ids))
    ax.legend()
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
