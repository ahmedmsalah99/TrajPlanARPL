#!/usr/bin/env python3
"""Plot the frozen plan's assumed position/velocity vs the recorded actual
vehicle state, for the plan-vs-actual debug capture in traj_manager.cpp
(this branch only).

Usage:
    python3 plot_plan_vs_actual.py [planned.csv] [actual.csv]

Defaults match traj_manager.cpp's default CSV paths:
    planned.csv -> /tmp/planned_trajectory.csv
    actual.csv  -> /tmp/actual_trajectory.csv

Both CSVs are expected to have a header row: t,x,y,z,vx,vy,vz
- planned.csv's t is seconds since the frozen plan's own t=0 (the moment
  it was solved).
- actual.csv's t is seconds since start_replan was called (i.e. since
  offboard was confirmed and recording began).

These two time origins are NOT the same instant in general -- the plan
can freeze well before offboard is actually requested. This script plots
both against their own t=0 on a shared time axis for a first look; if the
plan was frozen and then offboard was requested shortly after (the
intended debug procedure), the two should already be close enough to
compare directly. Pass --shift SECONDS to manually slide the actual
trace if you know the real offset between the two starts.
"""
import argparse
import csv
import sys

import matplotlib.pyplot as plt


def load_csv(path):
    t, x, y, z, vx, vy, vz = [], [], [], [], [], [], []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            z.append(float(row['z']))
            vx.append(float(row['vx']))
            vy.append(float(row['vy']))
            vz.append(float(row['vz']))
    return {'t': t, 'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('planned_csv', nargs='?', default='/tmp/planned_trajectory.csv')
    parser.add_argument('actual_csv', nargs='?', default='/tmp/actual_trajectory.csv')
    parser.add_argument('--shift', type=float, default=0.0,
                         help='seconds to add to the actual trace\'s time axis, '
                              'to manually align it with the planned trace')
    args = parser.parse_args()

    try:
        planned = load_csv(args.planned_csv)
    except FileNotFoundError:
        print('Could not find planned CSV at %s -- has the plan frozen yet '
              '([PLAN_VS_ACTUAL] visual target held steady... in the logs)?'
              % args.planned_csv, file=sys.stderr)
        sys.exit(1)

    try:
        actual = load_csv(args.actual_csv)
    except FileNotFoundError:
        print('Could not find actual CSV at %s -- has start_replan been called yet '
              '([PLAN_VS_ACTUAL] start_replan called... in the logs)?'
              % args.actual_csv, file=sys.stderr)
        sys.exit(1)

    actual_t = [t + args.shift for t in actual['t']]

    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
    labels = ['x', 'y', 'z', 'vx', 'vy', 'vz']
    for i, key in enumerate(labels):
        ax = axes[i // 3, i % 3]
        ax.plot(planned['t'], planned[key], '--', label='planned', color='tab:blue')
        ax.plot(actual_t, actual[key], '-', label='actual', color='tab:orange')
        ax.set_title(key)
        ax.set_xlabel('t (s)')
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend()

    fig.suptitle('Planned vs actual trajectory (NED)')
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
