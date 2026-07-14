#!/usr/bin/env python3
"""Plot the frozen plan's assumed position/velocity vs the recorded actual
vehicle state, for the plan-vs-actual debug capture in traj_manager.cpp
(this branch only).

Usage:
    python3 plot_plan_vs_actual.py [planned.csv] [actual.csv]

Defaults match traj_manager.cpp's default CSV paths:
    planned.csv -> /tmp/planned_trajectory.csv
    actual.csv  -> /tmp/actual_trajectory.csv

Both CSVs start with a comment line "# t0_abs,<seconds>" giving the
absolute (node clock) wall time of that file's own t=0, followed by the
header row t,x,y,z,vx,vy,vz:
- planned.csv's t is seconds since the frozen plan's own t=0 (the moment
  it was solved).
- actual.csv's t is seconds since start_replan was called (i.e. since
  offboard was confirmed and recording began).

These two time origins are NOT the same instant in general -- the plan
can freeze well before offboard is actually requested. Since both t0_abs
values come from the same node clock, this script uses them to compute
the exact offset and shifts the actual trace onto the planned trace's
time axis automatically -- no guessing required. Pass --shift SECONDS to
add a further manual nudge on top of that computed offset (e.g. to
compensate for a suspected fixed latency elsewhere in the pipeline).
"""
import argparse
import csv
import sys

import matplotlib.pyplot as plt


def load_csv(path):
    """Returns (data_dict, t0_abs). t0_abs is None if the file predates the
    "# t0_abs,<seconds>" leading comment line (older capture, no auto-align
    possible -- falls back to aligning both at t=0)."""
    t, x, y, z, vx, vy, vz = [], [], [], [], [], [], []
    t0_abs = None
    with open(path, newline='') as f:
        first_line = f.readline()
        if first_line.startswith('# t0_abs,'):
            t0_abs = float(first_line.split(',', 1)[1])
        else:
            f.seek(0)
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            z.append(float(row['z']))
            vx.append(float(row['vx']))
            vy.append(float(row['vy']))
            vz.append(float(row['vz']))
    return {'t': t, 'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}, t0_abs


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('planned_csv', nargs='?', default='/tmp/planned_trajectory.csv')
    parser.add_argument('actual_csv', nargs='?', default='/tmp/actual_trajectory.csv')
    parser.add_argument('--shift', type=float, default=0.0,
                         help='additional seconds to add to the actual trace\'s time '
                              'axis, on top of the offset auto-computed from each '
                              'file\'s t0_abs')
    args = parser.parse_args()

    try:
        planned, planned_t0_abs = load_csv(args.planned_csv)
    except FileNotFoundError:
        print('Could not find planned CSV at %s -- has the plan frozen yet '
              '([PLAN_VS_ACTUAL] visual target held steady... in the logs)?'
              % args.planned_csv, file=sys.stderr)
        sys.exit(1)

    try:
        actual, actual_t0_abs = load_csv(args.actual_csv)
    except FileNotFoundError:
        print('Could not find actual CSV at %s -- has start_replan been called yet '
              '([PLAN_VS_ACTUAL] start_replan called... in the logs)?'
              % args.actual_csv, file=sys.stderr)
        sys.exit(1)

    if planned_t0_abs is not None and actual_t0_abs is not None:
        auto_shift = actual_t0_abs - planned_t0_abs
        print('Auto-aligning: actual recording started %.3fs after the plan froze '
              '(actual t0_abs=%.3f, planned t0_abs=%.3f)'
              % (auto_shift, actual_t0_abs, planned_t0_abs))
    else:
        auto_shift = 0.0
        print('No t0_abs found in one or both CSVs (older capture?) -- '
              'falling back to aligning both at t=0.', file=sys.stderr)

    total_shift = auto_shift + args.shift
    actual_t = [t + total_shift for t in actual['t']]

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
