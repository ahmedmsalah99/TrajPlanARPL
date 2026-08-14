#!/usr/bin/env python3
"""Offline evaluation of spa_axis_node prediction accuracy, from CSVs
written by spa_eval_logger.py.

For every logged prediction (axis, made at t_made, for horizon h -- i.e.
"about" t_target = t_made + h), the ACTUAL value at t_target is recovered
by linearly interpolating the ground-truth CSV, giving
error = predicted - actual. Reports mean/median/RMSE/max error per
horizon, and plots:
  <axis>_shifted.png          -- predicted value (shifted to t_target) vs.
                                  ground truth, overlaid in time -- the
                                  "shifted predictions with the real value"
  <axis>_error.png            -- error (predicted - actual) vs. time, one
                                  line per horizon -- "the difference"
  <axis>_error_vs_horizon.png -- mean/median/RMSE/max |error| vs. horizon:
                                  the direct answer to "does accuracy
                                  degrade with horizon, and by how much"

Predictions whose t_target falls outside the logged ground-truth time
range are dropped (reported, not silently discarded) rather than compared
against a clamped/extrapolated non-value.

Requires pandas, numpy, matplotlib -- not ROS dependencies, pip install if
missing: pip install pandas numpy matplotlib

Usage:
  python3 spa_eval_analyze.py --truth pad_truth.csv --predictions spa_predictions.csv
  python3 spa_eval_analyze.py --axis heave --out-dir ./eval_plots --show
"""
import argparse
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# axis name (matches spa_axis_node's own axisName()/SpaPrediction "axis"
# tagging in spa_eval_logger.py) -> (position column, velocity column) in
# pad_truth.csv. NED: x=North, y=East, z=Down=heave -- see
# spa_axis_node.cpp's axis parameter comment.
AXIS_TRUTH_COLS = {
    'x': ('x', 'vx'),
    'y': ('y', 'vy'),
    'heave': ('z', 'vz'),
}


def load_data(truth_path, predictions_path):
    truth = pd.read_csv(truth_path).sort_values('t').reset_index(drop=True)
    predictions = pd.read_csv(predictions_path)
    return truth, predictions


def interpolate_truth(truth, t_query, pos_col, vel_col):
    """Linear-interpolate truth[pos_col]/truth[vel_col] at each time in
    t_query. Returns (pos, vel, in_range) -- in_range is False wherever
    t_query fell outside [truth.t.min(), truth.t.max()], since np.interp
    would otherwise silently clamp to the nearest endpoint instead of
    signaling "no ground truth available there". Callers must drop those
    rows rather than let a clamped, non-real value corrupt error stats."""
    t = truth['t'].to_numpy()
    pos = np.interp(t_query, t, truth[pos_col].to_numpy())
    vel = np.interp(t_query, t, truth[vel_col].to_numpy())
    in_range = (t_query >= t.min()) & (t_query <= t.max())
    return pos, vel, in_range


def compute_errors(axis_df, truth, pos_col, vel_col):
    t_target = axis_df['t_target'].to_numpy()
    true_pos, true_vel, in_range = interpolate_truth(truth, t_target, pos_col, vel_col)
    dropped = int((~in_range).sum())
    if dropped:
        print(f'  (dropped {dropped} prediction(s) whose t_target falls outside '
              f'the logged ground-truth time range)')
    df = axis_df.loc[in_range].copy()
    df['true_value'] = true_pos[in_range]
    df['true_velocity'] = true_vel[in_range]
    df['pos_error'] = df['predicted_value'] - df['true_value']
    df['vel_error'] = df['predicted_velocity'] - df['true_velocity']
    return df


def error_stats_by_horizon(df, error_col):
    def rmse(x):
        return float(np.sqrt(np.mean(np.square(x))))

    grouped = df.groupby('horizon_s')[error_col]
    stats = grouped.agg(
        n='count',
        mean_error='mean',
        median_error='median',
        max_abs_error=lambda x: float(np.max(np.abs(x))),
    )
    stats['rmse'] = grouped.apply(rmse)
    return stats.reset_index().sort_values('horizon_s')


def plot_shifted(df, truth, pos_col, axis, out_dir, show):
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(truth['t'], truth[pos_col], color='black', linewidth=2, label='ground truth')
    for h, group in df.groupby('horizon_s'):
        group = group.sort_values('t_target')
        ax.plot(group['t_target'], group['predicted_value'], linewidth=1,
                 alpha=0.8, label=f'predicted @ h={h:g}s')
    ax.set_xlabel('time (s)')
    ax.set_ylabel(pos_col)
    ax.set_title(f'{axis}: shifted predictions vs. ground truth')
    ax.legend(loc='upper right', fontsize='small', ncol=2)
    fig.tight_layout()
    path = os.path.join(out_dir, f'{axis}_shifted.png')
    fig.savefig(path, dpi=150)
    print(f'  wrote {path}')
    if show:
        plt.show(block=False)
    else:
        plt.close(fig)


def plot_error_over_time(df, axis, out_dir, show):
    fig, ax = plt.subplots(figsize=(12, 6))
    for h, group in df.groupby('horizon_s'):
        group = group.sort_values('t_target')
        ax.plot(group['t_target'], group['pos_error'], linewidth=1, alpha=0.8,
                 label=f'h={h:g}s')
    ax.axhline(0.0, color='black', linewidth=1, linestyle='--')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('predicted - actual')
    ax.set_title(f'{axis}: prediction error over time')
    ax.legend(loc='upper right', fontsize='small', ncol=2)
    fig.tight_layout()
    path = os.path.join(out_dir, f'{axis}_error.png')
    fig.savefig(path, dpi=150)
    print(f'  wrote {path}')
    if show:
        plt.show(block=False)
    else:
        plt.close(fig)


def plot_error_vs_horizon(stats, axis, out_dir, show):
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(stats['horizon_s'], stats['mean_error'], marker='o', label='mean error')
    ax.plot(stats['horizon_s'], stats['median_error'], marker='o', label='median error')
    ax.plot(stats['horizon_s'], stats['rmse'], marker='o', label='RMSE')
    ax.plot(stats['horizon_s'], stats['max_abs_error'], marker='o', label='max |error|')
    ax.axhline(0.0, color='black', linewidth=1, linestyle='--')
    ax.set_xlabel('horizon (s)')
    ax.set_ylabel('error')
    ax.set_title(f'{axis}: error vs. prediction horizon')
    ax.legend(loc='best', fontsize='small')
    fig.tight_layout()
    path = os.path.join(out_dir, f'{axis}_error_vs_horizon.png')
    fig.savefig(path, dpi=150)
    print(f'  wrote {path}')
    if show:
        plt.show(block=False)
    else:
        plt.close(fig)


def run_axis(axis, truth, predictions, out_dir, show):
    pos_col, vel_col = AXIS_TRUTH_COLS[axis]
    axis_df = predictions[predictions['axis'] == axis]
    if axis_df.empty:
        print(f'[{axis}] no predictions logged -- skipping.')
        return
    print(f'[{axis}] {len(axis_df)} logged prediction rows')
    df = compute_errors(axis_df, truth, pos_col, vel_col)
    if df.empty:
        print(f'[{axis}] no predictions with ground truth in range -- skipping.')
        return

    stats = error_stats_by_horizon(df, 'pos_error')
    print(f'[{axis}] position error by horizon:')
    print(stats.to_string(index=False))
    stats_path = os.path.join(out_dir, f'{axis}_error_stats.csv')
    stats.to_csv(stats_path, index=False)
    print(f'  wrote {stats_path}')

    plot_shifted(df, truth, pos_col, axis, out_dir, show)
    plot_error_over_time(df, axis, out_dir, show)
    plot_error_vs_horizon(stats, axis, out_dir, show)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--truth', default='pad_truth.csv')
    parser.add_argument('--predictions', default='spa_predictions.csv')
    parser.add_argument('--axis', choices=['x', 'y', 'heave', 'all'], default='all')
    parser.add_argument('--out-dir', default='.')
    parser.add_argument('--show', action='store_true',
                         help='also open interactive plot windows (default: save PNGs only)')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    truth, predictions = load_data(args.truth, args.predictions)

    axes = ['x', 'y', 'heave'] if args.axis == 'all' else [args.axis]
    for axis in axes:
        run_axis(axis, truth, predictions, args.out_dir, args.show)

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
