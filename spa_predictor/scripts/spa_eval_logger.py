#!/usr/bin/env python3
"""Diagnostic logger: records pad ground-truth odometry and SPA predictions
to CSV, for OFFLINE evaluation (see spa_eval_analyze.py) of prediction
accuracy vs. horizon. The online sigma_s self-assessment already tracks
something similar, but only as a per-horizon-bin EWMA; this keeps every
raw sample so error can be recomputed, re-binned, and plotted flexibly
after the fact -- and cross-checked against sigma_s, which it should agree
with roughly if both are working correctly.

Writes two files into --output_dir (default: current directory):
  pad_truth.csv        -- one row per VehicleOdometry message (ground truth)
  spa_predictions.csv  -- one row per (SpaPrediction message, horizon)
                           pair, "long" format, with t_target = made_at_t +
                           horizon_s already computed -- this is the
                           "shift" that lines a prediction up in time with
                           the ground-truth sample it was actually about.
                           Uses made_at_t (see SpaPrediction.msg), NOT
                           header.stamp -- the two are different clocks in
                           general (made_at_t is derived from the same PX4
                           timestamps addMeasurement() was fed; header.stamp
                           is ROS time at publish).

Run alongside spa_axes.launch.py (or a single spa_axis_node), e.g.:
  ros2 run spa_predictor spa_eval_logger.py --ros-args -p output_dir:=/tmp/spa_eval
Stop with Ctrl-C once enough data has been collected, then run
spa_eval_analyze.py on the two CSVs this produces.
"""
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from spa_predictor.msg import SpaPrediction


TRUTH_HEADER = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz']
PREDICTIONS_HEADER = ['axis', 't_made', 'horizon_s', 't_target',
                       'predicted_value', 'predicted_velocity', 'sigma_s',
                       'filtered_value', 'offset', 'dropped_samples']


class SpaEvalLogger(Node):
    def __init__(self):
        super().__init__('spa_eval_logger')

        self.declare_parameter('truth_topic', '/pad/fmu/out/vehicle_odometry')
        self.declare_parameter('x_topic', '/pad/spa/x_prediction')
        self.declare_parameter('y_topic', '/pad/spa/y_prediction')
        self.declare_parameter('heave_topic', '/pad/spa/heave_prediction')
        self.declare_parameter('output_dir', '.')
        self.declare_parameter('truth_csv_name', 'pad_truth.csv')
        self.declare_parameter('predictions_csv_name', 'spa_predictions.csv')

        truth_topic = self.get_parameter('truth_topic').value
        output_dir = self.get_parameter('output_dir').value
        os.makedirs(output_dir, exist_ok=True)

        truth_path = os.path.join(output_dir, self.get_parameter('truth_csv_name').value)
        predictions_path = os.path.join(
            output_dir, self.get_parameter('predictions_csv_name').value)

        # newline='' (correct csv-module practice) + explicit flush() after
        # every row (see the _on_* callbacks below) -- this tool is meant
        # to be stopped with Ctrl-C at an arbitrary moment, so nothing may
        # be left sitting in an unflushed buffer when that happens.
        self._truth_file = open(truth_path, 'w', newline='')
        self._truth_writer = csv.writer(self._truth_file)
        self._truth_writer.writerow(TRUTH_HEADER)

        self._pred_file = open(predictions_path, 'w', newline='')
        self._pred_writer = csv.writer(self._pred_file)
        self._pred_writer.writerow(PREDICTIONS_HEADER)

        self._truth_count = 0
        self._pred_count = 0

        # Matches the best-effort/depth-1 QoS every other PX4 topic in this
        # workspace uses (traj_manager.cpp, dummy_publisher.py, spa_axis_node.cpp).
        px4_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleOdometry, truth_topic, self._on_truth, px4_qos)

        # SpaPrediction publishers use the default (reliable, depth-10) QoS
        # (spa_axis_node.cpp: create_publisher<SpaPrediction>(topic, 10)) --
        # match it here, not best-effort, or a QoS mismatch can silently
        # drop messages from a reliable publisher.
        for axis, param in (('x', 'x_topic'), ('y', 'y_topic'), ('heave', 'heave_topic')):
            topic = self.get_parameter(param).value
            self.create_subscription(
                SpaPrediction, topic,
                lambda msg, axis=axis: self._on_prediction(axis, msg), 10)

        self.get_logger().info(
            'spa_eval_logger up: truth %s -> %s, predictions (x/y/heave) -> %s'
            % (truth_topic, truth_path, predictions_path))

    def _on_truth(self, msg):
        t = msg.timestamp * 1e-6
        self._truth_writer.writerow([
            t,
            msg.position[0], msg.position[1], msg.position[2],
            msg.velocity[0], msg.velocity[1], msg.velocity[2],
            msg.q[0], msg.q[1], msg.q[2], msg.q[3],
            msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2],
        ])
        self._truth_file.flush()
        self._truth_count += 1

    def _on_prediction(self, axis, msg):
        if not msg.initialized:
            return
        for i in range(len(msg.horizon_s)):
            h = msg.horizon_s[i]
            self._pred_writer.writerow([
                axis,
                msg.made_at_t,
                h,
                msg.made_at_t + h,
                msg.predicted_value[i],
                msg.predicted_velocity[i] if i < len(msg.predicted_velocity) else '',
                msg.sigma_s[i] if i < len(msg.sigma_s) else '',
                msg.filtered_value,
                msg.offset,
                msg.dropped_samples,
            ])
        self._pred_file.flush()
        self._pred_count += 1

    def close(self):
        self.get_logger().info(
            'spa_eval_logger shutting down: %d truth rows, %d prediction messages logged.'
            % (self._truth_count, self._pred_count))
        self._truth_file.close()
        self._pred_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = SpaEvalLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
