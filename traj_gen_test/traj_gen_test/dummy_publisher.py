#!/usr/bin/env python3
"""Dummy data publisher to exercise the ros_traj_gen_utils traj_exe node.

Publishes the inputs traj_exe consumes:
  - PX4 odometry  -> /fmu/out/vehicle_odometry   (px4_msgs/VehicleOdometry, best-effort)
  - waypoints     -> <device>/waypoints          (nav_msgs/Path)
  - (optional) tag pose -> /tags_features_extractor/tag_pose (geometry_msgs/PoseStamped)

The odometry is published continuously (a hover by default) so the planner has a
start state and the apriltag odom buffer fills. The waypoints are published once
(or periodically) to trigger planning/execution.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('traj_gen_dummy_publisher')

        self.declare_parameter('device', '/quadrotor')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('odom_rate_hz', 50.0)
        self.declare_parameter('publish_tag', False)
        self.declare_parameter('tag_rate_hz', 20.0)
        # 0.0 -> publish the waypoints once (after a short delay); >0 -> repeat at this period (s)
        self.declare_parameter('waypoint_period_s', 0.0)
        # hover position the dummy reports, in PX4 NED (x=N, y=E, z=Down)
        self.declare_parameter('odom_ned', [0.0, 0.0, 0.0])
        # waypoints as a flat list [x, y, z, yaw, x, y, z, yaw, ...] in the world frame
        self.declare_parameter('waypoints', [1.0, 0.0, 1.0, 0.0,
                                             2.0, 1.0, 1.5, 0.0,
                                             3.0, 0.0, 1.0, 0.0])

        self.device = self.get_parameter('device').value
        self.frame_id = self.get_parameter('frame_id').value

        # PX4 publishes best-effort; the subscriber in traj_exe uses best-effort too.
        be_qos = QoSProfile(depth=1,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/out/vehicle_odometry', be_qos)
        self.wp_pub = self.create_publisher(Path, self.device + '/waypoints', 10)
        self.tag_pub = self.create_publisher(
            PoseStamped, '/tags_features_extractor/tag_pose', 10)

        odom_rate = float(self.get_parameter('odom_rate_hz').value)
        self.create_timer(1.0 / odom_rate, self.publish_odom)

        if bool(self.get_parameter('publish_tag').value):
            tag_rate = float(self.get_parameter('tag_rate_hz').value)
            self.create_timer(1.0 / tag_rate, self.publish_tag)

        wp_period = float(self.get_parameter('waypoint_period_s').value)
        self._wp_repeat = wp_period > 0.0
        interval = wp_period if self._wp_repeat else 2.0  # 2 s delay lets subscribers connect
        self.wp_timer = self.create_timer(interval, self.publish_waypoints)

        self.get_logger().info(
            'Dummy publisher up: odom -> /fmu/out/vehicle_odometry, '
            'waypoints -> %s/waypoints%s' %
            (self.device, ', tag -> /tags_features_extractor/tag_pose'
             if bool(self.get_parameter('publish_tag').value) else ''))

    def now_us(self):
        # PX4 timestamps are microseconds; the converter multiplies by 1000 -> ns.
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_odom(self):
        ned = list(self.get_parameter('odom_ned').value)
        msg = VehicleOdometry()
        msg.timestamp = self.now_us()
        msg.timestamp_sample = msg.timestamp
        msg.position = [float(ned[0]), float(ned[1]), float(ned[2])]
        msg.q = [1.0, 0.0, 0.0, 0.0]          # [w, x, y, z] identity
        msg.velocity = [0.0, 0.0, 0.0]
        msg.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(msg)

    def publish_waypoints(self):
        flat = list(self.get_parameter('waypoints').value)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for i in range(0, len(flat) - 3, 4):
            x, y, z, yaw = (float(flat[i]), float(flat[i + 1]),
                            float(flat[i + 2]), float(flat[i + 3]))
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.wp_pub.publish(path)
        self.get_logger().info('Published %d waypoints on %s/waypoints' %
                               (len(path.poses), self.device))
        if not self._wp_repeat:
            self.wp_timer.cancel()

    def publish_tag(self):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'camera'
        ps.pose.position.x = 0.0
        ps.pose.position.y = 0.0
        ps.pose.position.z = 2.0          # 2 m in front of the camera
        ps.pose.orientation.w = 1.0
        self.tag_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
