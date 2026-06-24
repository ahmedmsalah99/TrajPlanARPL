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
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('traj_gen_dummy_publisher')

        self.declare_parameter('device', '/quadrotor')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('odom_rate_hz', 50.0)
        self.declare_parameter('publish_tag', True)
        self.declare_parameter('tag_rate_hz', 20.0)
        # 0.0 -> publish the waypoints once (after a short delay); >0 -> repeat at this period (s)
        self.declare_parameter('waypoint_period_s', 0.1)
        # hover position the dummy reports, in PX4 NED (x=N, y=E, z=Down)
        self.declare_parameter('odom_ned', [0.0, 0.0, 0.0])
        # waypoints as a flat list [x, y, z, yaw, x, y, z, yaw, ...] in the world frame.
        # The planner is NED-native, so z is Down: negative z = altitude above origin.
        self.declare_parameter('waypoints', [1.0, 0.0, -1.0, 0.0,
                                             2.0, 1.0, -1.5, 0.0,
                                             3.0, 0.0, -1.0, 0.0])
        self.declare_parameter('tag_pose',[4.0, 1.0, 2.0, 0.0,1.57,0.0])
        # TF so RViz can resolve the planner's frames (set Fixed Frame = fixed_frame)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('fixed_frame', 'map')
        # The planner now works in NED (z-down). RViz renders z-up, so NED data would
        # appear upside down. With this enabled the fixed_frame is treated as ENU and
        # the static fixed_frame->{planner frames} transforms carry the ENU<->NED flip,
        # so NED trajectories/waypoints display right-side-up. Set Fixed Frame = fixed_frame.
        self.declare_parameter('rviz_enu_flip', True)

        self.device = self.get_parameter('device').value
        self.frame_id = self.get_parameter('frame_id').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.rviz_enu_flip = bool(self.get_parameter('rviz_enu_flip').value)

        if self.publish_tf:
            # static transforms: fixed_frame -> {planner frames}, and a camera frame
            # under the drone for the tag pose. Dynamic odom->base_link (the drone
            # pose) is sent in publish_odom.
            # When rviz_enu_flip is on, fixed_frame is ENU and the planner frames are
            # NED; the ENU<->NED flip is a 180 deg rotation about the N=E axis, i.e.
            # quaternion (x, y, z, w) = (sqrt(2)/2, sqrt(2)/2, 0, 0). Otherwise identity.
            flip = (0.70710678, 0.70710678, 0.0, 0.0) if self.rviz_enu_flip \
                else (0.0, 0.0, 0.0, 1.0)
            self.static_tf = StaticTransformBroadcaster(self)
            self.tf_bcast = TransformBroadcaster(self)
            statics = []
            seen = set()
            for f in [self.frame_id, 'world', 'simulator', 'mocap']:
                if f == self.fixed_frame or f in seen:
                    continue
                seen.add(f)
                statics.append(self._static_tf(self.fixed_frame, f, flip))
            statics.append(self._identity_tf('base_link', 'camera'))
            self.static_tf.sendTransform(statics)


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

    def _identity_tf(self, parent, child):
        return self._static_tf(parent, child, (0.0, 0.0, 0.0, 1.0))

    def _static_tf(self, parent, child, quat):
        # quat is (x, y, z, w)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        return t

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

        if self.publish_tf:
            # dynamic drone pose: parent = odom frame (NED), child = base_link.
            # The planner is NED-native, so the pose is published as-is; the static
            # fixed_frame->odom flip (if enabled) handles the RViz z-up display.
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(ned[0])
            t.transform.translation.y = float(ned[1])
            t.transform.translation.z = float(ned[2])
            t.transform.rotation.w = 1.0
            self.tf_bcast.sendTransform(t)

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
        tag_pose = list(self.get_parameter('tag_pose').value)
        q = self.rpy_to_quaternion(tag_pose[3],tag_pose[4],tag_pose[5])
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'camera'
        ps.pose.position.x = tag_pose[0]
        ps.pose.position.y = tag_pose[1]
        ps.pose.position.z = tag_pose[2]          # in front of the camera
        # rpy_to_quaternion returns (x, y, z, w)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        self.tag_pub.publish(ps)

    def rpy_to_quaternion(self,roll, pitch, yaw, degrees=False):
        """
        Convert roll, pitch, yaw to quaternion.

        Parameters:
            roll, pitch, yaw: Euler angles
            degrees: if True, input is in degrees

        Returns:
            (x, y, z, w) quaternion
        """

        if degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (x, y, z, w)
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
