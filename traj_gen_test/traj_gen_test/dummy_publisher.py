#!/usr/bin/env python3
"""Dummy data publisher to exercise the ros_traj_gen_utils traj_exe node.

Publishes the inputs traj_exe consumes:
  - PX4 odometry  -> /fmu/out/vehicle_odometry   (px4_msgs/VehicleOdometry, best-effort)
  - waypoints     -> <device>/waypoints          (nav_msgs/Path)
  - (optional) tag pose -> /tags_features_extractor/tag_pose (geometry_msgs/PoseStamped)

The odometry is published continuously (a hover by default) so the planner has a
start state and the apriltag odom buffer fills. The waypoints are published once
(or periodically) to trigger planning/execution.

The 'tag_pose' parameter is configured relative to the BODY frame (intuitive:
"pad 4m ahead, 2m below" reads the same no matter how the camera is mounted).
publish_tag() inverts cam_translation/body_r_cam/theta_from_nadir_deg to
publish the equivalent CAMERA-frame pose, which is what a real AprilTag
detector actually reports and what apriltag_utils::WorldRot expects on this
topic. With identity extrinsics (the default) this is a no-op.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def _mat3_mul(a, b):
    """3x3 matrix multiply, a and b as 3x3 nested lists."""
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat3_to_quat(r):
    """Standard robust rotation-matrix -> quaternion (x, y, z, w)."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2][1] - r[1][2]) * s
        y = (r[0][2] - r[2][0]) * s
        z = (r[1][0] - r[0][1]) * s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2])
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2])
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1])
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _mat3_transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _mat3_vec_mul(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _quat_to_mat3(q):
    """Quaternion (x, y, z, w) -> rotation matrix (3x3 nested list)."""
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _cam_to_body_mat(body_r_cam_flat, theta_from_nadir_deg):
    """Camera-frame -> body-frame rotation matrix, mirroring traj_manager.cpp's
    R = Rtilt_y(theta_from_hor) * body_R_cam."""
    body_r_cam = [list(body_r_cam_flat[0:3]), list(body_r_cam_flat[3:6]), list(body_r_cam_flat[6:9])]
    theta_from_hor = (90.0 - theta_from_nadir_deg) * (math.pi / 180.0)
    c, s = math.cos(theta_from_hor), math.sin(theta_from_hor)
    r_tilt_y = [[c, 0.0, s],
                [0.0, 1.0, 0.0],
                [-s, 0.0, c]]
    return _mat3_mul(r_tilt_y, body_r_cam)


def _cam_to_body_quat(body_r_cam_flat, theta_from_nadir_deg):
    """Camera-frame -> body-frame rotation, as a TF quaternion (x, y, z, w)."""
    return _mat3_to_quat(_cam_to_body_mat(body_r_cam_flat, theta_from_nadir_deg))


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
        # densify the waypoint list so no two consecutive points are farther apart
        # than this (metres); <= 0 disables interpolation and publishes them as-is
        self.declare_parameter('max_segment_len', 1.0)
        # Target pose [x, y, z, roll, pitch, yaw] relative to the BODY frame --
        # NOT the camera frame the message is actually published in. This is
        # deliberately intuitive to configure (e.g. "flat pad 4m ahead, 2m
        # below" reads the same regardless of how the camera is mounted):
        # publish_tag() inverts cam_translation/body_r_cam/theta_from_nadir_deg
        # to compute and publish the equivalent camera-frame pose, which is
        # what a real AprilTag detector would actually report (tag pose as
        # seen by the camera). If those extrinsics are identity (the default),
        # this transform is a no-op and body frame == camera frame.
        self.declare_parameter('tag_pose',[4.0, 1.0, 2.0, 0.0,1.57,0.0])
        # camera offset in the body (base_link) frame, in metres. Must match the
        # planner's cam_translation so the dummy's tag (published in the 'camera'
        # frame) maps to the same world target the planner computes.
        self.declare_parameter('cam_translation', [0.0, 0.0, 0.0])
        # Camera-frame -> body-frame rotation, mirroring the planner's own
        # extrinsics (traj_manager.cpp): R = Rtilt_y(theta_from_hor) * body_R_cam.
        # Only used to build the 'camera' static TF below for correct RViz
        # visualization -- the planner computes its own H_RC independently from
        # its own config, not from TF, so these don't need to match exactly for
        # planning to work, but should match if you want the visualized camera
        # frame to reflect what the planner is actually assuming.
        # body_r_cam: fixed mount convention (row-major 3x3). Defaults to
        # identity (a body-frame-aligned camera); set to match a real rig's
        # nadir-facing (or other) mount, same as the planner's body_r_cam param.
        self.declare_parameter('body_r_cam', [1.0, 0.0, 0.0,
                                              0.0, 1.0, 0.0,
                                              0.0, 0.0, 1.0])
        # theta_from_nadir_deg: additional tilt UP from nadir (deg), same as the
        # planner's theta_from_nadir_deg param. 90 = no additional tilt.
        self.declare_parameter('theta_from_nadir_deg', 90.0)
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
        self.max_segment_len = float(self.get_parameter('max_segment_len').value)
        self.cam_translation = list(self.get_parameter('cam_translation').value)
        self.body_r_cam = list(self.get_parameter('body_r_cam').value)
        self.theta_from_nadir_deg = float(self.get_parameter('theta_from_nadir_deg').value)

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
            # camera under base_link, offset by cam_translation and rotated by
            # body_r_cam/theta_from_nadir_deg -- same construction as the
            # planner's own extrinsics (traj_manager.cpp), so the visualized
            # camera frame matches what the planner is actually assuming.
            cam_quat = _cam_to_body_quat(self.body_r_cam, self.theta_from_nadir_deg)
            statics.append(self._static_tf('base_link', 'camera',
                                           cam_quat, self.cam_translation))
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

    def _static_tf(self, parent, child, quat, trans=(0.0, 0.0, 0.0)):
        # quat is (x, y, z, w); trans is (x, y, z)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])
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

    def _densify(self, pts):
        """Insert interpolated waypoints so no consecutive pair (in x,y,z) is
        farther apart than max_segment_len. yaw is interpolated linearly."""
        if self.max_segment_len <= 0.0 or len(pts) < 2:
            return pts
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            dist = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 +
                             (b[2] - a[2]) ** 2)
            n = max(1, int(math.ceil(dist / self.max_segment_len)))
            for k in range(1, n + 1):
                t = k / n
                out.append((a[0] + (b[0] - a[0]) * t,
                            a[1] + (b[1] - a[1]) * t,
                            a[2] + (b[2] - a[2]) * t,
                            a[3] + (b[3] - a[3]) * t))
        return out

    def publish_waypoints(self):
        flat = list(self.get_parameter('waypoints').value)
        pts = [(float(flat[i]), float(flat[i + 1]),
                float(flat[i + 2]), float(flat[i + 3]))
               for i in range(0, len(flat) - 3, 4)]
        # pts = self._densify(pts)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for x, y, z, yaw in pts:
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
        # tag_pose is configured relative to the BODY frame (see the parameter
        # doc above). Convert it to the CAMERA frame before publishing, since
        # that's the frame a real AprilTag detector reports in and the frame
        # apriltag_utils::WorldRot expects on this topic:
        #   p_body = cam_to_body_R * p_cam + cam_translation
        #   R_body = cam_to_body_R * R_cam
        # so, solving for the camera-frame pose (cam_to_body_R is a rotation
        # matrix, so its inverse is its transpose):
        #   p_cam = cam_to_body_R^T * (p_body - cam_translation)
        #   R_cam = cam_to_body_R^T * R_body
        tag_pose = list(self.get_parameter('tag_pose').value)
        p_body = tag_pose[0:3]
        q_body = self.rpy_to_quaternion(tag_pose[3], tag_pose[4], tag_pose[5])
        r_body = _quat_to_mat3(q_body)

        cam_to_body_r = _cam_to_body_mat(self.body_r_cam, self.theta_from_nadir_deg)
        body_to_cam_r = _mat3_transpose(cam_to_body_r)

        p_offset = [p_body[i] - self.cam_translation[i] for i in range(3)]
        p_cam = _mat3_vec_mul(body_to_cam_r, p_offset)
        r_cam = _mat3_mul(body_to_cam_r, r_body)
        q_cam = _mat3_to_quat(r_cam)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'camera'
        ps.pose.position.x = p_cam[0]
        ps.pose.position.y = p_cam[1]
        ps.pose.position.z = p_cam[2]
        ps.pose.orientation.x = q_cam[0]
        ps.pose.orientation.y = q_cam[1]
        ps.pose.orientation.z = q_cam[2]
        ps.pose.orientation.w = q_cam[3]
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
