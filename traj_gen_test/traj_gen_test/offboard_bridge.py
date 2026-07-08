#!/usr/bin/env python3
"""PX4 offboard bridge: relays the planner's trajectory command to PX4.

Subscribes to the planner's quadrotor_msgs/PositionCommand (published by
ros_traj_gen_utils' poscmd_publisher on <device>/position_cmd) and republishes
it as a px4_msgs/TrajectorySetpoint on /fmu/in/trajectory_setpoint. Also
continuously publishes px4_msgs/OffboardControlMode, which PX4 requires as a
heartbeat both to enter and to remain in offboard mode.

Deliberately out of scope: arming and flight-mode switching (px4_msgs/
VehicleCommand). Commanding a real vehicle into OFFBOARD/arming it is left to
an explicit, separate action (RC switch, QGroundControl, or your own tooling)
-- this node only streams setpoints once you're already in offboard mode, and
provides the heartbeat PX4 needs to accept that mode switch when you request
it.

Both quadrotor_msgs::msg::PositionCommand and this repo's whole planning
pipeline are NED-native (see vehicleOdometryToRosOdometry's "NED/FRD
passthrough" comment in traj_manager.cpp) and px4_msgs uses the same NED/FRD
convention, so all fields are copied straight across with no axis remapping.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from quadrotor_msgs.msg import PositionCommand
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint


class OffboardBridge(Node):
    def __init__(self):
        super().__init__('px4_offboard_bridge')

        self.declare_parameter('device', '/quadrotor')
        # Rate to publish the OffboardControlMode heartbeat at. PX4 requires
        # this at a minimum of ~2 Hz to stay in offboard mode; default is
        # comfortably above that.
        self.declare_parameter('offboard_rate_hz', 50.0)
        # If no PositionCommand has been received in this long, log a
        # (throttled) warning instead of silently going stale. Informational
        # only -- this node does not take any autonomous failsafe action.
        self.declare_parameter('command_timeout_s', 0.5)

        device = self.get_parameter('device').value
        offboard_rate_hz = float(self.get_parameter('offboard_rate_hz').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)

        # PX4 topics are best-effort, depth-1 -- matches the QoS already used
        # for VehicleOdometry elsewhere in this repo (traj_manager.cpp,
        # dummy_publisher.py).
        px4_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)

        self.cmd_sub = self.create_subscription(
            PositionCommand, device + '/position_cmd', self._on_position_cmd, 10)

        self._last_cmd_time = None
        self._warned_stale = False

        self.create_timer(1.0 / offboard_rate_hz, self._publish_offboard_heartbeat)
        # Independent watchdog timer for the staleness warning, decoupled from
        # both the heartbeat rate and the (external) command rate.
        self.create_timer(0.25, self._check_stale)

        self.get_logger().info(
            'PX4 offboard bridge up: %s/position_cmd -> /fmu/in/trajectory_setpoint, '
            'heartbeat -> /fmu/in/offboard_control_mode @ %.1f Hz. '
            'Arming/mode-switch NOT handled here -- trigger offboard mode externally.'
            % (device, offboard_rate_hz))

    def _now_us(self):
        # PX4 timestamps are microseconds.
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def _check_stale(self):
        if self._last_cmd_time is None:
            return
        age_s = (self.get_clock().now().nanoseconds - self._last_cmd_time) / 1e9
        if age_s > self.command_timeout_s:
            if not self._warned_stale:
                self.get_logger().warn(
                    'No PositionCommand received in %.2fs (timeout=%.2fs) -- '
                    'trajectory_setpoint is no longer being updated.'
                    % (age_s, self.command_timeout_s))
                self._warned_stale = True
        else:
            self._warned_stale = False

    def _on_position_cmd(self, msg: PositionCommand):
        self._last_cmd_time = self.get_clock().now().nanoseconds

        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        sp.position[0] = msg.position.x
        sp.position[1] = msg.position.y
        sp.position[2] = msg.position.z
        sp.velocity[0] = msg.velocity.x
        sp.velocity[1] = msg.velocity.y
        sp.velocity[2] = msg.velocity.z
        sp.acceleration[0] = msg.acceleration.x
        sp.acceleration[1] = msg.acceleration.y
        sp.acceleration[2] = msg.acceleration.z
        sp.jerk[0] = msg.jerk.x
        sp.jerk[1] = msg.jerk.y
        sp.jerk[2] = msg.jerk.z
        sp.yaw = msg.yaw
        sp.yawspeed = msg.yaw_dot
        self.setpoint_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardBridge()
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
