#!/usr/bin/env python3
"""PX4 offboard bridge: relays the planner's trajectory command to PX4.

Subscribes to the planner's quadrotor_msgs/PositionCommand (published by
ros_traj_gen_utils' poscmd_publisher on <device>/position_cmd) and republishes
it as a px4_msgs/TrajectorySetpoint on /fmu/in/trajectory_setpoint, alongside
the px4_msgs/OffboardControlMode heartbeat PX4 requires both to enter and to
remain in offboard mode.

The stream (heartbeat + setpoint relay) does NOT start at node startup. PX4
only allows entering OFFBOARD once a valid stream is already flowing -- so if
an RC switch or QGC mode selector is already parked on Offboard when the
stream begins, PX4 grants that pending request the instant it becomes
possible, with no explicit command from this node. That is a surprise mode
switch at whatever moment the bridge happens to start, not something the
operator deliberately asked for right then.

Instead, the stream is gated behind the enable_offboard/disable_offboard
services (std_srvs/Trigger). Calling enable_offboard starts the heartbeat +
setpoint relay, waits offboard_prime_s for the stream to establish (PX4's own
precondition for entering offboard), then sends a single explicit
px4_msgs/VehicleCommand (DO_SET_MODE -> OFFBOARD) -- mirroring PX4's own
recommended stream-first-then-switch sequence, just moved from "whatever the
RC switch happens to be doing" to an explicit, single, timed operator action.
disable_offboard stops the stream again (PX4 falls out of offboard on its own
once the stream lapses).

Arming is deliberately still out of scope -- left to RC/QGroundControl/your
own tooling. enable_offboard only requests the OFFBOARD mode switch; it does
not arm the vehicle.

Both quadrotor_msgs::msg::PositionCommand and this repo's whole planning
pipeline are NED-native (see vehicleOdometryToRosOdometry's "NED/FRD
passthrough" comment in traj_manager.cpp) and px4_msgs uses the same NED/FRD
convention, so all fields are copied straight across with no axis remapping.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from quadrotor_msgs.msg import PositionCommand
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


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
        # How long to stream heartbeat+setpoints after enable_offboard is
        # called before sending the DO_SET_MODE command -- gives PX4 time to
        # see the stream as established (its own precondition for offboard).
        self.declare_parameter('offboard_prime_s', 1.0)

        device = self.get_parameter('device').value
        offboard_rate_hz = float(self.get_parameter('offboard_rate_hz').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self.offboard_prime_s = float(self.get_parameter('offboard_prime_s').value)

        # PX4 topics are best-effort, depth-1 -- matches the QoS already used
        # for VehicleOdometry elsewhere in this repo (traj_manager.cpp,
        # dummy_publisher.py).
        px4_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.wp_pub = self.create_publisher(Path, device + '/waypoints', 10)

        self.cmd_sub = self.create_subscription(
            PositionCommand, device + '/position_cmd', self._on_position_cmd, 10)

        self._last_cmd_time = None
        self._warned_stale = False

        # Stream is off until enable_offboard is called -- see module docstring.
        self._streaming = False
        self._streaming_since = None
        self._mode_cmd_sent = False

        self.create_timer(1.0 / offboard_rate_hz, self._publish_offboard_heartbeat)
        # Independent watchdog timer for the staleness warning, decoupled from
        # both the heartbeat rate and the (external) command rate.
        self.create_timer(0.25, self._check_stale)

        self.enable_srv = self.create_service(
            Trigger, 'enable_offboard', self._on_enable_offboard)
        self.disable_srv = self.create_service(
            Trigger, 'disable_offboard', self._on_disable_offboard)

        self.get_logger().info(
            'PX4 offboard bridge up: %s/position_cmd -> /fmu/in/trajectory_setpoint, '
            'heartbeat -> /fmu/in/offboard_control_mode @ %.1f Hz. '
            'Stream is OFF until the enable_offboard service is called '
            '(disable_offboard stops it again). Arming is NOT handled here.'
            % (device, offboard_rate_hz))

    def _now_us(self):
        # PX4 timestamps are microseconds.
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_offboard_heartbeat(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'world'
        ps = PoseStamped()
        path.poses.append(ps)
        self.wp_pub.publish(path)

        if not self._streaming:
            return

        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

        if not self._mode_cmd_sent:
            elapsed_s = (self.get_clock().now().nanoseconds - self._streaming_since) / 1e9
            if elapsed_s >= self.offboard_prime_s:
                self._send_offboard_mode_command()
                self._mode_cmd_sent = True
                self.get_logger().info(
                    'Stream established for %.2fs, requesting OFFBOARD mode switch.'
                    % elapsed_s)

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

        if not self._streaming:
            return

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

    def _send_offboard_mode_command(self):
        # DO_SET_MODE(base_mode=CUSTOM, custom_main_mode=OFFBOARD), the same
        # single command PX4's own ROS2 offboard example sends after priming
        # the stream. Field layout mirrors px4_msgs::msg::VehicleCommand as
        # used elsewhere in PX4's official examples; not build-verified in
        # this sandbox since px4_msgs isn't installed here.
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _on_enable_offboard(self, request, response):
        if self._streaming:
            response.success = False
            response.message = 'Already streaming (call disable_offboard first to restart).'
            return response

        if self._last_cmd_time is None:
            response.success = False
            response.message = 'No PositionCommand received yet -- refusing to enable.'
            return response
        age_s = (self.get_clock().now().nanoseconds - self._last_cmd_time) / 1e9
        if age_s > self.command_timeout_s:
            response.success = False
            response.message = (
                'Last PositionCommand is %.2fs old (timeout=%.2fs) -- refusing to enable.'
                % (age_s, self.command_timeout_s))
            return response

        self._streaming = True
        self._streaming_since = self.get_clock().now().nanoseconds
        self._mode_cmd_sent = False
        response.success = True
        response.message = (
            'Streaming started, requesting OFFBOARD in %.2fs.' % self.offboard_prime_s)
        self.get_logger().info(response.message)
        return response

    def _on_disable_offboard(self, request, response):
        self._streaming = False
        self._streaming_since = None
        self._mode_cmd_sent = False
        response.success = True
        response.message = 'Streaming stopped.'
        self.get_logger().info(response.message)
        return response


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
