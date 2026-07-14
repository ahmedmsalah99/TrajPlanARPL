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

Once OFFBOARD is confirmed active (by watching px4_msgs/VehicleStatus.nav_state,
not just by having sent the mode-switch command -- PX4 can reject/delay it),
this node calls the planner's (ros_traj_gen_utils' traj_exe) start_replan
service. Until then, traj_exe only publishes its initial plan on a loop and
never calls replan() -- see traj_manager.cpp's g_replanEnabled. This avoids
replan()'s elapsed-time bookkeeping running (on a real wall-clock timer)
while the vehicle isn't actually being driven by the plan yet, which used to
walk the "predicted current/future point" evaluation right off the end of
the segment and lock onto the trajectory's terminal (e.g. perch contact)
acceleration instead of the vehicle's real, ~zero, hovering acceleration.
disable_offboard calls stop_replan too, tying "leave offboard" and "stop
replanning" together.

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
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus


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
        # After sending DO_SET_MODE, how long to wait for VehicleStatus to
        # confirm OFFBOARD before logging a (one-time, non-fatal) warning that
        # it's taking unusually long. Checking continues either way -- this is
        # informational, not a give-up deadline.
        self.declare_parameter('offboard_confirm_timeout_s', 5.0)

        device = self.get_parameter('device').value
        offboard_rate_hz = float(self.get_parameter('offboard_rate_hz').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self.offboard_prime_s = float(self.get_parameter('offboard_prime_s').value)
        self.offboard_confirm_timeout_s = float(
            self.get_parameter('offboard_confirm_timeout_s').value)

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
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_vehicle_status, px4_qos)

        # traj_exe's replan gate (see traj_manager.cpp's g_replanEnabled) --
        # same vehicle_name-prefixed service naming convention as its other
        # services (mav_services/hover, trackers_manager/transition).
        self.start_replan_client = self.create_client(Trigger, device + '/start_replan')
        self.stop_replan_client = self.create_client(Trigger, device + '/stop_replan')

        self._last_cmd_time = None
        self._warned_stale = False
        self._nav_state = None

        # Stream is off until enable_offboard is called -- see module docstring.
        self._streaming = False
        self._streaming_since = None
        self._mode_cmd_sent = False
        self._mode_cmd_sent_at = None
        self._replan_started = False
        self._warned_confirm_slow = False

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
            '(disable_offboard stops it and calls %s/stop_replan). Once OFFBOARD is '
            'confirmed via VehicleStatus, %s/start_replan is called. Arming is NOT '
            'handled here.'
            % (device, offboard_rate_hz, device, device))

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
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

        if not self._mode_cmd_sent:
            elapsed_s = (self.get_clock().now().nanoseconds - self._streaming_since) / 1e9
            if elapsed_s >= self.offboard_prime_s:
                self._send_offboard_mode_command()
                self._mode_cmd_sent = True
                self._mode_cmd_sent_at = self.get_clock().now().nanoseconds
                self.get_logger().info(
                    'Stream established for %.2fs, requesting OFFBOARD mode switch.'
                    % elapsed_s)
        elif not self._replan_started:
            self._check_offboard_confirmed_and_start_replan()

    def _on_vehicle_status(self, msg: VehicleStatus):
        self._nav_state = msg.nav_state

    def _check_offboard_confirmed_and_start_replan(self):
        if self._nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self._replan_started = True
            self._warned_confirm_slow = False
            self.get_logger().info(
                'VehicleStatus confirms OFFBOARD is active -- calling start_replan.')
            if self.start_replan_client.service_is_ready():
                self.start_replan_client.call_async(Trigger.Request())
            else:
                self.get_logger().warn(
                    'start_replan service not available -- traj_exe may not be running.')
            return

        wait_s = (self.get_clock().now().nanoseconds - self._mode_cmd_sent_at) / 1e9
        if wait_s > self.offboard_confirm_timeout_s and not self._warned_confirm_slow:
            self.get_logger().warn(
                'DO_SET_MODE sent %.1fs ago but VehicleStatus still reports nav_state=%s, '
                'not OFFBOARD (%d) -- check the RC switch/QGC mode selector. Still watching.'
                % (wait_s, str(self._nav_state), VehicleStatus.NAVIGATION_STATE_OFFBOARD))
            self._warned_confirm_slow = True

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
        self._mode_cmd_sent_at = None
        self._replan_started = False
        self._warned_confirm_slow = False
        response.success = True
        response.message = (
            'Streaming started, requesting OFFBOARD in %.2fs.' % self.offboard_prime_s)
        self.get_logger().info(response.message)
        return response

    def _on_disable_offboard(self, request, response):
        self._streaming = False
        self._streaming_since = None
        self._mode_cmd_sent = False
        self._mode_cmd_sent_at = None
        was_replanning = self._replan_started
        self._replan_started = False
        self._warned_confirm_slow = False
        if was_replanning:
            if self.stop_replan_client.service_is_ready():
                self.stop_replan_client.call_async(Trigger.Request())
            else:
                self.get_logger().warn(
                    'stop_replan service not available -- traj_exe may not be running.')
        response.success = True
        response.message = 'Streaming stopped.' + (' Replanning stopped too.' if was_replanning else '')
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
