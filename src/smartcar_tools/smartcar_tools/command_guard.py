"""Single public command output with health gating and steady-clock timeout."""
import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatusArray, GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from smartcar_tools.system_health import SystemHealth


class CommandGuard(SystemHealth):
    def __init__(self):
        super().__init__('command_guard')
        input_topic = self.declare_parameter('input_topic', '/car/cmd_vel_checked').value
        output_topic = self.declare_parameter('output_topic', '/car/chassis/cmd_vel').value
        self.timeout = self.declare_parameter('timeout_sec', 0.5).value
        self.max_linear = self.declare_parameter('max_linear_velocity', 0.20).value
        self.max_angular = self.declare_parameter('max_angular_velocity', 1.20).value
        if any(not math.isfinite(item) or item <= 0 for item in (self.timeout, self.max_linear, self.max_angular)):
            raise ValueError('Command timeout and velocity limits must be finite and positive')
        self.publisher = self.create_publisher(Twist, output_topic, QoSProfile(depth=1))
        self.create_subscription(Twist, input_topic, self.command, QoSProfile(depth=1))
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.goal_status = None
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_goal_status, 10)
        self.create_service(Trigger, '/car/resume_commands', self.resume)
        self.last = None
        self.last_at = 0.0
        self.seen_healthy = False
        self.latched = False
        self.next_diagnostic = 0.0
        self.last_status = None
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.tick, clock=self.safety_clock)

    def on_goal_status(self, msg):
        self.goal_status = msg.status_list

    def command(self, msg):
        if not all(math.isfinite(getattr(vector, axis))
                   for vector in (msg.linear, msg.angular) for axis in ('x', 'y', 'z')):
            self.last = None
            self.get_logger().warning('Rejected non-finite velocity command')
            return
        if self.latched or self.problems():
            self.last = None
            return
        self.last = Twist()
        self.last.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        self.last.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self.last_at = time.monotonic()

    def resume(self, request, response):
        problems = self.problems()
        odom = self.samples.get('odom')
        stopped = odom and abs(odom.twist.twist.linear.x) < 0.01 and abs(odom.twist.twist.angular.z) < 0.02
        active_goal = self.goal_status is None or any(
            goal.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                            GoalStatus.STATUS_CANCELING) for goal in self.goal_status)
        response.success = not problems and bool(stopped) and not active_goal
        response.message = ('Command output rearmed; submit a new task/command' if response.success
                            else 'Cannot rearm: ' + ('; '.join(problems) or
                                 'chassis is moving' if not stopped else
                                 'goal status unavailable or an action is still active'))
        if response.success:
            self.latched = False
            self.last = None
        return response

    def tick(self):
        problems = self.problems()
        if problems and self.seen_healthy and not self.latched:
            self.latched = True
            self.last = None
            if self.cancel.service_is_ready():
                self.cancel.call_async(CancelGoal.Request())
            self.get_logger().error('Command output stopped: ' + '; '.join(problems))
        if not problems:
            self.seen_healthy = True
        out = Twist()
        if not problems and not self.latched and self.last is not None and time.monotonic() - self.last_at <= self.timeout:
            out = self.last
        self.publisher.publish(out)
        now = time.monotonic()
        status_text = '; '.join(problems) or ('Latched; call /car/resume_commands when stopped' if self.latched else 'Ready')
        if now >= self.next_diagnostic or status_text != self.last_status:
            status = DiagnosticStatus()
            status.name = 'smartcar/command_guard'
            status.hardware_id = 'smartcar'
            status.level = DiagnosticStatus.ERROR if problems or self.latched else DiagnosticStatus.OK
            status.message = status_text
            diagnostic = DiagnosticArray()
            diagnostic.header.stamp = self.get_clock().now().to_msg()
            diagnostic.status = [status]
            self.diagnostics.publish(diagnostic)
            self.next_diagnostic = now + 1.0
            self.last_status = status_text


def main():
    # Keep the context valid while handling SIGINT so the final zero command can
    # be published before node teardown. rclpy's default handler shuts the
    # context down before KeyboardInterrupt reaches this finally block.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
