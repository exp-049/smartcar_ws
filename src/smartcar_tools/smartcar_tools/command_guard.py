"""Apply a command lease and normalize body velocity into wheel commands."""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        input_topic = self.declare_parameter('input_topic', '/car/cmd_vel_checked').value
        output_topic = self.declare_parameter('output_topic', '/car/chassis/cmd_vel').value
        direct_input_topic = self.declare_parameter('direct_input_topic', '').value
        # Lease: a command keeps being republished until this long after the last
        # message arrived. Nav2 publishes at 20 Hz, so 5 s tolerates long stalls.
        self.timeout = self.declare_parameter('timeout_sec', 5.0).value
        self.max_linear = self.declare_parameter('max_linear_velocity', 0.20).value
        self.max_angular = self.declare_parameter('max_angular_velocity', 1.20).value
        self.normalized_output = self.declare_parameter('normalized_output', False).value
        self.invert_linear = self.declare_parameter('invert_linear', False).value
        self.invert_angular = self.declare_parameter('invert_angular', False).value
        self.speed_per_pwm = self.declare_parameter('speed_per_pwm', 0.35).value
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.133).value
        # The real motor has a static-friction zone. Ignore tiny wheel requests
        # until they cross a start threshold, keep an active wheel until it falls
        # below a lower stop threshold, then apply a small PWM floor.
        self.minimum_normalized_pwm = self.declare_parameter(
            'minimum_normalized_pwm', 0.0).value
        self.normalized_pwm_start_threshold = self.declare_parameter(
            'normalized_pwm_start_threshold', 0.0).value
        self.normalized_pwm_stop_threshold = self.declare_parameter(
            'normalized_pwm_stop_threshold', 0.0).value
        self.max_normalized_pwm_slew_rate = self.declare_parameter(
            'max_normalized_pwm_slew_rate', 0.0).value
        if any(not math.isfinite(item) or item <= 0 for item in
               (self.timeout, self.max_linear, self.max_angular,
                self.speed_per_pwm, self.wheel_separation)):
            raise ValueError('Timeout, velocity limits, speed_per_pwm and wheel separation must be positive')
        if not (0.0 <= self.normalized_pwm_stop_threshold <=
                self.normalized_pwm_start_threshold < 1.0):
            raise ValueError('Normalized PWM thresholds must satisfy 0 <= stop <= start < 1')
        if not (self.normalized_pwm_start_threshold <=
                self.minimum_normalized_pwm <= 1.0):
            raise ValueError('minimum_normalized_pwm must be between start threshold and 1')
        if not math.isfinite(self.max_normalized_pwm_slew_rate) or self.max_normalized_pwm_slew_rate < 0:
            raise ValueError('max_normalized_pwm_slew_rate must be finite and non-negative')
        self.publisher = self.create_publisher(Twist, output_topic, QoSProfile(depth=1))
        self.create_subscription(Twist, input_topic, self.command, QoSProfile(depth=1))
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.last = None
        self.last_at = 0.0
        self.last_direct = None
        self.last_direct_at = 0.0
        self.wheel_active = {False: [False, False], True: [False, False]}
        self.wheel_direction = {False: [0, 0], True: [0, 0]}
        self.output_wheels = [0.0, 0.0]
        self.last_output_at = time.monotonic()
        self.next_diagnostic = 0.0
        self.last_status = None
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.tick, clock=self.safety_clock)
        if direct_input_topic:
            if direct_input_topic == output_topic:
                raise ValueError('direct_input_topic must differ from output_topic')
            self.create_subscription(
                Twist, direct_input_topic,
                lambda msg: self.command(msg, direct=True), QoSProfile(depth=1))

    def command(self, msg, direct=False):
        # Reject non-finite input before converting it to wheel commands.
        if not all(math.isfinite(getattr(vector, axis))
                   for vector in (msg.linear, msg.angular) for axis in ('x', 'y', 'z')):
            self.get_logger().warning('Rejected non-finite velocity command')
            return
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        if self.invert_linear:
            linear = -linear
        if self.invert_angular:
            angular = -angular
        result = Twist()
        if self.normalized_output:
            left = linear - angular * self.wheel_separation / 2.0
            right = linear + angular * self.wheel_separation / 2.0
            scale = max(1.0, abs(left) / self.speed_per_pwm,
                        abs(right) / self.speed_per_pwm)
            # Internal chassis Twist: linear.x is normalized left PWM and
            # angular.z is normalized right PWM.
            wheels = (left / (self.speed_per_pwm * scale),
                      right / (self.speed_per_pwm * scale))
            active = self.wheel_active[direct]
            directions = self.wheel_direction[direct]
            shaped = []
            for index, value in enumerate(wheels):
                direction = (value > 0.0) - (value < 0.0)
                magnitude = abs(value)
                if direction == 0:
                    active[index] = False
                    directions[index] = 0
                    shaped.append(0.0)
                    continue
                if direction != directions[index]:
                    active[index] = False
                    directions[index] = direction
                threshold = (self.normalized_pwm_stop_threshold if active[index]
                             else self.normalized_pwm_start_threshold)
                if magnitude <= threshold:
                    active[index] = False
                    shaped.append(0.0)
                    continue
                active[index] = True
                shaped.append(direction * max(magnitude, self.minimum_normalized_pwm))
            result.linear.x, result.angular.z = shaped
        else:
            # Gazebo consumes physical body velocity rather than wheel PWM.
            result.linear.x = linear
            result.angular.z = angular
        now = time.monotonic()
        if direct:
            self.last_direct = result
            self.last_direct_at = now
        else:
            self.last = result
            self.last_at = now

    def tick(self):
        target = Twist()
        now = time.monotonic()
        direct_age = now - self.last_direct_at
        checked_age = now - self.last_at
        if self.last_direct is not None and direct_age <= self.timeout:
            target = self.last_direct
            age = direct_age
            source = 'direct body command'
        elif self.last is not None and checked_age <= self.timeout:
            target = self.last
            age = checked_age
            source = 'checked command'
        else:
            age = min(direct_age, checked_age)
            source = 'idle'
        out = target
        if self.normalized_output:
            desired = [target.linear.x, target.angular.z]
            if source == 'idle' or all(abs(value) <= 1e-9 for value in desired):
                # Safety stops and expired leases bypass the output slew limit.
                self.output_wheels = [0.0, 0.0]
            else:
                elapsed = max(0.0, min(0.1, now - self.last_output_at))
                max_delta = self.max_normalized_pwm_slew_rate * elapsed
                for index, value in enumerate(desired):
                    if abs(value) <= 1e-9:
                        self.output_wheels[index] = 0.0
                    elif self.max_normalized_pwm_slew_rate > 0.0:
                        difference = value - self.output_wheels[index]
                        self.output_wheels[index] += max(-max_delta, min(max_delta, difference))
                    else:
                        self.output_wheels[index] = value
            self.last_output_at = now
            out = Twist()
            out.linear.x, out.angular.z = self.output_wheels
        self.publisher.publish(out)
        now = time.monotonic()
        status_text = ('Forwarding %s (lease %.1f/%.1f s)' % (source, age, self.timeout)
                       if source != 'idle' else 'Idle (no active command)')
        if now >= self.next_diagnostic or status_text != self.last_status:
            status = DiagnosticStatus()
            status.name = 'smartcar/command_guard'
            status.hardware_id = 'smartcar'
            status.level = DiagnosticStatus.OK
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
