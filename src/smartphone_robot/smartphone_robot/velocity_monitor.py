#!/usr/bin/env python3
"""Subscribe to /cmd_vel and report what the robot is being told to do.

This is a diagnostic node. `ros2 topic echo /cmd_vel` already dumps raw messages,
so why write this? Because echo shows six floats and nothing else. This node
interprets them: it reports the commanded speed in human terms, detects a stop,
and -- importantly -- notices when commands stop arriving at all.

That last part matters on a real robot. A base that stops receiving /cmd_vel is
usually the symptom of a crashed controller or a dropped WiFi link, and it is far
more dangerous than a wrong command, because most drivers keep executing the last
velocity they were given. Watchdog nodes like this one are standard on hardware.
"""

import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist


class VelocityMonitor(Node):
    """Interprets the /cmd_vel stream and warns when it goes silent."""

    # Below this, a float is treated as "commanded to stop" rather than "crawling".
    ZERO_TOLERANCE = 1e-6

    def __init__(self):
        super().__init__('velocity_monitor')

        self.declare_parameter('timeout_sec', 1.0)
        self.timeout_sec = self.get_parameter('timeout_sec').value

        # --- Subscription -----------------------------------------------------
        # create_subscription(msg_type, topic_name, callback, qos_depth)
        #
        # The message type and the QoS depth must be compatible with the
        # publisher's, or DDS will silently refuse to connect the two -- the nodes
        # appear in `ros2 node list` but no data ever flows. That silence is the
        # single most common source of "why isn't it working?" in ROS 2.
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.on_cmd_vel, 10
        )

        self.msg_count = 0
        self.last_msg_time = None
        self.last_description = None
        self.warned_about_silence = False

        # Independent timer that checks for silence. It runs whether or not any
        # message arrives -- which is the whole point of a watchdog.
        self.watchdog = self.create_timer(0.5, self.check_for_silence)

        self.get_logger().info(
            f'velocity_monitor listening on /cmd_vel '
            f'(warns after {self.timeout_sec}s of silence)'
        )

    def on_cmd_vel(self, msg: Twist):
        """Called by rclpy every time a Twist arrives. `msg` is already deserialized."""
        self.msg_count += 1
        self.last_msg_time = self.get_clock().now()

        if self.warned_about_silence:
            self.get_logger().info('/cmd_vel is flowing again.')
            self.warned_about_silence = False

        forward = msg.linear.x
        turn = msg.angular.z

        if abs(forward) < self.ZERO_TOLERANCE and abs(turn) < self.ZERO_TOLERANCE:
            description = 'STOP'
        else:
            parts = []
            if abs(forward) >= self.ZERO_TOLERANCE:
                parts.append(f'{"forward" if forward > 0 else "backward"} {abs(forward):.2f} m/s')
            if abs(turn) >= self.ZERO_TOLERANCE:
                parts.append(f'turning {"left" if turn > 0 else "right"} '
                             f'{abs(math.degrees(turn)):.0f} deg/s')
            description = ', '.join(parts)

        # Throttling keeps a 10 Hz stream from flooding the console, but throttling
        # *everything* is wrong: a state CHANGE is the one message that must never be
        # dropped. Suppressing "STOP" because a routine line was logged 0.2 s earlier
        # is exactly the kind of gap that hides a fault on real hardware.
        #
        # So: log a change immediately, and throttle only the repetition of a state
        # we have already reported.
        if description != self.last_description:
            self.get_logger().info(f'[#{self.msg_count}] {description}')
            self.last_description = description
        else:
            self.get_logger().info(
                f'[#{self.msg_count}] {description}',
                throttle_duration_sec=1.0,
            )

    def check_for_silence(self):
        """Warn if /cmd_vel has gone quiet -- a stalled publisher or a lost link."""
        if self.last_msg_time is None or self.warned_about_silence:
            return

        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            self.get_logger().warn(
                f'No /cmd_vel for {elapsed:.1f}s. On real hardware the base would '
                f'still be executing the last command.'
            )
            self.warned_about_silence = True


def main(args=None):
    # Same reason as in velocity_publisher: rclpy's default SIGINT handler shuts the
    # context down before spin() returns, so the closing summary below would fail
    # with "Failed to publish log message to rosout: publisher's context is invalid".
    # Handling the signal ourselves keeps the context alive until we are done.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityMonitor()

    shutdown_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.get_logger().info(f'Shutting down after {node.msg_count} messages.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
