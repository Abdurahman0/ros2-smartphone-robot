#!/usr/bin/env python3
"""Action server that drives the robot a requested distance on /cmd_vel.

WHY AN ACTION AND NOT A SERVICE
-------------------------------
Driving two metres takes seconds. A service call would block the caller for the whole
move, could not report progress, and could not be cancelled once started. An action
solves all three: the goal is accepted immediately, feedback streams while the robot
moves, and the client can cancel mid-motion. Navigation2's "go to this pose" is an
action for exactly these reasons.

An action is not a new transport -- underneath it is built from topics and services
(goal service, cancel service, result service, feedback topic, status topic). Running
`ros2 topic list` while this node is up shows them.

HOW DISTANCE IS MEASURED (and the honest limitation)
----------------------------------------------------
There is no odometry yet, so travelled distance is estimated OPEN-LOOP as
speed x elapsed_time. That is dead reckoning: it assumes the wheels turn exactly as
commanded, no slip, no acceleration ramp. On a real robot that assumption is wrong
within a metre or two.

Once Gazebo publishes /odom (next stage), this node should subscribe to it and measure
real displacement instead. The action *interface* does not change when that happens --
only this file does. That is the whole point of separating interface from
implementation.
"""

import math
import signal
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

from smartphone_robot_interfaces.action import DriveDistance


class DriveDistanceServer(Node):
    """Executes DriveDistance goals by publishing a timed velocity command."""

    def __init__(self):
        super().__init__('drive_distance_server')

        self.declare_parameter('max_speed', 0.5)       # [m/s] safety clamp
        self.declare_parameter('feedback_rate', 5.0)   # [Hz] how often to report
        self.declare_parameter('cmd_rate', 20.0)       # [Hz] how often to command

        self.max_speed = self.get_parameter('max_speed').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.cmd_rate = self.get_parameter('cmd_rate').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Only one drive at a time: two goals would fight over /cmd_vel.
        self._busy = False

        # A ReentrantCallbackGroup lets the cancel request be handled WHILE
        # execute_callback is still running. With the default (mutually exclusive)
        # group and a single-threaded executor, execute_callback would block the
        # executor and cancellation could never be processed -- the robot would
        # ignore the stop request until the drive finished. Hence MultiThreadedExecutor
        # in main() as well; the two go together.
        callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            DriveDistance,
            'drive_distance',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=callback_group,
        )

        self.get_logger().info(
            f'drive_distance action server ready (max_speed={self.max_speed} m/s)'
        )

    # -------------------------------------------------------------- goal handling

    def goal_callback(self, goal_request):
        """Decide whether to accept a goal. Runs before execution starts.

        Rejecting bad goals here -- rather than failing later -- gives the client an
        immediate, explicit answer, and keeps the robot from ever being commanded
        with nonsense.
        """
        if self._busy:
            self.get_logger().warn('Goal rejected: already driving.')
            return GoalResponse.REJECT

        if goal_request.distance <= 0.0 or not math.isfinite(goal_request.distance):
            self.get_logger().warn(
                f'Goal rejected: distance={goal_request.distance} must be > 0.'
            )
            return GoalResponse.REJECT

        if goal_request.speed <= 0.0 or not math.isfinite(goal_request.speed):
            self.get_logger().warn(
                f'Goal rejected: speed={goal_request.speed} must be > 0.'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Goal accepted: {goal_request.distance:.2f} m at {goal_request.speed:.2f} m/s'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        """Always allow cancellation. A robot must always be stoppable."""
        self.get_logger().warn('Cancel requested.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Drive until the distance is reached, cancelled, or aborted."""
        self._busy = True
        try:
            return self._drive(goal_handle)
        finally:
            # Whatever happens -- success, cancel, or an unexpected exception -- the
            # robot must not be left moving and the server must not stay locked.
            self._stop_robot()
            self._busy = False

    def _drive(self, goal_handle):
        request = goal_handle.request
        target = request.distance
        # Clamp rather than reject: the caller asked for a valid move, just too fast.
        speed = min(request.speed, self.max_speed)
        if speed < request.speed:
            self.get_logger().warn(
                f'Speed clamped from {request.speed:.2f} to {speed:.2f} m/s.'
            )

        feedback = DriveDistance.Feedback()
        result = DriveDistance.Result()

        twist = Twist()
        twist.linear.x = speed

        start = self.get_clock().now()
        cmd_period = 1.0 / self.cmd_rate
        feedback_period = 1.0 / self.feedback_rate
        last_feedback = 0.0
        travelled = 0.0

        while True:
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            travelled = min(elapsed * speed, target)

            # --- cancelled by the client? ---
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.distance_travelled = travelled
                result.success = False
                result.message = f'Cancelled after {travelled:.2f} m of {target:.2f} m.'
                self.get_logger().warn(result.message)
                return result

            # --- goal no longer active (aborted, or server shutting down)? ---
            if not goal_handle.is_active:
                result.distance_travelled = travelled
                result.success = False
                result.message = 'Goal became inactive; aborting.'
                self.get_logger().warn(result.message)
                return result

            # --- finished? ---
            if travelled >= target:
                break

            self.cmd_pub.publish(twist)

            if elapsed - last_feedback >= feedback_period:
                feedback.distance_travelled = travelled
                feedback.percent_complete = 100.0 * travelled / target
                goal_handle.publish_feedback(feedback)
                last_feedback = elapsed

            # Sleeping here is safe because execute_callback runs on its own thread
            # of the MultiThreadedExecutor -- it does not stall the node.
            time.sleep(cmd_period)

        goal_handle.succeed()
        result.distance_travelled = travelled
        result.success = True
        result.message = f'Drove {travelled:.2f} m at {speed:.2f} m/s.'
        self.get_logger().info(result.message)
        return result

    def _stop_robot(self):
        """Publish zero velocity and let it flush before the node exits."""
        self.cmd_pub.publish(Twist())
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DriveDistanceServer()

    # MultiThreadedExecutor is required, not optional: with the single-threaded one,
    # a running execute_callback would block cancel requests (see ReentrantCallbackGroup
    # above).
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    shutdown_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        while rclpy.ok() and not shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node._stop_robot()
            node.get_logger().info('drive_distance_server shutting down.')
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
