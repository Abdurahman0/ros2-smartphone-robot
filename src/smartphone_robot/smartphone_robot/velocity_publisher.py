#!/usr/bin/env python3
"""Publish a velocity command stream on /cmd_vel, with runtime tuning and a gate.

/cmd_vel is the standard ROS 2 topic for commanding a mobile base. Whatever consumes
it -- the Gazebo differential-drive plugin in simulation, or a real motor driver on
hardware -- receives exactly the same geometry_msgs/msg/Twist message. That symmetry
is what lets this project move from simulation to a physical robot without rewriting
the control side.

All three ROS 2 communication patterns that matter for this project appear here, and
the contrast between them is the point:

  * a TOPIC (/cmd_vel) carries the continuous velocity stream. Fire-and-forget, no
    reply, any number of listeners.
  * a SERVICE (~/enable_motion) turns motion on and off. One caller, one reply,
    happens once. Asking "did it work?" is meaningful -- which is exactly when a
    service is right and a topic is wrong.
  * PARAMETERS (linear_x, angular_z, ...) hold configuration. They can be set from a
    YAML file at launch and changed while the node runs, with validation to reject
    unsafe values before they ever reach the wheels.
"""

import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import SetBool


class VelocityPublisher(Node):
    """Publishes a Twist on /cmd_vel; tunable at runtime, gateable via a service."""

    def __init__(self):
        # The string here is the NODE NAME shown by `ros2 node list`. It is
        # independent of the file name and of the executable name -- and it is the
        # key a parameter YAML file must match.
        super().__init__('velocity_publisher')

        # --- Parameters -------------------------------------------------------
        # Declaring parameters instead of hard-coding numbers means speeds can change
        # without touching code: from the command line, from a YAML file loaded by a
        # launch file, or live with `ros2 param set`. On a real robot this is how you
        # retune behaviour in the field without a rebuild and without a restart.
        self.declare_parameter('linear_x', 0.2)       # forward speed  [m/s]
        self.declare_parameter('angular_z', 0.5)      # turn rate      [rad/s]
        self.declare_parameter('publish_rate', 10.0)  # how often      [Hz]

        # Safety envelope. These exist because a parameter is an *input from outside
        # the program*, and outside input on a robot eventually contains a typo. A
        # stray `linear_x:=99` must be refused here, not discovered by the hardware.
        self.declare_parameter('max_linear_speed', 1.0)    # [m/s]
        self.declare_parameter('max_angular_speed', 2.0)   # [rad/s]

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.linear_x = self.get_parameter('linear_x').value
        self.angular_z = self.get_parameter('angular_z').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Register the validator BEFORE anything can call `ros2 param set`. The
        # callback runs for every attempted change and can veto it, so an invalid
        # value never becomes the node's state.
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # --- Publisher --------------------------------------------------------
        # create_publisher(msg_type, topic_name, qos_depth)
        #
        # 'cmd_vel' is written WITHOUT a leading slash on purpose. That makes it a
        # relative name, so launching this node inside a namespace automatically
        # remaps it (e.g. namespace 'robot1' -> /robot1/cmd_vel). Hard-coding
        # '/cmd_vel' would defeat that and break any multi-robot setup.
        #
        # The 10 is the queue depth: if a subscriber is slower than us, keep at most
        # the 10 newest messages and drop older ones.
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- Service ----------------------------------------------------------
        # '~/enable_motion' uses the '~' prefix, meaning "private to this node". It
        # resolves to /velocity_publisher/enable_motion. Private names keep a node's
        # own controls from colliding with another node's, which matters once a robot
        # runs thirty nodes.
        #
        # std_srvs/srv/SetBool is a ready-made request/reply definition:
        #     bool data          <- request
        #     ---
        #     bool success       <- response
        #     string message
        # Using a standard type means no custom interface package is needed and any
        # generic tool can call it.
        self.motion_enabled = True
        self.enable_service = self.create_service(
            SetBool, '~/enable_motion', self.on_enable_motion
        )

        # --- Timer ------------------------------------------------------------
        # A ROS timer, not a `while True: sleep()` loop. rclpy runs it on the node's
        # executor, so the node stays responsive to service calls, parameter changes
        # and shutdown requests while it publishes.
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        # get_logger() goes to the console AND to the /rosout topic, so logs from
        # every node on the robot can be recorded and inspected centrally.
        self.get_logger().info(
            f'velocity_publisher started: linear.x={self.linear_x} m/s, '
            f'angular.z={self.angular_z} rad/s, rate={self.publish_rate} Hz '
            f'(limits: {self.max_linear} m/s, {self.max_angular} rad/s)'
        )

    # ------------------------------------------------------------------ callbacks

    def on_timer(self):
        """Build one Twist and publish it. Called automatically by the timer."""
        msg = Twist()

        # A Twist carries 6 numbers (linear x/y/z + angular x/y/z), but a
        # differential-drive robot can only do two things: drive along its own
        # forward axis and rotate about its vertical axis. Everything else stays zero
        # because the robot physically cannot do it -- it has no sideways wheels and
        # it does not fly.
        if self.motion_enabled:
            msg.linear.x = self.linear_x
            msg.angular.z = self.angular_z

        # When motion is disabled we keep publishing -- an all-zero Twist. Going
        # silent instead would be wrong: most bases hold the last command they
        # received, so silence means "keep driving", not "stop".
        self.publisher.publish(msg)

    def on_enable_motion(self, request, response):
        """Service callback: enable or disable motion.

        A service callback receives a request and MUST return the response object it
        was handed. It runs on the node's executor, so it has to be quick -- long
        work belongs in an action, not a service.
        """
        self.motion_enabled = request.data

        if not request.data:
            # Do not wait for the next timer tick to stop; send zero immediately.
            self.publisher.publish(Twist())

        response.success = True
        response.message = (
            'Motion enabled.' if request.data
            else 'Motion disabled; zero velocity sent.'
        )
        self.get_logger().info(response.message)
        return response

    def on_parameter_change(self, params):
        """Validate and apply parameter changes made while the node is running.

        Returning successful=False REJECTS the change: the node keeps its old value
        and `ros2 param set` reports the failure. This is the right place for safety
        limits, because it is the one path every external change must pass through.
        """
        # Two passes on purpose: validate everything first, then apply. A partially
        # applied change (new speed, rejected rate) would leave the node in a state
        # no one asked for.
        for p in params:
            if p.name == 'linear_x' and abs(p.value) > self.max_linear:
                return SetParametersResult(
                    successful=False,
                    reason=f'linear_x={p.value} exceeds max_linear_speed='
                           f'{self.max_linear} m/s',
                )
            if p.name == 'angular_z' and abs(p.value) > self.max_angular:
                return SetParametersResult(
                    successful=False,
                    reason=f'angular_z={p.value} exceeds max_angular_speed='
                           f'{self.max_angular} rad/s',
                )
            if p.name == 'publish_rate' and not 0.0 < p.value <= 200.0:
                return SetParametersResult(
                    successful=False,
                    reason=f'publish_rate={p.value} must be in (0, 200] Hz',
                )

        for p in params:
            if p.name == 'linear_x':
                self.linear_x = p.value
            elif p.name == 'angular_z':
                self.angular_z = p.value
            elif p.name == 'publish_rate':
                self.publish_rate = p.value
                # A timer's period is fixed at creation, so changing the rate means
                # replacing the timer. Destroying the old one first avoids ending up
                # with two timers publishing at different rates.
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)
            elif p.name == 'max_linear_speed':
                self.max_linear = p.value
            elif p.name == 'max_angular_speed':
                self.max_angular = p.value

        self.get_logger().info(
            f'Parameters updated: linear.x={self.linear_x} m/s, '
            f'angular.z={self.angular_z} rad/s, rate={self.publish_rate} Hz'
        )
        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------------- misc

    def stop_robot(self):
        """Command zero velocity and give the middleware a moment to send it.

        Must be called while the ROS context is still valid -- see main().
        """
        self.publisher.publish(Twist())
        # publish() only hands the message to DDS; it does not wait for it to leave.
        # Tearing the node down immediately can drop it, so allow a short flush
        # window. On real hardware this is the difference between the base stopping
        # and the base continuing at 0.2 m/s.
        time.sleep(0.2)
        self.get_logger().info('Stopped: published zero velocity.')


def main(args=None):
    """Entry point registered in setup.py as `velocity_publisher`."""
    # SAFETY: a velocity command is a *latched intent*, not a one-off pulse. A real
    # base keeps driving at the last commanded speed until told otherwise, so a node
    # that dies mid-motion leaves the robot running into a wall. We therefore must
    # publish a zero Twist on the way out.
    #
    # rclpy's default SIGINT handler tears the context down BEFORE spin() returns,
    # which makes that final publish fail with "publisher's context is invalid". So
    # we take the signal over ourselves: SignalHandlerOptions.NO stops rclpy from
    # installing its handler, and our handler only sets a flag. The context stays
    # valid until we choose to shut it down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityPublisher()

    shutdown_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)    # Ctrl-C
    signal.signal(signal.SIGTERM, request_shutdown)   # docker stop / ros2 launch

    try:
        # spin_once() in a loop instead of spin(), so the flag is checked regularly.
        # timeout_sec bounds how long we block waiting for work.
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        # try_shutdown() is a no-op if the context is already down.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
