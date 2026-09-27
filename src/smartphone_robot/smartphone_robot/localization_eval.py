#!/usr/bin/env python3
"""Measure how wrong the robot's belief about its own position is.

WHY THIS NODE EXISTS
====================
Everything else in this project produces a demonstration: the robot maps a room, the
robot drives to a goal. A demonstration answers "does it work?" with a yes or a no, and
a yes tells you almost nothing -- it certainly does not tell you whether a change made
things better or worse.

This node produces a measurement instead. It compares, continuously:

    what AMCL BELIEVES   (/amcl_pose, estimated from laser scans and the map)
    where the robot IS   (/ground_truth/odom, straight from the physics engine)

The difference is the localisation error, in metres and degrees. That number is what
makes statements like "raising max_beams from 60 to 180 cut the mean error from 7.9 cm
to 0.9 cm" possible, instead of "it looks better now".

Ground truth is the one thing simulation gives that hardware does not. On a real robot
you would need a motion-capture rig to know the true pose. Which is also the honest
limitation of any result produced here: it measures the estimator against a *perfect*
world model, and real sensors are messier than the simulated ones feeding it.

FRAMES, AND WHY AN OFFSET IS NEEDED
===================================
The two sources do not share an origin:

    ground truth  is in Gazebo's WORLD frame, where the robot was spawned at (-3, -3)
    AMCL          is in the MAP frame, whose origin slam_toolbox anchored at the
                  robot's pose when mapping began -- that same spawn point

So map (0,0) and world (-3,-3) are the same physical place, and the truth must be
shifted before the two can be compared. That shift is the `map_origin_in_world`
parameter. Get it wrong and every error reported here is offset by a constant, which is
easy to spot: the error never approaches zero even when the robot has just started.
"""

import csv
import math
import signal
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def yaw_of(q) -> float:
    """Extract yaw from a quaternion. Only yaw matters for a robot on a flat floor."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    """Fold an angle into [-pi, pi], so an error of 359 deg reads as -1 deg."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationEval(Node):
    """Logs AMCL error against ground truth, and reports statistics on shutdown."""

    def __init__(self):
        super().__init__('localization_eval')

        # Where the map frame's origin sits in Gazebo's world frame. Defaults match the
        # spawn pose used by navigation.launch.py.
        self.declare_parameter('map_origin_in_world', [-3.0, -3.0, 0.0])  # x, y, yaw
        self.declare_parameter('sample_rate', 5.0)      # [Hz]
        self.declare_parameter('output_csv', '')        # empty -> no file, console only
        self.declare_parameter('settle_time', 5.0)      # [s] ignore the first moments

        ox, oy, oyaw = self.get_parameter('map_origin_in_world').value
        self.offset = (float(ox), float(oy), float(oyaw))
        rate = self.get_parameter('sample_rate').value
        self.settle_time = self.get_parameter('settle_time').value
        csv_path = self.get_parameter('output_csv').value

        # AMCL publishes its pose with TRANSIENT_LOCAL durability, so a subscriber that
        # joins late still receives the most recent estimate instead of waiting for the
        # next one. Matching that here is required, or nothing arrives at all.
        amcl_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_amcl, amcl_qos)
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 self.on_truth, 10)

        self.amcl = None
        self.truth = None
        self.samples = []
        self.start_time = None

        self.writer = None
        self.csv_file = None
        if csv_path:
            Path(csv_path).parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(csv_path, 'w', newline='')
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow([
                't_sec', 'truth_x', 'truth_y', 'truth_yaw_deg',
                'est_x', 'est_y', 'est_yaw_deg',
                'err_position_m', 'err_yaw_deg', 'amcl_std_x_m',
            ])

        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'localization_eval: map origin at world {self.offset}, '
            f'sampling at {rate} Hz'
            + (f', writing {csv_path}' if csv_path else '')
        )

    # ------------------------------------------------------------------ callbacks

    def on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        # covariance[0] is the variance in x; its square root is a standard deviation in
        # metres, i.e. how confident the filter claims to be. Worth recording alongside
        # the true error: a filter that is confidently wrong is far more dangerous than
        # one that admits uncertainty.
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation),
                     math.sqrt(max(msg.pose.covariance[0], 0.0)))

    def on_truth(self, msg: Odometry):
        p = msg.pose.pose
        ox, oy, oyaw = self.offset
        # World -> map: undo the map origin's translation, then its rotation.
        dx, dy = p.position.x - ox, p.position.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        self.truth = (dx * c - dy * s, dx * s + dy * c,
                      wrap(yaw_of(p.orientation) - oyaw))

    def on_timer(self):
        if self.amcl is None or self.truth is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time
        # Skip the first seconds: AMCL's particle cloud has not converged yet, and
        # including that transient would flatter or damn the result depending only on
        # how long the recording happened to run.
        if t < self.settle_time:
            return

        tx, ty, tyaw = self.truth
        ex, ey, eyaw, std = self.amcl
        err_pos = math.hypot(ex - tx, ey - ty)
        err_yaw = abs(math.degrees(wrap(eyaw - tyaw)))

        self.samples.append((t, err_pos, err_yaw, std))
        if self.writer:
            self.writer.writerow([
                f'{t:.3f}', f'{tx:.4f}', f'{ty:.4f}', f'{math.degrees(tyaw):.2f}',
                f'{ex:.4f}', f'{ey:.4f}', f'{math.degrees(eyaw):.2f}',
                f'{err_pos:.4f}', f'{err_yaw:.2f}', f'{std:.4f}',
            ])

        if len(self.samples) % 50 == 0:
            self.get_logger().info(
                f'n={len(self.samples)}  position error {err_pos*100:.1f} cm, '
                f'heading error {err_yaw:.1f} deg'
            )

    # -------------------------------------------------------------------- summary

    def report(self):
        if not self.samples:
            self.get_logger().warn('No samples collected.')
            return
        pos = sorted(s[1] for s in self.samples)
        yaw = sorted(s[2] for s in self.samples)

        def pct(values, p):
            return values[min(len(values) - 1, int(len(values) * p))]

        n = len(pos)
        self.get_logger().info(
            '\n'
            '==================== LOCALISATION ERROR ====================\n'
            f'  samples            : {n} over {self.samples[-1][0]:.0f} s\n'
            f'  position  mean     : {sum(pos)/n*100:6.1f} cm\n'
            f'            median   : {pct(pos, 0.50)*100:6.1f} cm\n'
            f'            95th pct : {pct(pos, 0.95)*100:6.1f} cm\n'
            f'            max      : {pos[-1]*100:6.1f} cm\n'
            f'  heading   mean     : {sum(yaw)/n:6.2f} deg\n'
            f'            median   : {pct(yaw, 0.50):6.2f} deg\n'
            f'            95th pct : {pct(yaw, 0.95):6.2f} deg\n'
            f'            max      : {yaw[-1]:6.2f} deg\n'
            '============================================================'
        )
        if self.csv_file:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationEval()

    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
