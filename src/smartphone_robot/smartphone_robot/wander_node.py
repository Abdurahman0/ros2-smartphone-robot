#!/usr/bin/env python3
"""Reactive obstacle avoidance: drive forward, turn away from whatever is close.

PURPOSE
=======
SLAM needs the robot to move around in order to see the room. Driving it by hand is
tedious and not reproducible, so this node explores on its own.

It is deliberately REACTIVE, not planned: there is no map, no path, no memory of where
it has been. Every decision comes from the newest laser scan alone. That makes it the
simplest thing that can possibly work, and a useful contrast with Navigation2 later:
Nav2 builds a costmap, plans a global route and follows it, which is why Nav2 can reach
a specific goal and this node cannot. This one only guarantees "keep moving, do not hit
anything".

Architecture note: this node never reads a sensor directly. It subscribes to /scan.
Whether those ranges come from Gazebo's simulated GPU LiDAR or from a real RPLidar
driver is invisible here, and the code would not change.
"""

import math

from rclpy.node import Node
from smartphone_robot.node_utils import run_node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.qos import DurabilityPolicy


class WanderNode(Node):
    """Drives forward and steers away from obstacles using /scan."""

    def __init__(self):
        super().__init__('wander_node')

        self.declare_parameter('forward_speed', 0.22)     # [m/s]
        self.declare_parameter('turn_speed', 0.7)         # [rad/s]
        self.declare_parameter('stop_distance', 0.55)     # [m] start avoiding
        self.declare_parameter('clear_distance', 0.85)    # [m] resume forward
        self.declare_parameter('front_angle', 0.6)        # [rad] half-width of "front"
        self.declare_parameter('side_angle', 1.2)         # [rad] half-width of each side
        self.declare_parameter('control_rate', 10.0)      # [Hz]
        # Auto-stop when the map stops growing. Exploration is "done" when the number of
        # known map cells barely increases over a window of time -- the robot has seen
        # essentially all it can reach. After that it stops driving and hands /cmd_vel
        # over to whatever else wants it (e.g. keyboard teleop).
        self.declare_parameter('auto_stop', True)
        self.declare_parameter('done_growth_frac', 0.01)  # <1% growth over the window
        self.declare_parameter('done_window_sec', 12.0)   # window to measure growth
        self.declare_parameter('min_map_cells', 2000)     # don't declare done too early

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.clear_distance = self.get_parameter('clear_distance').value
        self.front_angle = self.get_parameter('front_angle').value
        self.side_angle = self.get_parameter('side_angle').value
        rate = self.get_parameter('control_rate').value

        # Sensor data uses BEST_EFFORT: a dropped laser scan should be skipped, not
        # retransmitted, because by the time it arrives it describes the past. Most
        # ROS 2 sensor drivers publish with this profile, so a subscriber using the
        # default RELIABLE profile silently receives nothing -- the single most common
        # "my node sees no data" bug in ROS 2.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.on_scan, sensor_qos
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.latest_scan = None
        self.turn_direction = 0.0   # 0 = driving straight, +1 left, -1 right

        # --- map-completion tracking ---
        self.auto_stop = self.get_parameter('auto_stop').value
        self.done_growth = self.get_parameter('done_growth_frac').value
        self.done_window = self.get_parameter('done_window_sec').value
        self.min_cells = self.get_parameter('min_map_cells').value
        self.done = False
        self.known_cells = 0
        self.window_start_cells = 0
        self.window_start_time = None

        # The map is latched (TRANSIENT_LOCAL), so match that or the last map never arrives.
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        # Announce completion so a launch or another node can react to it.
        self.done_pub = self.create_publisher(Bool, 'exploration_done', 10)

        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'wander_node started: forward={self.forward_speed} m/s, '
            f'stop<{self.stop_distance} m, clear>{self.clear_distance} m'
        )

    # -------------------------------------------------------------------- helpers

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_map(self, msg: OccupancyGrid):
        """Count known cells and decide when exploration has plateaued."""
        if self.done or not self.auto_stop:
            return
        # Known = anything that is not -1 (unknown): free or occupied.
        self.known_cells = sum(1 for v in msg.data if v >= 0)

        now = self.get_clock().now().nanoseconds / 1e9
        if self.window_start_time is None:
            self.window_start_time = now
            self.window_start_cells = self.known_cells
            return

        if now - self.window_start_time < self.done_window:
            return

        # A full window has passed: how much did the known area grow?
        base = max(self.window_start_cells, 1)
        growth = (self.known_cells - self.window_start_cells) / base
        if self.known_cells >= self.min_cells and growth < self.done_growth:
            self.done = True
            self.cmd_pub.publish(Twist())   # stop
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info(
                '\n'
                '======================================================\n'
                f' MAPPING COMPLETE ({self.known_cells} cells mapped).\n'
                ' The robot has stopped. You can now DRIVE IT with the\n'
                ' arrow keys in the teleop window, then save the map.\n'
                '======================================================'
            )
        else:
            # Slide the window forward for the next measurement.
            self.window_start_time = now
            self.window_start_cells = self.known_cells

    def _sector_min(self, scan: LaserScan, centre: float, half_width: float) -> float:
        """Smallest valid range within +/- half_width radians of `centre`.

        Index arithmetic rather than a hard-coded slice: the array layout depends on
        angle_min and angle_increment, which differ between LiDAR models. Computing
        indices from angles is what makes this node work unchanged on real hardware.
        """
        if scan.angle_increment == 0.0:
            return math.inf

        lo_angle = centre - half_width
        hi_angle = centre + half_width
        lo = int(round((lo_angle - scan.angle_min) / scan.angle_increment))
        hi = int(round((hi_angle - scan.angle_min) / scan.angle_increment))
        lo = max(0, min(len(scan.ranges) - 1, lo))
        hi = max(0, min(len(scan.ranges) - 1, hi))

        best = math.inf
        for r in scan.ranges[lo:hi + 1]:
            # inf means "nothing within range" and NaN means "bad reading". Both are
            # normal in real laser data and must be filtered, or min() returns NaN and
            # every comparison downstream silently becomes False.
            if r != r or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            best = min(best, r)
        return best

    # ------------------------------------------------------------------ control

    def on_timer(self):
        """One control step. Runs at a fixed rate regardless of scan timing."""
        if self.latest_scan is None:
            return

        # Once mapping is done, stop driving and stay silent so another publisher
        # (keyboard teleop) can own /cmd_vel without a fight.
        if self.done:
            return

        scan = self.latest_scan
        front = self._sector_min(scan, 0.0, self.front_angle)
        left = self._sector_min(scan, self.side_angle, self.front_angle)
        right = self._sector_min(scan, -self.side_angle, self.front_angle)

        cmd = Twist()

        if self.turn_direction == 0.0:
            # Driving straight until something gets too close.
            if front < self.stop_distance:
                # Turn towards whichever side has more room. Choosing by comparison
                # rather than always turning left stops the robot from getting stuck
                # in a corner, ping-ponging between two walls.
                self.turn_direction = 1.0 if left > right else -1.0
                self.get_logger().info(
                    f'Obstacle at {front:.2f} m -> turning '
                    f'{"left" if self.turn_direction > 0 else "right"} '
                    f'(left {left:.2f} m, right {right:.2f} m)'
                )
            else:
                cmd.linear.x = self.forward_speed
        else:
            # Turning in place until the way ahead is clear again. The gap between
            # stop_distance and clear_distance is HYSTERESIS: without it the robot
            # would oscillate on and off the threshold and crawl forward in jerks.
            if front > self.clear_distance:
                self.get_logger().info(f'Clear at {front:.2f} m -> driving on')
                self.turn_direction = 0.0
                cmd.linear.x = self.forward_speed
            else:
                cmd.angular.z = self.turn_direction * self.turn_speed

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())



def _wander_shutdown(node):
    node.stop_robot()
    node.get_logger().info('wander_node stopped; zero velocity sent.')


def main(args=None):
    run_node(WanderNode, on_shutdown=_wander_shutdown, args=args)


if __name__ == '__main__':
    main()
