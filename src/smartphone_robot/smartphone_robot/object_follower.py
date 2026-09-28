#!/usr/bin/env python3
"""Detect a coloured object in the phone camera and steer the robot toward it.

WHAT THIS CLOSES
================
Until now the phone's data was published but never used. This node is the loop that was
missing: a real camera frame drives a real decision that moves the robot.

    /phone/camera/image_raw  ->  detect object  ->  /cmd_vel  ->  robot turns/drives

Hold a brightly coloured object in front of the phone and the simulated robot turns to
face it and drives toward it; take it away and the robot stops. That is VISUAL SERVOING --
closing a control loop through a camera -- and it is the same principle a real robot uses
to approach a charging dock or track a person, only with a colour blob standing in for a
learned detector.

WHY COLOUR, AND WHAT IT IS NOT
==============================
Detection here is an HSV colour threshold: convert to hue/saturation/value, keep pixels
in a chosen hue band, and take the largest connected blob. This is deliberately simple
and completely honest about being so -- it is classical computer vision, not machine
learning. It is robust, needs no training and no GPU, and runs easily at camera rate. It
also fails exactly where you would expect: poor lighting, a background the same colour as
the target, motion blur. A neural detector would be the upgrade, and swapping one in would
change only this node -- the servoing, the topics and the robot are unaffected.

HSV, NOT RGB
============
Colour thresholding is done in HSV because HUE is roughly constant as lighting changes,
while in RGB every channel moves together when a shadow falls across the object. This is
why "find the red thing" is a hue window in HSV and a fragile mess in RGB. Red is the one
awkward case: its hue wraps around 0/180, so it needs two bands, which the config handles.

SAFETY
======
This node commands motion, so it inherits the rules the rest of the project follows:
  * no object in view -> publish zero velocity, do not go silent (a base holds its last
    command, so silence would mean "keep driving")
  * a clean zero on shutdown
  * commanded speeds clamped to limits, independent of what the vision produces
It writes to /cmd_vel, so run it INSTEAD of Nav2, not alongside it -- two publishers
fighting over one base is not something to arbitrate here.
"""

import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class ObjectFollower(Node):
    """Colour-blob visual servoing from the phone camera to /cmd_vel."""

    def __init__(self):
        super().__init__('object_follower')

        # --- detection ---
        # HSV hue is 0-179 in OpenCV. Defaults target a strong blue; override for other
        # colours. For red, set hue_wrap true and use a low band near 0.
        self.declare_parameter('hue_min', 100)
        self.declare_parameter('hue_max', 130)
        self.declare_parameter('sat_min', 90)
        self.declare_parameter('val_min', 60)
        self.declare_parameter('hue_wrap', False)      # true for red (wraps 180->0)
        self.declare_parameter('min_area_frac', 0.004) # ignore blobs smaller than this
                                                       # fraction of the frame (noise)

        # --- control ---
        self.declare_parameter('angular_gain', 1.8)    # rad/s per unit of centering error
        self.declare_parameter('max_angular', 1.0)     # rad/s clamp
        self.declare_parameter('forward_speed', 0.18)  # m/s when approaching
        self.declare_parameter('stop_area_frac', 0.14) # object this big => close enough
        self.declare_parameter('center_deadband', 0.08) # don't fidget when near-centred
        self.declare_parameter('lost_timeout', 0.7)    # s of no detection before stopping
        self.declare_parameter('drive', True)          # false = detect only, no motion

        self.hue_min = self.get_parameter('hue_min').value
        self.hue_max = self.get_parameter('hue_max').value
        self.sat_min = self.get_parameter('sat_min').value
        self.val_min = self.get_parameter('val_min').value
        self.hue_wrap = self.get_parameter('hue_wrap').value
        self.min_area_frac = self.get_parameter('min_area_frac').value
        self.k_ang = self.get_parameter('angular_gain').value
        self.max_ang = self.get_parameter('max_angular').value
        self.fwd = self.get_parameter('forward_speed').value
        self.stop_area = self.get_parameter('stop_area_frac').value
        self.deadband = self.get_parameter('center_deadband').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.drive = self.get_parameter('drive').value

        self.bridge = CvBridge()

        # The camera publishes BEST_EFFORT; a subscriber defaulting to RELIABLE would
        # silently receive nothing. This mismatch is the single most common "my node sees
        # no data" bug in ROS 2, and it has to be matched deliberately.
        cam_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Image, 'phone/camera/image_raw', self.on_image, cam_qos)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.detected_pub = self.create_publisher(Bool, 'object/detected', 10)
        self.target_pub = self.create_publisher(Point, 'object/target', 10)
        # Annotated image for viewing. Republished so the detection can be seen without
        # re-running the detector in an image viewer.
        self.debug_pub = self.create_publisher(Image, 'object/debug_image', cam_qos)

        self.last_seen = None
        # A watchdog, independent of frame arrival, so "camera stopped" also stops the
        # robot rather than leaving it driving on the last command.
        self.create_timer(0.1, self.check_lost)

        mode = 'DRIVING' if self.drive else 'detect-only'
        self.get_logger().info(
            f'object_follower ({mode}): hue {self.hue_min}-{self.hue_max}, '
            f'sat>{self.sat_min}, val>{self.val_min}'
            + (' (wrapped)' if self.hue_wrap else '')
        )

    # --------------------------------------------------------------- detection

    def detect(self, bgr):
        """Return (found, cx_norm, cy_norm, area_frac, mask). cx_norm in [-1, 1]."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if self.hue_wrap:
            # Red straddles the 0/180 hue boundary, so two bands are OR-ed together.
            lo1 = np.array([0, self.sat_min, self.val_min])
            hi1 = np.array([self.hue_max, 255, 255])
            lo2 = np.array([self.hue_min, self.sat_min, self.val_min])
            hi2 = np.array([179, 255, 255])
            mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)
        else:
            lo = np.array([self.hue_min, self.sat_min, self.val_min])
            hi = np.array([self.hue_max, 255, 255])
            mask = cv2.inRange(hsv, lo, hi)

        # Morphological opening removes speckle, closing fills the blob. Without this a
        # noisy mask gives a jittering centroid and the robot twitches.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        h, w = mask.shape
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0, 0.0, mask

        biggest = max(contours, key=cv2.contourArea)
        area_frac = cv2.contourArea(biggest) / float(w * h)
        if area_frac < self.min_area_frac:
            return False, 0.0, 0.0, 0.0, mask

        m = cv2.moments(biggest)
        if m['m00'] == 0:
            return False, 0.0, 0.0, 0.0, mask
        cx = m['m10'] / m['m00']
        cy = m['m01'] / m['m00']
        # Normalise: 0 is centre, -1 is left edge, +1 is right edge.
        return True, (cx - w / 2) / (w / 2), (cy - h / 2) / (h / 2), area_frac, mask

    # ------------------------------------------------------------------ callback

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        found, cx, cy, area, mask = self.detect(bgr)

        self.detected_pub.publish(Bool(data=found))
        if found:
            self.last_seen = self.get_clock().now()
            self.target_pub.publish(Point(x=float(cx), y=float(cy), z=float(area)))
            if self.drive:
                self.servo(cx, area)

        self.publish_debug(bgr, found, cx, cy, area)

    def servo(self, cx, area):
        """Turn to centre the object; drive forward until it is close enough."""
        cmd = Twist()
        # Turn toward the object. Negative because a positive cx (object to the right)
        # needs a negative angular.z (clockwise / right turn) in REP-103.
        if abs(cx) > self.deadband:
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, -self.k_ang * cx))
        # Approach only once roughly facing it, and only until it fills enough of the
        # frame -- area is the stand-in for distance with a single camera.
        if abs(cx) < 0.35 and area < self.stop_area:
            cmd.linear.x = self.fwd
        self.cmd_pub.publish(cmd)

    def check_lost(self):
        """Stop if the object has not been seen recently (gone, or camera dropped)."""
        if not self.drive:
            return
        if self.last_seen is None:
            self.cmd_pub.publish(Twist())
            return
        elapsed = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        if elapsed > self.lost_timeout:
            self.cmd_pub.publish(Twist())   # zero, not silence

    def publish_debug(self, bgr, found, cx, cy, area):
        annotated = bgr.copy()
        h, w = annotated.shape[:2]
        colour = (0, 255, 0) if found else (0, 0, 255)
        text = (f'target x={cx:+.2f} area={area*100:.1f}%' if found
                else 'no target')
        cv2.putText(annotated, text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
        cv2.line(annotated, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
        if found:
            px = int((cx + 1) * w / 2)
            py = int((cy + 1) * h / 2)
            cv2.circle(annotated, (px, py), 12, colour, 2)
            cv2.line(annotated, (w // 2, h // 2), (px, py), colour, 2)
        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        self.debug_pub.publish(out)

    def stop(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ObjectFollower()

    stop = False

    def request_stop(_s, _f):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok() and node.drive:
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
