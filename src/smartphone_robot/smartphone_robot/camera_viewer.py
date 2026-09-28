#!/usr/bin/env python3
"""Show a ROS image topic in a large, resizable OpenCV window.

Why not rqt_image_view: it opens at a tiny default size, often behind other windows, and
takes no size argument -- awkward to place beside RViz for a demo. This gives one plain
window at a chosen size, always on the same spot, that scales the frame up cleanly.

Deliberately trivial. It subscribes to an image topic, converts it with cv_bridge, and
shows it. It runs the display on the ROS callback and pumps the GUI with waitKey(1), which
is the standard way to drive an OpenCV window from inside a node.

    ros2 run smartphone_robot camera_viewer --ros-args \\
        -p image_topic:=/camera/image_raw -p window_width:=960
"""

import signal

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('window_title', 'Robot camera')
        self.declare_parameter('window_width', 960)   # window is scaled to this width

        topic = self.get_parameter('image_topic').value
        self.title = self.get_parameter('window_title').value
        self.win_w = self.get_parameter('window_width').value

        # WINDOW_NORMAL makes the window user-resizable; then we size it once up front.
        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.title, self.win_w, int(self.win_w * 9 / 16))
        cv2.moveWindow(self.title, 40, 40)

        self.bridge = CvBridge()
        # Match the camera's BEST_EFFORT QoS, or nothing arrives -- the usual ROS 2 trap.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Image, topic, self.on_image, qos)
        self.frames = 0
        self.get_logger().info(f'camera_viewer showing {topic} in a {self.win_w}px window')

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow(self.title, frame)
        cv2.waitKey(1)   # lets the window repaint; also processes close events
        self.frames += 1

    def close(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CameraViewer()

    stop = False

    def request_stop(_s, _f):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
