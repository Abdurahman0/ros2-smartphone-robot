#!/usr/bin/env python3
"""Publish a smartphone's camera into ROS 2 as sensor_msgs/Image.

WHAT THIS IS
============
A camera driver. It opens the phone's video stream over the network and republishes each
frame as the exact message a real camera driver or Gazebo's camera plugin publishes:
sensor_msgs/Image on /phone/camera/image_raw, plus a matching CameraInfo. Anything that
consumes camera images -- an OpenCV detector, image_view, a rosbag -- cannot tell the
source apart.

    phone (IP Camera Lite)  --RTSP/MJPEG-->  this node  --DDS-->  /phone/camera/image_raw

WHY OpenCV DOES THE NETWORKING, NOT us
======================================
The phone serves both an MJPEG stream over HTTP and an RTSP stream. Either could be
parsed by hand, but cv2.VideoCapture already speaks both, handles reconnection, and
decodes the frames. Reimplementing an MJPEG boundary parser or an RTSP/RTP stack here
would be effort spent reproducing what the vision library the project already depends on
does correctly. The node's job is the ROS side: timestamps, frame ids, encoding, QoS.

CREDENTIALS
===========
The stream is password protected, which is the right default for a camera on a network.
The URL (with credentials) is a parameter, never hard-coded, so it stays out of the
source and out of git. Pass it at launch:

    ros2 run smartphone_robot phone_camera_bridge --ros-args \\
      -p stream_url:='http://user:pass@172.20.10.1:8081/video'

THROUGHPUT, AND WHY A DEDICATED THREAD
======================================
Reading a frame blocks until one is decoded. Doing that inside a ROS timer would stall
the executor between frames. So a background thread does nothing but pull the newest
frame into a slot, and a timer publishes whatever is in the slot at a fixed rate. That
also drops frames rather than queuing them when the network is slow -- for a live camera,
the freshest frame is the only one worth having.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
import signal

import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class PhoneCameraBridge(Node):
    """Streams the phone camera into /phone/camera/image_raw."""

    def __init__(self):
        super().__init__('phone_camera_bridge')

        # No default URL on purpose: an empty one fails loudly and immediately, rather
        # than silently trying to open nothing. Credentials must be supplied by the
        # caller, so they never live in this file.
        self.declare_parameter('stream_url', '')
        # 10 Hz, matched to what the phone actually delivers over this link (measured
        # ~10 Hz raw). Publishing faster just republishes duplicate frames.
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'phone_camera_optical_link')
        self.declare_parameter('reconnect_delay', 2.0)  # s between reopen attempts
        # Downscale before publishing. The phone sends 1920x1080; a raw frame at that
        # size is 6.2 MB, and pushing 15 of them per second through DDS is 93 MB/s, which
        # the transport cannot sustain -- measured throughput collapsed to 1.9 Hz. Robot
        # perception does not need 1080p anyway; the simulated camera is 640x480. Scaling
        # to 640 wide cuts each frame to ~0.7 MB and lets the full frame rate through.
        self.declare_parameter('output_width', 640)    # 0 keeps the source size

        self.url = self.get_parameter('stream_url').value
        if not self.url:
            self.get_logger().fatal(
                'stream_url is empty. Pass it, with credentials, e.g. '
                "-p stream_url:='http://user:pass@172.20.10.1:8081/video'"
            )
            raise SystemExit(1)

        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value
        rate = self.get_parameter('publish_rate').value
        self.out_width = self.get_parameter('output_width').value

        # Images are large; BEST_EFFORT avoids the retransmit machinery of RELIABLE,
        # which is wrong for a live video feed where a late frame is worthless.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        self.image_pub = self.create_publisher(Image, 'phone/camera/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, 'phone/camera/camera_info', qos)

        self.bridge = CvBridge()
        self._latest = None
        self._lock = threading.Lock()
        self._running = True
        self._connected = False
        self.frame_count = 0

        # The grab loop runs on its own thread so a blocking read never stalls the node.
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

        self.create_timer(1.0 / rate, self.publish_latest)
        # Hide credentials when echoing the URL to the log.
        safe = self.url
        if '@' in safe:
            safe = safe.split('://')[0] + '://***@' + safe.split('@', 1)[1]
        self.get_logger().info(f'phone_camera_bridge opening {safe}')

    def _grab_loop(self):
        """Continuously pull the newest frame into a shared slot. Reconnects on failure."""
        cap = None
        while self._running:
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                cap = cv2.VideoCapture(self.url)
                # Keep OpenCV's own buffer tiny so read() returns the latest frame, not a
                # backlog -- otherwise the feed lags further behind real time every second.
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    self.get_logger().warn(
                        f'Cannot open stream; retrying in {self.reconnect_delay}s. '
                        f'Is IP Camera Lite in the foreground with the server started?',
                        throttle_duration_sec=5.0,
                    )
                    self._sleep(self.reconnect_delay)
                    continue
                self.get_logger().info('Stream opened.')
                self._connected = True

            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warn('Frame read failed; reconnecting.',
                                       throttle_duration_sec=5.0)
                self._connected = False
                cap.release()
                cap = None
                self._sleep(self.reconnect_delay)
                continue

            with self._lock:
                self._latest = frame

        if cap is not None:
            cap.release()

    def _sleep(self, seconds):
        """Interruptible sleep, so shutdown does not wait out a reconnect delay."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self._running and self.get_clock().now().nanoseconds < end:
            import time
            time.sleep(0.05)

    def publish_latest(self):
        """Publish whatever frame the grab thread last stored. Runs on the ROS timer."""
        with self._lock:
            frame = self._latest
            self._latest = None   # publish each frame at most once
        if frame is None:
            return

        # Downscale on the publisher side, before serialisation, so the large frame never
        # goes near DDS. Aspect ratio is preserved.
        if self.out_width and frame.shape[1] > self.out_width:
            h, w = frame.shape[:2]
            new_h = int(h * self.out_width / w)
            frame = cv2.resize(frame, (self.out_width, new_h), interpolation=cv2.INTER_AREA)

        stamp = self.get_clock().now().to_msg()
        # IP Camera Lite delivers BGR; cv_bridge tags it bgr8, the OpenCV-native order,
        # so no colour conversion happens on this hop.
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

        # A minimal CameraInfo. The intrinsics are left unset because this camera has not
        # been calibrated -- publishing invented ones would let downstream code compute
        # confidently wrong 3D positions. The frame id and size are honest; the calibration
        # is absent, and marked so by a zero K matrix.
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width = frame.shape[:2]
        self.info_pub.publish(info)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'{self.frame_count} frames published ({frame.shape[1]}x{frame.shape[0]})'
            )

    def stop(self):
        self._running = False


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PhoneCameraBridge()

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
        node.stop()
        if rclpy.ok():
            node.get_logger().info(f'Shutting down after {node.frame_count} frames.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
