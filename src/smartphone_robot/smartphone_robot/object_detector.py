#!/usr/bin/env python3
"""Detect and label real objects in a camera stream with a neural network.

WHAT THIS IS, AND HOW IT DIFFERS FROM object_follower
=====================================================
object_follower finds one coloured blob and steers toward it -- classical computer
vision, no learning. This node runs a trained neural network (YOLOv4-tiny, 80 COCO
classes) and RECOGNISES what things are: "person 87%", "laptop 72%", "chair 64%". It
labels every object it finds, each with a box and a confidence.

The two are complementary and honest about their limits:
  * colour detection is trivial, robust and needs no model, but only knows "blue";
  * the neural detector knows real object categories, but only the 80 it was trained on,
    and it costs far more compute.

WHY IT RUNS ON THE PHONE CAMERA, NOT THE ROBOT'S
================================================
The network was trained on photos of the real world, so it recognises real objects:
people, laptops, bottles, chairs. The robot's SIMULATED camera sees only walls, crates
and pillars, which are not COCO categories -- so YOLO finds nothing there. This detector
is therefore meaningful on the phone camera (a real room) and near-useless on the
simulated one. That is not a bug; it is the honest reason a real sensor is worth having.

The node subscribes to a configurable topic, so the same code runs on either camera --
the point is that swapping the source is a parameter, not a rewrite.

INFERENCE
=========
OpenCV's DNN module loads the Darknet model and runs it on the CPU. YOLOv4-tiny is the
small variant precisely so this is possible without a GPU; expect a handful of frames
per second, which is plenty for labelling a scene.
"""

import os

import cv2
import numpy as np
from rclpy.node import Node
from smartphone_robot.node_utils import run_node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ObjectDetector(Node):
    """Runs YOLOv4-tiny on an image topic and publishes a labelled image."""

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('image_topic', '/phone/camera/image_raw')
        self.declare_parameter('model_dir', os.path.expanduser('~/ros2_ws/models/yolo'))
        self.declare_parameter('conf_threshold', 0.35)   # ignore weaker detections
        self.declare_parameter('nms_threshold', 0.4)     # merge overlapping boxes
        self.declare_parameter('input_size', 416)        # network input; 416 is standard
        # Run the network on every Nth frame. Inference is the expensive step, and a
        # scene does not change much in 100 ms, so skipping frames keeps the node
        # responsive without missing anything a demo cares about.
        self.declare_parameter('detect_every', 2)

        topic = self.get_parameter('image_topic').value
        model_dir = self.get_parameter('model_dir').value
        self.conf = self.get_parameter('conf_threshold').value
        self.nms = self.get_parameter('nms_threshold').value
        size = self.get_parameter('input_size').value
        self.detect_every = max(1, self.get_parameter('detect_every').value)

        cfg = os.path.join(model_dir, 'yolov4-tiny.cfg')
        weights = os.path.join(model_dir, 'yolov4-tiny.weights')
        names_file = os.path.join(model_dir, 'coco.names')
        for f in (cfg, weights, names_file):
            if not os.path.exists(f):
                self.get_logger().fatal(
                    f'Model file missing: {f}. Run scripts/download_yolo.sh first.'
                )
                raise SystemExit(1)

        net = cv2.dnn.readNetFromDarknet(cfg, weights)
        # CPU backend explicitly: no GPU is assumed, and this keeps behaviour identical
        # on any machine.
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.model = cv2.dnn_DetectionModel(net)
        self.model.setInputParams(size=(size, size), scale=1 / 255.0, swapRB=True)
        self.names = open(names_file).read().strip().split('\n')

        # A stable colour per class, so the same object type is always drawn the same
        # colour across frames -- easier to read than random colours that flicker.
        rng = np.random.default_rng(42)
        self.colours = rng.integers(60, 255, size=(len(self.names), 3)).tolist()

        self.bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Image, topic, self.on_image, qos)
        self.image_pub = self.create_publisher(Image, 'detections/image', qos)
        # A plain text summary of what is currently in view, so other nodes (or a human
        # reading `ros2 topic echo`) can use the result without decoding an image.
        self.summary_pub = self.create_publisher(String, 'detections/summary', 10)

        self.frame_i = 0
        self.last = []   # remember the last detections so skipped frames still draw them
        self.get_logger().info(
            f'object_detector: YOLOv4-tiny on {topic}, {len(self.names)} classes, '
            f'inference every {self.detect_every} frame(s)'
        )

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_i += 1

        if self.frame_i % self.detect_every == 0:
            classes, scores, boxes = self.model.detect(frame, self.conf, self.nms)
            self.last = list(zip(
                classes.flatten() if len(classes) else [],
                scores.flatten() if len(scores) else [],
                boxes,
            ))
            self.publish_summary()

        # Draw the most recent detections (fresh or from the last inference frame).
        for cls, score, box in self.last:
            x, y, w, h = box
            colour = self.colours[int(cls)]
            label = f'{self.names[int(cls)]} {int(score * 100)}%'
            cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x, y - th - 8), (x + tw + 4, y), colour, -1)
            cv2.putText(frame, label, (x + 2, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)

        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    def publish_summary(self):
        if not self.last:
            text = 'nothing detected'
        else:
            counts = {}
            for cls, _score, _box in self.last:
                name = self.names[int(cls)]
                counts[name] = counts.get(name, 0) + 1
            text = ', '.join(f'{n}x {name}' if n > 1 else name
                             for name, n in sorted(counts.items()))
        self.summary_pub.publish(String(data=text))
        self.get_logger().info(f'detected: {text}', throttle_duration_sec=1.0)



def main(args=None):
    run_node(ObjectDetector, args=args)


if __name__ == '__main__':
    main()
