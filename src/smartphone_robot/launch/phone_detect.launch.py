#!/usr/bin/env python3
"""Neural object detection on the phone camera, with a live labelled window.

    phone camera  --WiFi-->  phone_camera_bridge  -->  /phone/camera/image_raw
                                                             |
                                                       object_detector  (YOLOv4-tiny)
                                                             |  /detections/image
                                                             v
                                                       camera_viewer  (a window)

Point the phone at a room and the window labels the real objects it recognises -- people,
laptops, bottles, chairs -- each with a box and a confidence. This is the neural detector,
the counterpart to the colour-blob follower: it knows WHAT things are, not just where a
colour is.

No Gazebo here: this demo is about perception on a real camera, not about moving the
robot. It needs only the phone.

    ros2 launch smartphone_robot phone_detect.launch.py \\
        stream_url:='http://user:pass@172.20.10.1:8081/video'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    url_arg = DeclareLaunchArgument(
        'stream_url',
        description='Phone MJPEG/RTSP URL with credentials.',
    )

    camera = Node(
        package='smartphone_robot', executable='phone_camera_bridge',
        name='phone_camera_bridge', output='screen',
        parameters=[{'stream_url': LaunchConfiguration('stream_url')}],
    )

    detector = Node(
        package='smartphone_robot', executable='object_detector',
        name='object_detector', output='screen',
        parameters=[{'image_topic': '/phone/camera/image_raw'}],
    )

    viewer = Node(
        package='smartphone_robot', executable='camera_viewer',
        name='detection_view', output='screen',
        parameters=[{
            'image_topic': '/detections/image',
            'window_title': 'Phone camera - object detection',
            'window_width': 1000,
        }],
    )

    return LaunchDescription([
        url_arg,
        camera,
        # Give the camera a moment to open before the detector and viewer attach.
        TimerAction(period=3.0, actions=[detector]),
        TimerAction(period=5.0, actions=[viewer]),
    ])
