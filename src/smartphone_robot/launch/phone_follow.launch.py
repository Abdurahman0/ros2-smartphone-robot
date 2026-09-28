#!/usr/bin/env python3
"""The full loop: a real phone camera drives the simulated robot.

    phone camera  --WiFi-->  phone_camera_bridge  -->  /phone/camera/image_raw
                                                              |
                                                       object_follower
                                                              |  /cmd_vel
                                                              v
                                                     Gazebo differential drive

Hold a coloured object in front of the phone and the simulated robot turns to face it and
drives toward it. This is the piece that makes the project more than a simulation demo:
real sensor data, processed with OpenCV, changing what the robot does.

It is deliberately the ONLY thing commanding /cmd_vel -- Nav2 is not launched here,
because two publishers fighting over the base helps no one. This is the "reactive" mode,
the counterpart to the "planned" mode in navigation.launch.py.

    ros2 launch smartphone_robot phone_follow.launch.py \\
        stream_url:='http://user:pass@172.20.10.1:8081/video'

The stream URL, with credentials, is required and has no default, so nothing secret lives
in the repository. Colour defaults to blue; override hue_min/hue_max for another colour,
or set hue_wrap:=true for red.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('smartphone_robot')

    url_arg = DeclareLaunchArgument(
        'stream_url',
        description='Phone MJPEG/RTSP URL WITH credentials, e.g. '
                    "http://user:pass@172.20.10.1:8081/video",
    )
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='true',
        description='Launch Gazebo so the robot actually moves. false = detection only.',
    )
    hue_min_arg = DeclareLaunchArgument('hue_min', default_value='100')
    hue_max_arg = DeclareLaunchArgument('hue_max', default_value='130')
    hue_wrap_arg = DeclareLaunchArgument('hue_wrap', default_value='false')

    simulation = GroupAction(
        scoped=True, forwarding=True,
        condition=IfCondition(LaunchConfiguration('sim')),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={'gz_args': '-r -v 3 -s', 'rviz': 'false'}.items(),
        )],
    )

    camera = Node(
        package='smartphone_robot', executable='phone_camera_bridge',
        name='phone_camera_bridge', output='screen',
        parameters=[{'stream_url': LaunchConfiguration('stream_url')}],
    )

    follower = Node(
        package='smartphone_robot', executable='object_follower',
        name='object_follower', output='screen',
        parameters=[{
            'hue_min': LaunchConfiguration('hue_min'),
            'hue_max': LaunchConfiguration('hue_max'),
            'hue_wrap': LaunchConfiguration('hue_wrap'),
            'drive': True,
            'use_sim_time': LaunchConfiguration('sim'),
        }],
    )

    return LaunchDescription([
        url_arg, sim_arg, hue_min_arg, hue_max_arg, hue_wrap_arg,
        simulation, camera, follower,
    ])
