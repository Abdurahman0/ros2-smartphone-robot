#!/usr/bin/env python3
"""Vision demo, fully in simulation: the robot follows a coloured object with its OWN camera.

    Gazebo camera  -->  /camera/image_raw  -->  object_follower  -->  /cmd_vel  -->  robot

This is the same object_follower node used with the phone (phone_follow.launch.py). Only
its input topic changes -- from the phone's stream to the robot's own simulated camera.
That is the point worth showing: the perception code does not know or care where its
images come from, so the phone demo and this one share every line of detection and control
logic. Swapping the sensor is a launch remapping, not a rewrite.

Why this version exists alongside the phone one:
  * it is fully reproducible -- no phone, no battery, no network, nothing to go wrong in
    front of an audience;
  * the robot's camera view can be shown next to the world, so what the robot sees and what
    it does are visible together;
  * it needs only this machine.

The target is the bright red post in indoor.world. Detection is tuned for red, whose hue
wraps around the 0/180 boundary, so hue_wrap is set.

    ros2 launch smartphone_robot follow_sim.launch.py
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
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'follow.rviz'])

    gz_args_arg = DeclareLaunchArgument('gz_args', default_value='-r -v 3 -s')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')

    simulation = GroupAction(
        scoped=True, forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={
                'gz_args': LaunchConfiguration('gz_args'),
                'rviz': 'false',
                # Spawn at the mapping origin, facing +x -- straight at the red post.
                'x': '-3.0', 'y': '-3.0', 'yaw': '0.0',
            }.items(),
        )],
    )

    follower = Node(
        package='smartphone_robot', executable='object_follower',
        name='object_follower', output='screen',
        parameters=[{
            'use_sim_time': True,
            # Red: its hue sits at both ends of the 0-179 range, so wrap two bands.
            'hue_wrap': True,
            'hue_min': 165,   # upper band start
            'hue_max': 12,    # lower band end
            'sat_min': 120,   # the post is highly saturated; a high floor rejects walls
            'val_min': 60,
            'drive': True,
            'forward_speed': 0.2,
            'stop_area_frac': 0.10,   # stop once the post fills ~10% of the frame
        }],
        # The one line that makes this the robot's camera instead of the phone's.
        remappings=[('phone/camera/image_raw', '/camera/image_raw')],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([gz_args_arg, rviz_arg, simulation, follower, rviz])
