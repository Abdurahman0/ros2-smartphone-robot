#!/usr/bin/env python3
"""Drive the robot by hand with the arrow keys, watching it in RViz.

    keyboard (arrows)  -->  keyboard_teleop  -->  /cmd_vel  -->  Gazebo robot

Gazebo + a clean RViz view + the arrow-key teleop. No SLAM and no Nav2: this is manual
driving, so nothing else should be writing /cmd_vel.

    ros2 launch smartphone_robot teleop.launch.py

The teleop reads the keyboard, so it must run in a terminal you can type into. When the
launch owns the terminal that is automatic; the arrow keys drive the robot as soon as the
RViz window has appeared.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('smartphone_robot')
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'teleop.rviz'])

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    simulation = GroupAction(
        scoped=True, forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={'gz_args': '-r -v 3 -s', 'rviz': 'false',
                              'x': '0.0', 'y': '0.0', 'yaw': '0.0'}.items(),
        )],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # prefix opens the teleop in its own xterm so it has a keyboard even when the launch
    # is backgrounded. If xterm is missing it still runs in the launching terminal.
    teleop = Node(
        package='smartphone_robot', executable='keyboard_teleop',
        name='keyboard_teleop', output='screen',
        parameters=[{'use_sim_time': True}],
        prefix='xterm -e',
    )

    return LaunchDescription([
        rviz_arg,
        simulation,
        TimerAction(period=8.0, actions=[rviz]),
        TimerAction(period=9.0, actions=[teleop]),
    ])
