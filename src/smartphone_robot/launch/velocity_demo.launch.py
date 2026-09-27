#!/usr/bin/env python3
"""Start the velocity publisher and the monitor together.

Why a launch file at all? Running each node by hand in its own terminal does not
scale: the finished system in this project will start Gazebo, a robot state
publisher, SLAM, Nav2, RViz and several custom nodes -- ten-plus processes with
interdependent parameters. A launch file makes that one command, reproducible,
and version-controlled.

A launch file is ordinary Python, but it does NOT run the nodes itself. It builds
and returns a LaunchDescription -- a *description* of what should run. The launch
system then executes it, supervises the processes and shuts them down together.
That distinction is why values below are "substitutions" rather than plain strings:
they are resolved at launch time, not while this function runs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Called by `ros2 launch`. The name is fixed -- it must be exactly this."""

    # --- Launch arguments -----------------------------------------------------
    # These become command-line options:
    #   ros2 launch smartphone_robot velocity_demo.launch.py monitor:=false
    # `ros2 launch <pkg> <file> --show-args` lists them with their descriptions.
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            # FindPackageShare resolves to install/smartphone_robot/share/... at
            # runtime. Never hard-code an absolute path: it would break on any
            # other machine, and on the robot itself.
            FindPackageShare('smartphone_robot'),
            'config',
            'velocity_params.yaml',
        ]),
        description='YAML file with parameters for both nodes.',
    )

    monitor_arg = DeclareLaunchArgument(
        'monitor',
        default_value='true',
        description='Also start velocity_monitor (set false to run the publisher alone).',
    )

    params_file = LaunchConfiguration('params_file')

    # --- Nodes ----------------------------------------------------------------
    velocity_publisher = Node(
        package='smartphone_robot',
        executable='velocity_publisher',   # the name from setup.py entry_points
        name='velocity_publisher',         # must match the key in the YAML file
        parameters=[params_file],
        output='screen',                   # show logs in this terminal
        emulate_tty=True,                  # keep rclpy's colour codes readable
    )

    velocity_monitor = Node(
        package='smartphone_robot',
        executable='velocity_monitor',
        name='velocity_monitor',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
        # IfCondition makes this node conditional on the 'monitor' argument.
        condition=IfCondition(LaunchConfiguration('monitor')),
    )

    return LaunchDescription([
        params_file_arg,
        monitor_arg,
        LogInfo(msg=['Loading parameters from: ', params_file]),
        velocity_publisher,
        velocity_monitor,
    ])
