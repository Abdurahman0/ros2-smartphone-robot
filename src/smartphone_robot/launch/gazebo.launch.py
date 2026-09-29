#!/usr/bin/env python3
"""Bring up the full simulation: Gazebo, the robot, and the ROS bridge.

WHAT RUNS WHERE
===============
  gz sim                  the simulator. Physics, rendering, simulated sensors.
                          Knows nothing about ROS.
  robot_state_publisher   reads the URDF and publishes the fixed TF transforms.
                          Identical to the real-hardware setup.
  create                  a one-shot client that asks Gazebo to insert the robot,
                          taking the model from the /robot_description topic. It
                          exits as soon as the robot is in.
  ros_gz_bridge           translates between gz-transport and ROS 2, per
                          config/gz_bridge.yaml.
  image_bridge            a dedicated bridge for camera images. Separate because
                          images are large and this one avoids a redundant copy.

WHY use_sim_time MATTERS
========================
Gazebo publishes its own clock on /clock, and it does not run at wall-clock speed. On
this machine it will run slower than real time. Every ROS node here therefore sets
use_sim_time=true so they all agree on what "now" means. Forgetting it on a single node
is a classic and very confusing bug: TF lookups start failing with "extrapolation into
the future" because that node's timestamps come from a different clock.
"""

import subprocess

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def _simulation_already_running() -> bool:
    """True if a gz sim process is already up.

    Running two simulations at once is not an error anyone notices immediately, and the
    symptoms are baffling: two /clock publishers make simulated time jump backwards, TF
    buffers clear themselves, the robot appears to teleport, and Nav2 aborts goals with
    transform errors that look like a tuning problem. It happened here simply because
    two people started the same launch file a few minutes apart.

    Refusing to start the second one turns a confusing failure into a clear message.
    """
    try:
        return subprocess.run(['pgrep', '-f', 'gz sim'],
                              capture_output=True).returncode == 0
    except Exception:
        return False   # never block a launch because the check itself failed


def generate_launch_description():
    if _simulation_already_running():
        return LaunchDescription([
            LogInfo(msg=[
                '\n',
                '======================================================================\n',
                ' A simulation is ALREADY RUNNING.\n',
                '\n',
                ' Starting a second one gives two /clock publishers, which makes\n',
                ' simulated time jump backwards. Everything downstream then fails in\n',
                ' ways that look like bugs: the robot teleports, TF buffers clear, and\n',
                ' navigation goals abort with transform errors.\n',
                '\n',
                ' Stop the running one first:   bash scripts/stop_all.sh\n',
                '======================================================================\n',
            ]),
            Shutdown(reason='simulation already running'),
        ])

    pkg_share = FindPackageShare('smartphone_robot')

    # Which world to load, from the package's worlds/ directory. Default is the built-in
    # indoor room; pass world:=house.world to map a downloaded 3D house instead.
    world_file = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    bridge_config = PathJoinSubstitution([pkg_share, 'config', 'gz_bridge.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'navigation.rviz'])

    # '-r' starts the simulation running rather than paused.
    # '-v 3' sets log verbosity. Append '-s' for a headless server (much lighter on
    # RAM and CPU, which matters on this machine).
    world_arg = DeclareLaunchArgument(
        'world', default_value='indoor.world',
        description='World file in worlds/ to load (e.g. indoor.world, house.world).',
    )
    gz_args_arg = DeclareLaunchArgument(
        'gz_args', default_value='-r -v 3',
        description="Extra arguments for gz sim. Add '-s' to run headless.",
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Also start RViz.',
    )
    x_arg = DeclareLaunchArgument('x', default_value='-3.0')
    y_arg = DeclareLaunchArgument('y', default_value='-3.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' ', LaunchConfiguration('gz_args')],
        }.items(),
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawning is delayed: Gazebo needs a few seconds to load the world, and
    # robot_state_publisher needs to have published /robot_description first.
    spawn_robot = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'smartphone_robot',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', '0.1',
                '-Y', LaunchConfiguration('yaw'),
            ],
        )],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    # image_bridge takes the topic name as a plain argument, not from the YAML.
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_image_bridge',
        output='screen',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg, gz_args_arg, rviz_arg, x_arg, y_arg, yaw_arg,
        gz_sim,
        robot_state_publisher,
        bridge,
        image_bridge,
        spawn_robot,
        rviz,
    ])
