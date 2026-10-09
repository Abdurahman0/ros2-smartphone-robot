#!/usr/bin/env python3
"""Build a map: Gazebo + slam_toolbox + autonomous exploration, in one command.

PIPELINE
========
    Gazebo  --/scan-->  slam_toolbox  --/map-->  (occupancy grid)
      ^                      |
      |                      +--> map -> odom transform (the odometry correction)
      |
      +--/cmd_vel--  wander_node  <--/scan--  Gazebo

wander_node drives the robot around so that the LiDAR sees the whole room; slam_toolbox
turns those scans into a map and simultaneously corrects the robot's drifting estimate
of where it is. Neither could work alone: without motion there is nothing to map,
without localisation the scans cannot be stitched together. That mutual dependency is
what the "simultaneous" in SLAM refers to.

When the map looks complete, save it from another terminal with:
    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/indoor
which writes indoor.pgm (the image) and indoor.yaml (its resolution and origin).
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
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox.yaml'])
    node_params = PathJoinSubstitution([pkg_share, 'config', 'mapping_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'mapping.rviz'])

    x_arg = DeclareLaunchArgument('x', default_value='-3.0')
    y_arg = DeclareLaunchArgument('y', default_value='-3.0')
    world_arg = DeclareLaunchArgument(
        'world', default_value='indoor.world',
        description='World to map (indoor.world, or house.world for the 3D house).',
    )
    gz_args_arg = DeclareLaunchArgument(
        'gz_args', default_value='-r -v 3 -s',
        description="Arguments for gz sim; default is headless. Drop '-s' to see the GUI.",
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    wander_arg = DeclareLaunchArgument(
        'wander', default_value='true',
        description='Explore automatically until the map is complete, then stop.',
    )
    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='true',
        description='Also open arrow-key teleop, silent until the auto-explore finishes.',
    )
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description="Also open the robot's camera in a window.",
    )

    # Launch configurations are GLOBAL by default: an included launch file sees the
    # parent's arguments, and any launch_arguments set for it leak back out to the
    # parent afterwards. Both directions caused real bugs here:
    #   * without 'rviz': 'false', gazebo.launch.py inherited this file's rviz:=true
    #     and started a SECOND RViz with the wrong config;
    #   * with it but unscoped, that 'false' leaked back out and suppressed the RViz
    #     this file wanted to start.
    # GroupAction(scoped=True) is the fix: the child still sees the parent's values
    # (forwarding), but nothing set inside escapes.
    simulation = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
                ]),
                launch_arguments={
                    'gz_args': LaunchConfiguration('gz_args'),
                    'rviz': 'false',
                    'world': LaunchConfiguration('world'),
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                }.items(),
            ),
        ],
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    # slam_toolbox on Jazzy is a LIFECYCLE node. It starts in the 'unconfigured' state,
    # where it subscribes to nothing and publishes nothing -- it looks like it is running
    # but silently does no work. Something has to walk it through
    # configure -> activate.
    #
    # That something is nav2_lifecycle_manager, the same supervisor Navigation2 uses for
    # its own nodes. Lifecycle nodes exist precisely so a robot can bring subsystems up
    # in a controlled order and, more importantly, shut them down deterministically:
    # 'deactivate' on a running robot means the node stops acting but keeps its state,
    # which is how you stop a robot safely rather than by killing processes.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            # Bond checking disabled.
            #
            # The manager normally keeps a heartbeat "bond" with each managed node and
            # declares it dead if the beat is late. slam_toolbox never establishes that
            # bond, so the manager reports "unable to be reached ... Aborting bringup"
            # regardless of the timeout -- raising it to 20 s changed nothing. Meanwhile
            # SLAM is configured, activated and mapping perfectly well, because the
            # configure/activate transitions happened before the bond was ever checked.
            #
            # 0.0 disables the check. What is lost is automatic detection of a crashed
            # slam_toolbox; here that is worth trading for a bringup that does not report
            # a failure that has not happened.
            'bond_timeout': 0.0,
        }],
    )

    # Both are delayed: SLAM needs the /scan topic and the TF tree to exist before it
    # starts, and the robot is only spawned a few seconds into the Gazebo launch.
    # Starting SLAM too early makes it log TF errors and give up on the first scans.
    delayed_slam = TimerAction(period=10.0, actions=[slam, lifecycle_manager])

    wander = TimerAction(period=14.0, actions=[Node(
        package='smartphone_robot',
        executable='wander_node',
        name='wander_node',
        output='screen',
        parameters=[node_params, {'use_sim_time': True, 'auto_stop': True}],
    )], condition=IfCondition(LaunchConfiguration('wander')))

    # Arrow-key teleop, opened in its own xterm, silent until the first key. While
    # wander_node is auto-exploring it publishes nothing, so the two never fight; the
    # moment exploration finishes (wander stops) and you press an arrow, teleop takes
    # over. This is how you keep driving the robot after the map is built.
    teleop = TimerAction(period=16.0, actions=[Node(
        package='smartphone_robot',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[{'use_sim_time': True, 'silent_until_first_key': True}],
        prefix='xterm -e',
    )], condition=IfCondition(LaunchConfiguration('teleop')))

    # The robot's own camera, in its own window (see navigation.launch.py for why a
    # separate window rather than an embedded RViz panel).
    camera_view = TimerAction(period=14.0, actions=[Node(
        package='smartphone_robot',
        executable='camera_viewer',
        name='robot_camera_view',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'window_title': 'Robot camera',
            'window_width': 640,
            'use_sim_time': True,
        }],
    )], condition=IfCondition(LaunchConfiguration('camera')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg, x_arg, y_arg, gz_args_arg, rviz_arg, wander_arg, teleop_arg, camera_arg,
        simulation,
        delayed_slam,
        wander,
        teleop,
        camera_view,
        TimerAction(period=12.0, actions=[rviz]),
    ])
