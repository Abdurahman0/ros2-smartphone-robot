#!/usr/bin/env python3
"""Autonomous navigation: Gazebo + a saved map + AMCL + Navigation2.

DIFFERENCE FROM mapping.launch.py
=================================
While mapping, slam_toolbox built the map AND localised the robot in it at the same
time. Here the map is already known and fixed, so the job splits:

    map_server   loads maps/indoor.yaml and serves it on /map
    amcl         works out WHERE the robot is on that map, by matching each laser scan
                 against it, and publishes the map -> odom correction
    Nav2         plans and drives to goals

Both modes publish map -> odom; that transform always means the same thing (the
accumulated odometry error). Only the source of the correction differs. This is why the
navigation stack neither knows nor cares whether a map came from SLAM or from a floor
plan drawn by hand.

USAGE
    ros2 launch smartphone_robot navigation.launch.py
Then in RViz use "Nav2 Goal" to click a destination, or from a terminal:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 2.0}, \\
        orientation: {w: 1.0}}}}"

Note that this is an ACTION, for the reasons set out in drive_distance_server.py:
it takes many seconds, streams progress, and must be cancellable.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = FindPackageShare('smartphone_robot')

    map_file = PathJoinSubstitution([pkg_share, 'maps', 'indoor.yaml'])
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'navigation.rviz'])

    gz_args_arg = DeclareLaunchArgument('gz_args', default_value='-r -v 3 -s')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    map_arg = DeclareLaunchArgument('map', default_value=map_file)

    # Scoped so the 'rviz' argument below does not leak into the included launch file
    # and start a second RViz. See the same note in mapping.launch.py.
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
                    # Spawn where mapping began, so the saved map and the robot's
                    # actual position agree from the first scan.
                    'x': '-3.0',
                    'y': '-3.0',
                    'yaw': '0.0',
                }.items(),
            ),
        ],
    )

    # nav2_bringup's bringup_launch.py starts the whole stack: map_server, amcl, the
    # planner, controller, behaviour and BT servers, plus the lifecycle managers that
    # bring them all up in the right order. Reusing it rather than listing a dozen
    # nodes by hand is what the package is for.
    # The map path is injected INTO the parameter file rather than passed as nav2's
    # 'map' launch argument.
    #
    # Why: nav2_bringup defines two map_server nodes, selected by whether the 'map'
    # argument is empty, and the argument does not survive the hop from
    # bringup_launch.py into localization_launch.py here -- so the "no map given"
    # variant starts and map_server sits with an empty yaml_filename, AMCL logs
    # "Waiting for map...." forever, and the whole navigation bringup then aborts
    # because global_costmap can never find the 'map' frame.
    #
    # RewrittenYaml (nav2's own helper) rewrites the YAML at launch time, substituting
    # every 'yaml_filename' key with the resolved path. map_server then gets the map
    # from its parameters regardless of which variant starts.
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={'yaml_filename': LaunchConfiguration('map')},
        convert_types=True,
    )

    nav2 = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                          'launch', 'bringup_launch.py'])
                ]),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': 'true',
                    'params_file': configured_params,
                    'autostart': 'true',
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items(),
            ),
        ],
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
        gz_args_arg, rviz_arg, map_arg,
        simulation,
        # Nav2 is delayed until the robot exists and is publishing /scan and TF.
        # Starting AMCL before any transform is available makes it fail its first
        # updates and log a wall of transform errors.
        TimerAction(period=12.0, actions=[nav2]),
        TimerAction(period=16.0, actions=[rviz]),
    ])
