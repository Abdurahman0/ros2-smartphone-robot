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

WHY THE NODES ARE LISTED HERE INSTEAD OF INCLUDING nav2_bringup
===============================================================
nav2_bringup's bringup_launch.py starts the full Nav2 product: docking_server,
route_server and waypoint_follower as well as the planning and control stack. This
project uses none of those three, and they are not free:

  * they must still be given valid parameters, or the lifecycle manager aborts the
    entire bringup -- a missing docking parameter stops navigation from starting at all;
  * they cost memory, which is scarce on this machine;
  * and they add failure modes. docking_server timed out sending its lifecycle response
    under load, so the manager waited forever and the navigation servers never activated.
    Nothing reported an error; navigation simply rejected every goal as "inactive".

Listing the nodes explicitly makes the stack exactly as large as the problem, and makes
the /cmd_vel chain below visible rather than hidden inside someone else's launch file:

    controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
                      -> collision_monitor -> /cmd_vel -> the robot

USAGE
    ros2 launch smartphone_robot navigation.launch.py
Then click "Nav2 Goal" in RViz -- press, drag to set the heading, release -- or:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0}, \\
        orientation: {w: 1.0}}}}" --feedback
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
    # and start a second RViz. Launch configurations are global by default.
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
                    # Spawn where mapping began, so the saved map and the robot's actual
                    # position agree from the first scan.
                    'x': '-3.0',
                    'y': '-3.0',
                    'yaw': '0.0',
                }.items(),
            ),
        ],
    )

    # The map path is injected INTO the parameter file rather than passed as nav2's
    # 'map' launch argument, because map_server reads yaml_filename from its parameters.
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={'yaml_filename': LaunchConfiguration('map')},
        convert_types=True,
    )

    # Nav2 nodes publish TF on the global /tf; remapping keeps that explicit and makes
    # the stack work unchanged inside a namespace.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = dict(output='screen', parameters=[configured_params], remappings=remappings)

    localization_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', **common),
        Node(package='nav2_amcl', executable='amcl', name='amcl', **common),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ]

    navigation_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[configured_params],
             # The controller does NOT write straight to the robot. Its output goes to
             # the smoother, then the collision monitor, and only then to /cmd_vel.
             remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', **common),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **common),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[configured_params],
             remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        # Last in the chain: watches raw laser data independently of the costmaps and
        # scales or stops the command if the robot is about to hit something. Its input
        # and output topics come from nav2_params.yaml.
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', **common),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server', 'smoother_server', 'planner_server',
                    'behavior_server', 'bt_navigator', 'velocity_smoother',
                    'collision_monitor',
                ],
                # Generous, because these nodes share a machine with Gazebo. A late
                # heartbeat here is a busy CPU, not a crashed server.
                'bond_timeout': 20.0,
            }],
        ),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        gz_args_arg, rviz_arg, map_arg,
        simulation,
        # Delayed until the robot exists and is publishing /scan and TF. AMCL started
        # before any transform is available fails its first updates and logs a wall of
        # transform errors.
        TimerAction(period=12.0, actions=localization_nodes),
        TimerAction(period=15.0, actions=navigation_nodes),
        TimerAction(period=19.0, actions=[rviz]),
    ])
