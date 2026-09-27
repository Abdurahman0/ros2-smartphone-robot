#!/usr/bin/env python3
"""Visualise the robot description in RViz, with no simulator involved.

This launch file answers one question: "is my URDF correct?" -- and it answers it
before Gazebo, SLAM or Nav2 are in the picture, when a mistake is still cheap to find.

Three processes, and the division of labour between them is the lesson:

  robot_state_publisher     reads the URDF, listens to /joint_states, and publishes the
                            TF tree. It is the single source of truth for "where is
                            every part of the robot relative to every other part".
                            It runs unchanged on real hardware.

  joint_state_publisher_gui gives sliders for the two continuous wheel joints. It is a
                            STAND-IN for reality: on a physical robot the wheel angles
                            come from encoders, and in Gazebo they come from the
                            simulated joints. This node exists only because neither is
                            present yet -- without something publishing /joint_states,
                            the wheels would have no known angle and TF would be
                            incomplete.

  rviz2                     draws it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('smartphone_robot')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'robot_view.rviz'])

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Use joint_state_publisher_gui (sliders) instead of the headless one.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Follow /clock instead of the wall clock. Must be true under Gazebo.',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Command() runs `xacro <file>` at launch time and captures its stdout. The URDF is
    # therefore always freshly expanded -- editing the xacro and relaunching is enough,
    # with no build step and no generated .urdf file to keep in sync.
    #
    # ParameterValue(..., value_type=str) is required: without it the long XML string
    # gets guessed as some other type and robot_state_publisher rejects it.
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
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
