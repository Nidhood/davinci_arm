#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_moveit_rviz = LaunchConfiguration('launch_moveit_rviz')

    pkg_gz = FindPackageShare('davinci_arm_gazebo')
    pkg_moveit = FindPackageShare('davinci_arm_moveit_config')

    # Your existing Gazebo stack
    start_gazebo_launch = PathJoinSubstitution([
        pkg_gz, 'launch', 'start_gazebo.launch.py'
    ])

    # Generated MoveIt launch files
    move_group_launch = PathJoinSubstitution([
        pkg_moveit, 'launch', 'move_group.launch.py'
    ])

    moveit_rviz_launch = PathJoinSubstitution([
        pkg_moveit, 'launch', 'moveit_rviz.launch.py'
    ])

    # Relay current simulated joint state to the topic MoveIt expects
    joint_states_relay = Node(
        package='topic_tools',
        executable='relay',
        name='joint_states_relay',
        arguments=['/davinci_arm/joint_states', '/joint_states'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),
        DeclareLaunchArgument(
            'launch_moveit_rviz',
            default_value='true',
            description='Launch RViz with the MoveIt MotionPlanning plugin'
        ),

        # 1) Start your existing Gazebo + robot + bridge stack:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(start_gazebo_launch),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),

        # 2) After Gazebo is up and publishing /davinci_arm/joint_states:
        TimerAction(
            period=10.0,
            actions=[joint_states_relay]
        ),

        # 3) Start move_group:
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch),
                    launch_arguments={'use_sim_time': 'true'}.items()
                )
            ]
        ),

        # 4) Start MoveIt RViz:
        TimerAction(
            period=14.0,
            condition=IfCondition(launch_moveit_rviz),
            actions=[     
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_rviz_launch),
                    launch_arguments={'use_sim_time': 'true'}.items()
                )
            ]
        ),
    ])