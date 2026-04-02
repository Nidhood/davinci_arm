#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_moveit_rviz = LaunchConfiguration('launch_moveit_rviz')
    launch_controllers = LaunchConfiguration('launch_controllers')

    pkg_gz = FindPackageShare('davinci_arm_gazebo')
    pkg_gz_plugins = FindPackageShare('davinci_arm_gazebo_plugins')
    pkg_moveit = FindPackageShare('davinci_arm_moveit_config')

    # Existing Gazebo stack
    start_gazebo_launch = PathJoinSubstitution([
        pkg_gz, 'launch', 'start_gazebo.launch.py'
    ])

    # Controllers launcher from gazebo plugins package
    spawn_controllers_launch = PathJoinSubstitution([
        pkg_gz_plugins, 'launch', 'spawn_controllers.launch.py'
    ])

    # Generated MoveIt launch files
    move_group_launch = PathJoinSubstitution([
        pkg_moveit, 'launch', 'move_group.launch.py'
    ])

    moveit_rviz_launch = PathJoinSubstitution([
        pkg_moveit, 'launch', 'moveit_rviz.launch.py'
    ])

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
        DeclareLaunchArgument(
            'launch_controllers',
            default_value='true',
            description='Spawn ros2_control controllers after Gazebo starts'
        ),

        # 1. Start Gazebo + robot + bridge stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(start_gazebo_launch),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),

        # 2. Spawn ros2_control controllers from gazebo_plugins package
        TimerAction(
            period=10.0,
            condition=IfCondition(launch_controllers),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_controllers_launch),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items()
                )
            ]
        ),

        # 3. Start move_group
        TimerAction(
            period=14.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items()
                )
            ]
        ),

        # 4. Start MoveIt RViz
        TimerAction(
            period=16.0,
            condition=IfCondition(launch_moveit_rviz),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_rviz_launch),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items()
                )
            ]
        ),
    ])