#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Spawn the joint state broadcaster first.
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawn the trajectory controller a bit later.
    spawn_davinci_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_davinci_arm_controller',
        arguments=[
            'davinci_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        spawn_joint_state_broadcaster,

        TimerAction(
            period=2.0,
            actions=[spawn_davinci_arm_controller]
        ),
    ])