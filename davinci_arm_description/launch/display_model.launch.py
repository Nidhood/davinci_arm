#!usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # rviz configuration file path:
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('davinci_arm_description'),
        'rviz',
        'davinci_arm_display.rviz'
    ])
    
    # rviz2 node:
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # joint state publisher gui node:
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Return launch description:
    return LaunchDescription([
        rviz_node,
        joint_state_publisher_gui_node
    ])