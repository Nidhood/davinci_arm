#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Model xacro file path:
    xacro_file = PathJoinSubstitution([
        FindPackageShare('davinci_arm_description'),
        'models',
        'davinci_arm_v1',
        'urdf',
        'davinci_arm.urdf.xacro'
    ])
    
    # Physics variables configuration file path:
    physics_config_file = PathJoinSubstitution([
        FindPackageShare('davinci_arm_description'),
        'config',
        'dynamics_params.yaml'
    ])
    
    # Robot state publisher node:
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='davinci_arm_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', xacro_file,
                    ' physics_config_file:=', physics_config_file
                    ]),
                value_type=str
                )
        }]
    )
    
    # Return launch description:
    return LaunchDescription([
        robot_state_publisher_node
    ])