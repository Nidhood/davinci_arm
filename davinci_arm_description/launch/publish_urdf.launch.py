#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_state_publisher = LaunchConfiguration('launch_state_publisher')
    joint_states_topic = LaunchConfiguration('joint_states_topic')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('davinci_arm_description'),
        'models',
        'davinci_arm_v3',
        'urdf',
        'davinci_arm.urdf.xacro'
    ])

    physics_config_file = PathJoinSubstitution([
        FindPackageShare('davinci_arm_description'),
        'config',
        'dynamics_params_v3.yaml'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='davinci_arm_state_publisher',
        condition=IfCondition(launch_state_publisher),
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command([
                    'xacro ', xacro_file,
                    ' physics_config_file:=', physics_config_file
                ]),
                value_type=str
            )
        }],
        remappings=[
            ('/joint_states', joint_states_topic)
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'launch_state_publisher',
            default_value='true',
            description='Launch robot_state_publisher'
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='Topic used as input for robot_state_publisher'
        ),
        robot_state_publisher_node
    ])