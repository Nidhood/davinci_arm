#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    allow_trajectory_execution = LaunchConfiguration('allow_trajectory_execution')
    publish_monitored_planning_scene = LaunchConfiguration('publish_monitored_planning_scene')
    capabilities = LaunchConfiguration('capabilities')
    disable_capabilities = LaunchConfiguration('disable_capabilities')
    monitor_dynamics = LaunchConfiguration('monitor_dynamics')

    moveit_config = MoveItConfigsBuilder(
        "davinci_arm",
        package_name="davinci_arm_moveit_config"
    ).to_moveit_configs()

    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': allow_trajectory_execution,
        'capabilities': ParameterValue(capabilities, value_type=str),
        'disable_capabilities': ParameterValue(disable_capabilities, value_type=str),
        'publish_planning_scene': publish_monitored_planning_scene,
        'publish_geometry_updates': publish_monitored_planning_scene,
        'publish_state_updates': publish_monitored_planning_scene,
        'publish_transforms_updates': publish_monitored_planning_scene,
        'monitor_dynamics': monitor_dynamics,
        'use_sim_time': use_sim_time,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'allow_trajectory_execution',
            default_value='true',
            description='Allow trajectory execution'
        ),
        DeclareLaunchArgument(
            'publish_monitored_planning_scene',
            default_value='true',
            description='Publish planning scene updates'
        ),
        DeclareLaunchArgument(
            'capabilities',
            default_value=moveit_config.move_group_capabilities['capabilities'],
            description='Additional move_group capabilities'
        ),
        DeclareLaunchArgument(
            'disable_capabilities',
            default_value=moveit_config.move_group_capabilities['disable_capabilities'],
            description='Capabilities to disable'
        ),
        DeclareLaunchArgument(
            'monitor_dynamics',
            default_value='false',
            description='Monitor dynamics from joint states'
        ),
        move_group_node,
    ])