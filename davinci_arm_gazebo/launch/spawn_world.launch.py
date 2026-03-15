#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    # Gazebo launch path:
    gz_launch_path = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    ])
    
    # World file path:
    world_file_path = PathJoinSubstitution([
        FindPackageShare("davinci_arm_gazebo"),
        "worlds",
        "main_world.sdf"
    ])

    # GUI config file path:
    gui_config_path = PathJoinSubstitution([
        FindPackageShare("davinci_arm_gazebo"),
        "config",
        "scenery.config"
    ])
    
    # Update all Gazebo model paths:
    gazebo_models_path = os.path.join(
        get_package_share_directory("davinci_arm_gazebo"),
        "models"
    )
    
    # Paths for robot model and plugins:
    install_dir_model = get_package_prefix("davinci_arm_description")
    install_dir_resources = get_package_prefix("davinci_arm_gazebo_plugins")
    install_dir_plugins = get_package_prefix("davinci_arm_gazebo_plugins")

    # Environment variables configuration:
    env_vars = [
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=(
                os.environ.get('GAZEBO_MODEL_PATH', '') + ':' +
                install_dir_model + "/share" + ':' + gazebo_models_path
            )
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=(
                os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' +
                install_dir_model + "/share" + ':' + gazebo_models_path
            )
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=(
                os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' +
                install_dir_plugins + '/lib'
            )
        )
    ]
    
    # Gazebo launch description:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [
                '-r -v 4 ',
                world_file_path,
                ' --gui-config ',
                gui_config_path
            ]
        }.items(),
    )
    
    return LaunchDescription(
        env_vars + [
            gazebo,
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description='If true, use simulated clock'
            ),
        ]
    )