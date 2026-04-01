#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_desc = FindPackageShare('davinci_arm_description')

    display_model_launch = PathJoinSubstitution([
        pkg_desc, 'launch', 'display_model.launch.py'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_model_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'use_joint_state_gui': 'false'
            }.items()
        )
    ])