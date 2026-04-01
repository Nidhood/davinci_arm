#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_desc = FindPackageShare('davinci_arm_description')

    publish_urdf_launch = PathJoinSubstitution([
        pkg_desc, 'launch', 'publish_urdf.launch.py'
    ])

    display_model_launch = PathJoinSubstitution([
        pkg_desc, 'launch', 'display_model.launch.py'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(publish_urdf_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'launch_state_publisher': 'true',
                'joint_states_topic': '/joint_states'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_model_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'use_joint_state_gui': 'true'
            }.items()
        )
    ])