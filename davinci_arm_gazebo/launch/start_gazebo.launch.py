#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch arguments:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    launch_gui_arg = DeclareLaunchArgument(
        'launch_gui', default_value='true',
        description='Launch the PropArm GUI interface'
    )
    launch_gui = LaunchConfiguration('launch_gui')

    gui_delay_arg = DeclareLaunchArgument(
        'gui_delay', default_value='8.0',
        description='Delay before launching GUI (seconds)'
    )

    # Find your packages:
    pkg_gz   = FindPackageShare("davinci_arm_gazebo")
    pkg_desc = FindPackageShare("davinci_arm_description")
    pkg_bridge = FindPackageShare("davinci_arm_gazebo_bridge")

    # Launch files:
    gazebo_launch     = PathJoinSubstitution([pkg_gz,   "launch", "spawn_world.launch.py"])
    urdf_launch       = PathJoinSubstitution([pkg_desc, "launch", "publish_urdf.launch.py"])
    spawn_launch      = PathJoinSubstitution([pkg_gz,   "launch", "spawn_robot.launch.py"])
    spawn_models_launch = PathJoinSubstitution([pkg_gz, "launch", "spawn_models.launch.py"])
    spawn_gazebo_bridge = PathJoinSubstitution([pkg_bridge, "launch", "gazebo_bridge.launch.py"])

    return LaunchDescription([
        use_sim_time_arg,
        launch_gui_arg,
        gui_delay_arg,

        # 1. Start Gazebo:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 2. Publish URDF:
        TimerAction(
            period=3.0,
            actions=[ IncludeLaunchDescription(
                PythonLaunchDescriptionSource(urdf_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ) ]
        ),

        # 4. Spawn the robot in Gazebo:
        TimerAction(
            period=5.0,
            actions=[ IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ) ]
        ),
        
        #5. Spawn object models in Gazebo:
        TimerAction(
            period=7.0,
            actions=[ IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_models_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ) ]
        ),       

        #6. Spawn gazebo and ROS2 bridge:
        TimerAction(
            period = 8.0,
            actions=[ IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_gazebo_bridge),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )]
        )
    ])