from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("davinci_arm_characterization"), "config", "joint_tf_visualizer.yaml"]
        ),
        description="Path to the parameter file for the joint TF visualizer.",
    )

    fixed_frame_arg = DeclareLaunchArgument(
        "fixed_frame",
        default_value="world",
        description="Fixed frame used by the marker overlay.",
    )

    robot_description_node_arg = DeclareLaunchArgument(
        "robot_description_node",
        default_value="davinci_arm_state_publisher",
        description="Node that owns the robot_description parameter.",
    )

    return LaunchDescription(
        [
            config_arg,
            fixed_frame_arg,
            robot_description_node_arg,
            Node(
                package="davinci_arm_characterization",
                executable="joint_tf_visualizer_node",
                name="joint_tf_visualizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "fixed_frame": LaunchConfiguration("fixed_frame"),
                        "robot_description_node": LaunchConfiguration("robot_description_node"),
                    },
                ],
            ),
        ]
    )
