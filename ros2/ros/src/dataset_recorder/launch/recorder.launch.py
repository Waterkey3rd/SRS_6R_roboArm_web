from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value="/home/rm/comm_ws/datasets",
            description="Root directory for dataset sessions",
        ),
        Node(
            package="dataset_recorder",
            executable="recorder_node",
            name="dataset_recorder",
            output="screen",
            parameters=[{
                "output_dir": LaunchConfiguration("output_dir"),
            }],
        ),
    ])
