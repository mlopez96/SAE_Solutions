import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    go_to_waypoint_node = Node(
        package="capstone_project",
        executable="go_to_waypoint",
        name="go_to_waypoint",
        output="screen",
        )
    
    return LaunchDescription(
        [
            go_to_waypoint_node,
        ]
    )
