import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    wall_following_node = Node(
        package="capstone_project",
        executable="wall_follower",
        name="wall_follower",
        output="screen",
        )
    
    line_following_node = Node(
        package="capstone_project",
        executable="line_follower",
        name="line_follower",
        output="screen",
        )
    
    pure_pursuit_node = Node(
        package="capstone_project",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        )
        
    state_machine = Node(
        package="capstone_project",
        executable="state_machine",
        name="state_machine",
        output="screen",
        )
    
    classify_resnet50 = Node(
        package="capstone_project",
        executable="classify_resnet50",
        name="classify_resnet50",
        output="screen",
        )
    
    go_to_waypoint_node = Node(
        package="capstone_project",
        executable="go_to_waypoint",
        name="go_to_waypoint",
        output="screen",
    )
       
       
    return LaunchDescription(
        [
            wall_following_node,
            line_following_node,
            pure_pursuit_node,
            state_machine,
            classify_resnet50,
            go_to_waypoint_node,
        ]
    )
