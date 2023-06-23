from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    wall_following_node = Node(
        package="wall_following",
        executable="sae_wall_following",
        name="sae_wall_following",
    )

    return LaunchDescription(
        [
            wall_following_node
        ]
    )
