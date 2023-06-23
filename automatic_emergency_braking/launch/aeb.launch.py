from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

# Define node and return the Launch Description
    aeb_node = Node(
        package="automatic_emergency_braking",
        executable="sae_aeb",
        name="sae_aeb",
    )
    return LaunchDescription(
        [
            aeb_node 
        ]
    )
