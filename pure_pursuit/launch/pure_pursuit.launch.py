from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
    )

    return LaunchDescription(
        [
            pure_pursuit_node
        ]
    )
