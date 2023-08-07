from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lane_keeping_node = Node(
        package="lane_keeping",
        executable="simple_lane_keep",
        name="simple_lane_keep",
    )
    return LaunchDescription(
        [
            lane_keeping_node
        ]
    )
