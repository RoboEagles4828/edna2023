from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'port': 8765,
        }],
    )

    return LaunchDescription([
        foxglove
    ])