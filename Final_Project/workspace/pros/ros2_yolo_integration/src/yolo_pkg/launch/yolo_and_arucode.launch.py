from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arucode_pkg',
            executable='arucode_node',
            name='arucode_node',
            output='screen'
        ),
        Node(
            package='yolo_pkg',
            executable='yolo_detection_node',
            name='yolo_detection_node',
            output='screen'
        )
    ])
