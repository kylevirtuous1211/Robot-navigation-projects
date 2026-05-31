from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pros_car_py",
                executable="arm_writer",
                name="arm_writer",
                output="screen",
            ),
            Node(
                package="arm_control_pkg",
                executable="arm_control_node",
                name="arm_control_node",
                output="screen",
            ),
            Node(
                package="arm_control_pkg",
                executable="unity_arm_republish_node",
                name="unity_arm_republish_node",
                output="screen",
            ),
        ]
    )
