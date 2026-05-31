#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 啟動 car_manual_node（非互動節點）
            Node(
                package="car_control_pkg",
                executable="car_manual_node",
                name="car_manual_node",
                output="screen",
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
