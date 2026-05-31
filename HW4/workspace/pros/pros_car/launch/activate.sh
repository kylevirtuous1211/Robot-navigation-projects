#!/bin/bash
# 先啟動車輛控制相關的 launch 檔在背景
ros2 launch ./car_control_launch.launch.py &
sleep 2
# 接著在目前終端啟動鍵盤互動節點
ros2 run keyboard_mode_interface_pkg keyboard_control_node
