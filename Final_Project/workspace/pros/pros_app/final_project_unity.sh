#!/bin/bash
#
# Final Project — 線上 SLAM + Nav2 一鍵啟動
# ==========================================
# 同時啟動：
#   robot_unity      : Unity 機器人橋接 (相機 / TF / 輪子 / 手臂 topic)
#   slam_unity       : slam_toolbox 線上建圖 (即時 /map + map->base_footprint TF)
#   navigation_unity : Nav2 規劃器與 costmap (global costmap 直接吃 slam 的 /map)
#
# 這讓 costmap 隨車子探索「線上長出來」，不需先 store_map，
# 適合每次都隨機生成的 Final Project 地圖。
#
# 注意：navigation_unity.xml 會把 map 參數傳給 nav2_bringup 的 navigation_launch.py。
# 若該 launch 不接受 map 參數而報錯，請把 demo/navigation_unity.xml 內
# <arg name="map" .../> 那行移除 (navigation_launch 不需要 map server，
# costmap 只要訂閱 slam 發出的 /map topic 即可)。

source "./utils.sh"
main "./docker/compose/docker-compose_robot_unity.yml" \
     "./docker/compose/docker-compose_slam_unity.yml" \
     "./docker/compose/docker-compose_navigation_unity.yml"
