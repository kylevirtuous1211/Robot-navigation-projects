#!/bin/bash

source "./utils.sh"
main "./docker/compose/docker-compose_lidar_pkg.yml" "./docker/compose/docker-compose_oradarlidar.yml" "./docker/compose/docker-compose_slam_oradarlidar.yml"
