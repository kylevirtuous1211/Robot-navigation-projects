#!/bin/bash

source "./utils.sh"
main "./docker/compose/docker-compose_robot_unity.yml" "./docker/compose/docker-compose_slam_unity.yml"
