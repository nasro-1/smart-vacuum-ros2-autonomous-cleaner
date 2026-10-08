#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash

ros2 run cartographer_ros cartographer_node \
  -configuration_directory /home/lxmix \
  -configuration_basename cartographer.lua \
  --ros-args -p use_sim_time:=true &
sleep 3

ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args -p use_sim_time:=true -p resolution:=0.05
