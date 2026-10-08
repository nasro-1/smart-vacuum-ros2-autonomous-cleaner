#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Bridge Gazebo topics to ROS2 - use the correct TF topic
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /model/smart_vacuum/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  --ros-args -p use_sim_time:=true -r /model/smart_vacuum/tf:=/tf &
sleep 3

# Connect base_link to lidar frame
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.065 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id "smart_vacuum/base_link/lidar" \
  --ros-args -p use_sim_time:=true &
sleep 2

echo "All bridges started. Starting SLAM..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/lxmix/slam_params.yaml \
  use_sim_time:=true
