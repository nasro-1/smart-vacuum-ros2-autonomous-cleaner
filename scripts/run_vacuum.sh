#!/bin/bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models:~/.gazebo/models
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning up old processes ==="
pkill -9 -f "gz sim"; pkill -9 -f static_transform_publisher
pkill -9 -f ros_gz_bridge; pkill -9 -f cartographer
pkill -9 -f slam_toolbox; pkill -9 -f rviz2
pkill -9 -f autonomous_explorer
pkill -9 -f dirt_detector_node.py
pkill -9 -f "ros2 run dirt_cleaner semantic_detector"
pkill -9 -f "ros2 run dirt_cleaner behavior_manager"
sleep 3

echo "=== Starting Gazebo ==="
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models:~/.gazebo/models
gz sim -r -s -v2 /home/lxmix/turtlebot3_ws/worlds/my_house.sdf --force-version 8 &
sleep 8

echo "=== Spawning robot ==="
gz service -s /world/my_house/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/home/lxmix/turtlebot3_ws/my_models/smart_vacuum/model.sdf", name: "smart_vacuum", pose: {position: {x: 0, y: 0, z: 0.1}}'
sleep 3

echo "=== Starting bridges ==="
source ~/turtlebot3_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /model/smart_vacuum/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  --ros-args -p use_sim_time:=true \
  -r /model/smart_vacuum/tf:=/tf &
sleep 3

echo "=== Starting static TF ==="
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.065 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id "smart_vacuum/base_link/lidar" \
  --ros-args -p use_sim_time:=true &
sleep 2

echo "=== Starting Cartographer ==="
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /home/lxmix/turtlebot3_ws \
  -configuration_basename cartographer.lua \
  --ros-args -p use_sim_time:=true &
sleep 3
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args -p use_sim_time:=true -p resolution:=0.05 &
sleep 5

echo "=== Starting Gazebo GUI ==="
env -i HOME=$HOME DISPLAY=:1 PATH=/usr/bin:/bin \
  /usr/bin/gz sim -g --force-version 8 &
sleep 3

echo "=== Starting RViz ==="
RVIZ_LOG="$HOME/run_vacuum_rviz.log"
SEMANTIC_LOG="$HOME/run_vacuum_semantic.log"
echo "RViz logs will be written to $RVIZ_LOG"
echo "Semantic detector logs will be written to $SEMANTIC_LOG"
env -i HOME="$HOME" DISPLAY=:1 XDG_RUNTIME_DIR=/run/user/1000 PATH=/opt/ros/jazzy/bin:/usr/bin:/bin RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  /bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/lxmix/turtlebot3_ws/install/setup.bash && exec /opt/ros/jazzy/bin/rviz2 -d /home/lxmix/turtlebot3_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/rviz/tb3_cartographer.rviz' > "$RVIZ_LOG" 2>&1 &
sleep 3



echo "=== Starting semantic detector ==="
ros2 run dirt_cleaner semantic_detector > "$SEMANTIC_LOG" 2>&1 &
sleep 2

echo "=== Starting behavior manager ==="
ros2 run dirt_cleaner behavior_manager &
sleep 2

echo "=== Starting autonomous explorer ==="
python3 ~/turtlebot3_ws/autonomous_explorer.py
