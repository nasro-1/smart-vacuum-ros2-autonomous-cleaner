#!/usr/bin/env python3
"""
smart_vacuum.launch.py  —  updated launch for dirt_cleaner
Changes vs original:
  - slam_toolbox added  (online async SLAM — builds map without prior)
  - autonomous_explorer replaced by coverage_explorer
  - coverage_explorer starts at t=13 s (after SLAM has initialised)
  - my_vacuum_cpp removed (was empty, now deleted)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Dynamic paths
# ---------------------------------------------------------------------------
WORKSPACE = os.path.normpath(os.path.join(
    get_package_share_directory('dirt_cleaner'),
    "..", "..", "..", "..",
))

TB3_GAZEBO  = get_package_share_directory('turtlebot3_gazebo')
ROS_GZ_SIM  = get_package_share_directory('ros_gz_sim')
SLAM_TB     = get_package_share_directory('slam_toolbox')


def generate_launch_description():

    weights_default = os.path.join(
        WORKSPACE, "models", "dirt_yolo", "weights", "best.pt"
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose',       default='0.0')
    y_pose       = LaunchConfiguration('y_pose',        default='0.0')

    world = os.path.join(WORKSPACE, "worlds", "my_house.sdf")
    if not os.path.exists(world):
        world = os.path.join(
            get_package_share_directory('dirt_cleaner'), "worlds", "my_house.sdf"
        )

    # ── Gazebo ──────────────────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ROS_GZ_SIM, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ROS_GZ_SIM, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Robot ────────────────────────────────────────────────────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(TB3_GAZEBO, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(TB3_GAZEBO, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(TB3_GAZEBO, 'models')
    )

    # ── SLAM toolbox  (starts immediately — needs laser before map appears) ─
    # online_async = builds map live without any prior map file
    slam_toolbox = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SLAM_TB, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    # ── Perception nodes (unchanged) ─────────────────────────────────────────
    semantic_detector = TimerAction(
        period=10.0,
        actions=[Node(
            package="dirt_cleaner",
            executable="semantic_detector",
            output="screen",
        )]
    )

    dirt_detector = TimerAction(
        period=10.0,
        actions=[Node(
            package="dirt_cleaner",
            executable="dirt_detector_node",
            parameters=[{"weights": weights_default}],
            output="screen",
        )]
    )

    # ── NEW: coverage explorer  (replaces autonomous_explorer) ───────────────
    coverage_explorer = TimerAction(
        period=13.0,
        actions=[Node(
            package="dirt_cleaner",
            executable="coverage_explorer",
            output="screen",
        )]
    )

    # ── Assemble ─────────────────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(set_gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    ld.add_action(slam_toolbox)          
    ld.add_action(semantic_detector)
    ld.add_action(dirt_detector)
    ld.add_action(coverage_explorer)    
    return ld