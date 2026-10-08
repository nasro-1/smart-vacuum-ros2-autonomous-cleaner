from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', ''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('turtlebot3_gazebo'),
                '/launch/turtlebot3_world.launch.py'
            ]),
        ),
    ])
