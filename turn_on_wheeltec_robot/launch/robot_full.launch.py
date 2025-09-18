import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir,'launch')

    rplidar_dir = get_package_share_directory('lidar_ros2')
    rplidar_launch_dir = os.path.join(rplidar_dir, 'launch')
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    Astra_S = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'astra_mini.launch.py')),)

    Astra_Pro = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'astra_pro.launch.py')),)

    Dabai = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'dabai.launch.py')),)

    Gemini = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'gemini.launch.py')),)

    wheeltec_robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),)
    
    rplidar_ros = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),)

    # Create the launch description and populate
    ld = LaunchDescription()
    
    #Select your camera here, options include:
    #Astra_S、Astra_Pro、Dabai、Gemini
    ld.add_action(Gemini)
    ld.add_action(wheeltec_robot)
    ld.add_action(rplidar_ros)

    return ld