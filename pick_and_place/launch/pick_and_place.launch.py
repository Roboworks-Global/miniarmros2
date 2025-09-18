import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Get the launch directory
    astra_dir = get_package_share_directory('astra_camera')

    astra_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(astra_dir, 'launch/gemini.launch.py')),
    )
                              
    colour_detector = launch_ros.actions.Node(
        package='pick_and_place', 
        executable='colour_detector', 
        name='colour_detector',
        parameters=[{'use_sim_time': True}]
    )

    arm_tester = launch_ros.actions.Node(
        package='pick_and_place', 
        executable='arm_tester', 
        name='arm_tester',
    )
    
    nav2_tester = launch_ros.actions.Node(
        package='pick_and_place', 
        executable='nav2_tester', 
        name='nav2_tester',
    )

    ld = LaunchDescription()

    ld.add_action(astra_node)
    ld.add_action(colour_detector)

    return ld

