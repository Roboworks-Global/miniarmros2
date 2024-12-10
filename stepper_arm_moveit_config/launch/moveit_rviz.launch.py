from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Debug flag'
    )

    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Path to RViz config file'
    )

    # Create RViz node with conditional arguments
    rviz_node = Node(
        package='rviz2',  # Note: 'rviz' becomes 'rviz2' in ROS 2
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=UnlessCondition(LaunchConfiguration('rviz_config') == ''),
        prefix=['gdb --ex run --args' if LaunchConfiguration('debug') else '']
    )

    # Alternative RViz node without config file
    rviz_node_no_config = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz_config') == ''),
        prefix=['gdb --ex run --args' if LaunchConfiguration('debug') else '']
    )

    return LaunchDescription([
        # Arguments
        debug,
        rviz_config,
        
        # Nodes
        rviz_node,
        rviz_node_no_config,
    ])