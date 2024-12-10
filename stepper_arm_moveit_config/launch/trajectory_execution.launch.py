from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('mini_mec_four_arm_moveit_config')

    # Declare arguments
    moveit_manage_controllers = DeclareLaunchArgument(
        'moveit_manage_controllers',
        default_value='true',
        description='Flag indicating whether MoveIt is allowed to load/unload or switch controllers'
    )

    moveit_controller_manager = DeclareLaunchArgument(
        'moveit_controller_manager',
        default_value='mini_mec_four_arm',
        description='Robot specific controller manager'
    )

    # Parameters node
    params_node = Node(
        package='moveit_ros_planning',
        executable='moveit_params',
        name='trajectory_execution_params',
        output='screen',
        parameters=[{
            'moveit_manage_controllers': LaunchConfiguration('moveit_manage_controllers'),
            'trajectory_execution': {
                'allowed_execution_duration_scaling': 1.2,  # default 1.2
                'allowed_goal_duration_margin': 0.5,        # default 0.5
                'allowed_start_tolerance': 0.01            # default 0.01
            }
        }]
    )

    # Include controller manager launch file
    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch',
            PathJoinSubstitution([
                LaunchConfiguration('moveit_controller_manager'),
                '_moveit_controller_manager.launch.py'
            ]))
        ])
    )

    return LaunchDescription([
        # Arguments
        moveit_manage_controllers,
        moveit_controller_manager,
        
        # Nodes
        params_node,
        
        # Includes
        controller_manager_launch,
    ])