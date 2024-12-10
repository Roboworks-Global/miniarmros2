from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get package directories
    moveit_config_pkg = get_package_share_directory('stepper_arm_moveit_config')
    robot_pkg = get_package_share_directory('stepper_arm')

    # Declare arguments
    load_robot_description = DeclareLaunchArgument(
        'load_robot_description',
        default_value='false',
        description='Whether to load robot description'
    )

    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value='robot_description',
        description='Name of robot description parameter'
    )

    # Load YAML files
    with open(os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml'), 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)

    with open(os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml'), 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    # Create robot description parameter
    robot_description = {
        'robot_description': {
            'type': 'yaml',
            'params': {'robot_description': LaunchConfiguration('robot_description')}
        } if LaunchConfiguration('load_robot_description') else None
    }

    # Create SRDF parameter
    robot_description_semantic = {
        'robot_description_semantic': {
            'type': 'file',
            'content': os.path.join(moveit_config_pkg, 'config', 'stepper_arm.srdf'),
        }
    }

    # Parameters node
    param_node = Node(
        package='moveit_ros_planning',
        executable='moveit_params',
        name='moveit_params',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_planning': joint_limits_yaml},
            {'robot_description_kinematics': kinematics_yaml}
        ],
    )

    return LaunchDescription([
        # Arguments
        load_robot_description,
        robot_description_arg,
        
        # Nodes
        param_node,
    ])