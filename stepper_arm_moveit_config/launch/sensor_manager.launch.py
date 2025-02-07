from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('stepper_arm_moveit_config')

    # Load the sensors_3d.yaml file
    with open(os.path.join(pkg_share, 'config', 'sensors_3d.yaml'), 'r') as f:
        sensors_3d_config = yaml.safe_load(f)

    # Declare arguments
    moveit_sensor_manager = DeclareLaunchArgument(
        'moveit_sensor_manager',
        default_value='stepper_arm',
        description='Robot specific sensor manager'
    )

    # Parameters node
    params_node = Node(
        package='moveit_ros_planning',
        executable='moveit_params',
        name='sensor_manager_params',
        output='screen',
        parameters=[
            sensors_3d_config,
            {
                # Octomap parameters
                'octomap_resolution': 0.025,
                'max_range': 5.0,
                # Uncomment and modify if needed:
                # 'octomap_frame': 'some_frame'
            }
        ]
    )

    # Include sensor manager launch file
    sensor_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch',
            PathJoinSubstitution([
                LaunchConfiguration('moveit_sensor_manager'),
                '_moveit_sensor_manager.launch.py'
            ]))
        ])
    )

    return LaunchDescription([
        # Arguments
        moveit_sensor_manager,
        
        # Nodes
        params_node,
        
        # Includes
        sensor_manager_launch,
    ])