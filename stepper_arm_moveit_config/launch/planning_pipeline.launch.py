from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('stepper_arm_moveit_config')

    # Declare arguments
    pipeline = DeclareLaunchArgument(
        'pipeline',
        default_value='ompl',
        description='Planning pipeline to use'
    )

    # Include the specific planning pipeline launch file
    planning_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 
            PathJoinSubstitution([LaunchConfiguration('pipeline'), '_planning_pipeline.launch.py']))
        ])
    )

    return LaunchDescription([
        pipeline,
        planning_pipeline_launch
    ])