from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_name = "stepper_arm_moveit_config"
    pkg_share = get_package_share_directory(pkg_name)

    # Declare all launch arguments
    debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Debug flag'
    )

    info = DeclareLaunchArgument(
        'info',
        default_value=LaunchConfiguration('debug'),
        description='Info mode flag'
    )

    pipeline = DeclareLaunchArgument(
        'pipeline',
        default_value='ompl',
        description='Planning pipeline'
    )

    allow_trajectory_execution = DeclareLaunchArgument(
        'allow_trajectory_execution',
        default_value='true',
        description='Allow trajectory execution'
    )

    fake_execution = DeclareLaunchArgument(
        'fake_execution',
        default_value='false',
        description='Fake execution flag'
    )

    max_safe_path_cost = DeclareLaunchArgument(
        'max_safe_path_cost',
        default_value='1',
        description='Max safe path cost'
    )

    jiggle_fraction = DeclareLaunchArgument(
        'jiggle_fraction',
        default_value='0.05',
        description='Jiggle fraction'
    )

    publish_monitored_planning_scene = DeclareLaunchArgument(
        'publish_monitored_planning_scene',
        default_value='true',
        description='Publish monitored planning scene'
    )

    capabilities = DeclareLaunchArgument(
        'capabilities',
        default_value='',
        description='Additional capabilities'
    )

    disable_capabilities = DeclareLaunchArgument(
        'disable_capabilities',
        default_value='',
        description='Capabilities to disable'
    )

    load_robot_description = DeclareLaunchArgument(
        'load_robot_description',
        default_value='true',
        description='Load robot description flag'
    )

    # Include the planning context launch file
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'planning_context.launch.py')
        ]),
        launch_arguments={'load_robot_description': LaunchConfiguration('load_robot_description')}.items()
    )

    # Include the planning pipeline launch file
    planning_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'planning_pipeline.launch.py')
        ]),
        launch_arguments={'pipeline': LaunchConfiguration('pipeline')}.items()
    )

    # Include the trajectory execution launch file
    trajectory_execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'trajectory_execution.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('allow_trajectory_execution')),
        launch_arguments={
            'moveit_manage_controllers': 'true',
            'moveit_controller_manager': PythonExpression([
                "'fake' if '", LaunchConfiguration('fake_execution'), "' else 'stepper_arm'"
            ])
        }.items()
    )

    # Include the sensor manager launch file
    sensor_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'sensor_manager.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('allow_trajectory_execution')),
        launch_arguments={'moveit_sensor_manager': 'stepper_arm'}.items()
    )

    # Start move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('debug')),
        parameters=[{
            'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
            'max_safe_path_cost': LaunchConfiguration('max_safe_path_cost'),
            'jiggle_fraction': LaunchConfiguration('jiggle_fraction'),
            'capabilities': LaunchConfiguration('capabilities'),
            'disable_capabilities': LaunchConfiguration('disable_capabilities'),
            'planning_scene_monitor/publish_planning_scene': LaunchConfiguration('publish_monitored_planning_scene'),
            'planning_scene_monitor/publish_geometry_updates': LaunchConfiguration('publish_monitored_planning_scene'),
            'planning_scene_monitor/publish_state_updates': LaunchConfiguration('publish_monitored_planning_scene'),
            'planning_scene_monitor/publish_transforms_updates': LaunchConfiguration('publish_monitored_planning_scene')
        }],
        remappings=[],
        arguments=['--ros-args', '--log-level', 'info'],
        prefix=['gdb -x ' + os.path.join(pkg_share, 'launch', 'gdb_settings.gdb') + ' --ex run --args' 
                if LaunchConfiguration('debug') else '']
    )

    # Create launch description
    return LaunchDescription([
        # Launch arguments
        debug,
        info,
        pipeline,
        allow_trajectory_execution,
        fake_execution,
        max_safe_path_cost,
        jiggle_fraction,
        publish_monitored_planning_scene,
        capabilities,
        disable_capabilities,
        load_robot_description,
        
        # Include launch files
        planning_context,
        planning_pipeline,
        trajectory_execution,
        sensor_manager,
        
        # Nodes
        move_group_node,
    ])