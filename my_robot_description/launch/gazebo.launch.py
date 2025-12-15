# my_robot_description/launch/gazebo.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')

    # Paths to files
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf.xacro')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    bridge_config_path = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    default_world_path = os.path.join(pkg_dir, 'worlds', 'my_world.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world_path)

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world file to load'
    )
    
    # Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # Robot Description
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
        output='screen'
    )

    # Spawn Robot
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot'],
        output='screen'
    )
    
    # EKF Node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Bridge
    gz_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config_path, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Teleop Keyboard
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    # Delay other nodes until the robot is spawned
    delay_nodes_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                robot_localization_node,
                gz_ros_bridge_node,
                teleop_keyboard_node
            ]
        )
    )

    return LaunchDescription([
        declare_world_arg,
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_robot_node,
        delay_nodes_handler
    ])