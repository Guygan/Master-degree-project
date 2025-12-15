# slam.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    my_robot_pkg_dir = get_package_share_directory('my_robot_description')
    
    urdf_path = os.path.join(my_robot_pkg_dir, 'urdf', 'my_robot.urdf.xacro')
    bridge_config_path = os.path.join(my_robot_pkg_dir, 'config', 'gz_bridge.yaml')
    rviz_config_path = os.path.join(my_robot_pkg_dir, 'rviz', 'slam_view.rviz')
    ekf_config_path = os.path.join(my_robot_pkg_dir, 'config', 'ekf.yaml')
    slam_params_path = os.path.join(my_robot_pkg_dir, 'config', 'slam_params.yaml')

    # ใช้ simulation time (จาก Gazebo)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ✅ Wrap ด้วย ParameterValue เพื่อแก้ error "Unable to parse"
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content
        }]
    )

    # Spawn robot into Gazebo
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot'],
        output='screen'
    )

    # EKF
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ROS ↔ Gazebo bridge
    gz_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config_path, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Frame fixer
    frame_fixer_node = Node(
        package='my_robot_description',
        executable='frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Teleop keyboard
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ✅ Slam toolbox (ใช้ IncludeLaunchDescription เรียก online_async_launch.py โดยตรง)
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_path
        }.items()
    )

    # Delay หลัง spawn robot เพื่อให้ odom/scan เริ่มก่อน แล้วค่อยรัน slam_toolbox
    delay_nodes_after_spawn_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                LogInfo(msg='✅ Robot spawned. Launching bridge, EKF, frame fixer, RViz...'),
                gz_ros_bridge_node,
                robot_localization_node,
                frame_fixer_node,
                rviz_node,
                teleop_keyboard_node,
                TimerAction(
                    period=6.0,
                    actions=[
                        LogInfo(msg='✅ Delay complete. Launching SLAM Toolbox...'),
                        start_slam_toolbox,
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_robot_node,
        delay_nodes_after_spawn_handler,
    ])
