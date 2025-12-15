# In file: launch/robot_localization.launch.py


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # --- File Paths ---
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf.xacro')
    bridge_config_path = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'my_nav2_view.rviz')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # --- Launch Arguments ---
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'slam_world.sdf'),
        description='Full path to the world file to load'
    )
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'my_slam_map.yaml'),
        description='Full path to map file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # --- Gazebo Simulation ---
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r ', world]}.items() 
    )

    # --- Core Robot Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '0.0'
                   ],
        output='screen'
    )

    # --- Bridge, EKF, Frame Fixer Nodes ---
    gz_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config_path, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    frame_fixer_node = Node(
        package='my_robot_description',
        executable='frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- [✨✨✨ นี่คือจุดแก้ไข Refactor ✨✨✨] ---
    # โหนดตัวเก่า (จะไม่ได้ถูกเรียกใช้)
    # interactive_stuck_detector_node = Node(
    #     package='my_robot_description',
    #     executable='interactive_stuck_detector',
    #     name='interactive_stuck_detector',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # โหนด "สมอง" ตัวใหม่
    stuck_manager_node = Node(
        package='my_robot_description',
        executable='stuck_manager_node',
        name='stuck_manager_node',
        output='screen',
        emulate_tty=True, # (เผื่อไว้)
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # โหนด "UI" ตัวใหม่
    stuck_ui_node = Node(
        package='my_robot_description',
        executable='stuck_ui_node',
        name='stuck_ui_node',
        output='screen',
        emulate_tty=True, # (สำคัญสำหรับ Zenity)
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    pause_mode_node = Node(
        package='my_robot_description',
        executable='pause_mode_node',
        name='pause_mode_node',
        output='screen',
        emulate_tty=True,
    )
    return_to_home_node = Node(
        package='my_robot_description',
        executable='return_to_home_node',
        name='return_to_home_node',
        output='screen',
        emulate_tty=True,
    )
    
    go_to_checkpoint_node = Node(
        package='my_robot_description',
        executable='go_to_checkpoint_node',
        name='go_to_checkpoint_node',
        output='screen',
        emulate_tty=True,
    )

    goal_monitor_node = Node( 
        package='my_robot_description',
        executable='goal_monitor_node',
        name='goal_monitor_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    laser_to_sonar_node = Node(
        package='my_robot_description',
        executable='laser_to_sonar_node',
        name='laser_to_sonar_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Nav2 Stack ---
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'map': map_yaml_file,
            'autostart': 'true',
            'use_rviz': 'false',
            'launch_collision_monitor': 'false',
            'launch_velocity_smoother': 'true',
    'launch_docking_server': 'true',
            'launch_smoother_server': 'false',
            'launch_waypoint_follower': 'true'
        }.items() 
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- ลำดับการรัน ---
    start_bridge_handler = TimerAction(period=2.0, actions=[
        LogInfo(msg='Gazebo ready, starting Bridge...'),
        gz_ros_bridge_node,
        TimerAction(period=1.0, actions=[
             LogInfo(msg='Bridge ready, starting EKF & Frame Fixer...'),
             robot_localization_node,
             frame_fixer_node,
        ])
    ])

    start_nav2_rviz_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_localization_node,
            on_start=[
                LogInfo(msg='EKF started, starting Nav2, RViz, and ALL Helper Nodes...'),
                start_nav2, 
                rviz_node,
                
                stuck_manager_node,                
                stuck_ui_node,                    
                
                goal_monitor_node, 
                laser_to_sonar_node,
                pause_mode_node,
                return_to_home_node,
                
                go_to_checkpoint_node
            ]
        )
    )

    return LaunchDescription([
        declare_world_arg,
        declare_map_arg,
        declare_use_sim_time_cmd,
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_robot_node,
        start_bridge_handler,
        start_nav2_rviz_handler
    ])