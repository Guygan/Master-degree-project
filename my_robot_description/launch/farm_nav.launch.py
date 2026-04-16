import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    AppendEnvironmentVariable, 
    RegisterEventHandler, 
    LogInfo
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # =========================================================================
    # 1. SETUP PATHS & CONFIGURATION
    # =========================================================================
    pkg_name = 'my_robot_description'
    pkg_path = get_package_share_directory(pkg_name)
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    
    # Define file paths
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    map_file = os.path.join(pkg_path, 'maps', 'farm2.yaml') 
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params_gps.yaml')
    bridge_params_file = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'my_nav2_view.rviz')
    world_file = os.path.join(pkg_path, 'worlds', 'test.sdf')
    
    # --- Config สำหรับระบบ GPS ---
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf_gps.yaml')
    navsat_config_file = os.path.join(pkg_path, 'config', 'navsat_transform_params.yaml') # [✨ ใหม่]
    
    # Config ของ Mapviz
    mapviz_config_file = os.path.join(pkg_path, 'config', 'my_mapviz_config.mvc')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Environment Variables (สำหรับ Gazebo)
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', 
        os.path.join(pkg_path, '..')
    )

    # Robot Description (Xacro -> URDF)
    robot_desc = ParameterValue(
        Command(['xacro ', urdf_file]), 
        value_type=str
    )

    # =========================================================================
    # 2. CORE SIMULATION NODES (Gazebo & TF)
    # =========================================================================
    
    robot_state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        output='both', 
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_desc
        }]
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim', 
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'my_bot',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.1', 
            '-Y', '0.0'
        ]
    )

    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_params_file, 
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }]
    )

    frame_fixer = Node(
        package='my_robot_description', 
        executable='frame_fixer', 
        name='frame_fixer', 
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # =========================================================================
    # 3. LOCALIZATION NODES (Dual EKF + NavSat) - [✨ MAJOR UPDATE]
    # =========================================================================
    
    # 3.1 Local EKF (คำนวณ Odom -> Base_link)
    # ใช้ข้อมูลจากล้อและ IMU เพื่อความต่อเนื่อง (Continuous)
    ekf_local_node = Node(
       package='robot_localization', 
       executable='ekf_node', 
       name='ekf_filter_node_local',
       output='screen', 
       parameters=[
           ekf_config_file, 
           {'use_sim_time': use_sim_time}
       ],
       remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 3.2 Navsat Transform (แปลง GPS Lat/Lon -> X,Y Coordinates)
    # รับค่า GPS ดิบ แล้วแปลงเป็นพิกัดเมตรให้ EKF Global
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('gps/fix', '/gps/fix'),
            ('imu/data', '/imu'),
            ('odometry/filtered', 'odometry/global'), # รับค่าจาก Global EKF มาอ้างอิง
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps')          # ส่งค่าที่แปลงแล้วออกไป
        ]
    )

    # 3.3 Global EKF (คำนวณ Map -> Odom)
    # รับค่าจาก NavSat (odometry/gps) มาหลอมรวม เพื่อระบุตำแหน่งบนโลกจริง
    ekf_global_node = Node(
       package='robot_localization', 
       executable='ekf_node', 
       name='ekf_filter_node_global',
       output='screen', 
       parameters=[
           ekf_config_file, 
           {'use_sim_time': use_sim_time}
       ],
       remappings=[('odometry/filtered', 'odometry/global')]
    )

    
    # 4.1 Map Server
    map_server = Node(
        package='nav2_map_server', 
        executable='map_server', 
        name='map_server', 
        output='screen',
        parameters=[
            nav2_params_file, 
            {'yaml_filename': map_file}, 
            {'use_sim_time': use_sim_time}
        ]
    )
    
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', 
        executable='lifecycle_manager', 
        name='lifecycle_manager_localization', 
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'autostart': True, 
            'node_names': ['map_server']
        }]
    )

    # 4.2 Navigation Nodes
    nav2_common_params = [
        nav2_params_file, 
        {'use_sim_time': use_sim_time}
    ]

    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=nav2_common_params),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=nav2_common_params),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=nav2_common_params),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=nav2_common_params),
        Node(package='nav2_behaviors', executable='behavior_server', name='recoveries_server', output='screen', parameters=nav2_common_params),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen', parameters=nav2_common_params),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen', parameters=nav2_common_params)
    ]

    # 4.3 Navigation Lifecycle
    nav_lifecycle = Node(
        package='nav2_lifecycle_manager', 
        executable='lifecycle_manager', 
        name='lifecycle_manager_navigation', 
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'autostart': True,
            'node_names': [
                'controller_server', 
                'planner_server', 
                'behavior_server',
                'bt_navigator', 
                'recoveries_server', 
                'velocity_smoother', 
                'collision_monitor'
            ]
        }]
    )

    # =========================================================================
    # 5. HELPER NODES (Custom Logic)
    # =========================================================================
    
    helper_common_params = [{'use_sim_time': use_sim_time}]
    
    helper_nodes = [
        Node(package='my_robot_description', executable='stuck_manager_node', name='stuck_manager_node', parameters=helper_common_params, output='screen'),
        Node(package='my_robot_description', executable='stuck_ui_node', name='stuck_ui_node', parameters=helper_common_params, output='screen'),
        Node(package='my_robot_description', executable='goal_monitor_node', name='goal_monitor_node', parameters=helper_common_params, output='screen'),
        Node(package='my_robot_description', executable='laser_to_sonar_node', name='laser_to_sonar_node', parameters=helper_common_params, output='screen'),
        Node(package='my_robot_description', executable='pause_mode_node', name='pause_mode_node', output='screen'),
        Node(package='my_robot_description', executable='return_to_home_node', name='return_to_home_node', output='screen'),
        Node(package='my_robot_description', executable='go_to_checkpoint_node', name='go_to_checkpoint_node', output='screen')
    ]

    # =========================================================================
    # 6. VISUALIZATION (RViz & Mapviz)
    # =========================================================================
    
    gui_env = os.environ.copy()
    gui_env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    gui_env['QT_QPA_PLATFORM'] = 'xcb'

    # 6.1 RViz
    rviz = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        env=gui_env
    )

  
    # =========================================================================
    # 7. EXECUTION SEQUENCE
    # =========================================================================
    
    start_main_systems = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_local_node, # รอให้ EKF ตัวแรกเริ่มก่อน
            on_start=[
                LogInfo(msg='>> Local EKF Started. Launching Global GPS, Nav2 & Helpers...'),
                
                # Start Global Localization
                ekf_global_node,
                navsat_transform_node,
                
                # Start Nav2 Stack
                map_server, 
                map_lifecycle,
                *nav2_nodes,  
                nav_lifecycle,
                
                # Visualization
                rviz,
        
                
                *helper_nodes
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        set_env_vars,
        
        # 1. Start Simulation
        robot_state_publisher, 
        gazebo, 
        spawn_robot,
        
        # 2. Start Bridges & Fixers
        bridge, 
        frame_fixer, 
        
        # 3. Start Local Localization (ตัวเริ่ม)
        ekf_local_node,
        
        # 4. Trigger the rest
        start_main_systems
    ])