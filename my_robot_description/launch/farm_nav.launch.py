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
    map_file = os.path.join(pkg_path, 'maps', 'my_farm_map.yaml') 
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params_gps.yaml')
    bridge_params_file = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'my_nav2_view.rviz')
    world_file = os.path.join(pkg_path, 'worlds', 'farm.sdf')
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf_gps.yaml')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Environment Variables
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
    # 3. LOCALIZATION NODES (EKF & Static TF)
    # =========================================================================
    
    ekf_node = Node(
       package='robot_localization', 
       executable='ekf_node', 
       name='ekf_filter_node',
       output='screen', 
       parameters=[
           ekf_config_file, 
           {'use_sim_time': use_sim_time}
       ]
    )

    # Static TF (Map -> Odom) *ใช้ชั่วคราวแทน GPS เต็มระบบ*
    map_to_odom_tf = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0.0', '0.0', '0', 
            '0', '0', '0', 
            'map', 'odom'
        ]
    )

    # =========================================================================
    # 4. NAVIGATION STACK (NAV2 NODES)
    # =========================================================================
    
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

    # 4.2 Navigation Nodes (Controller, Planner, Behaviors, etc.)
    # รวมไว้ใน List เพื่อความสะอาด
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

    # 4.3 Navigation Lifecycle Manager
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
    # 5. HELPER NODES (Custom Logic) - Vertical Style
    # =========================================================================
    
    helper_common_params = [{'use_sim_time': use_sim_time}]
    
    helper_nodes = [
        # 5.1 Stuck Manager
        Node(
            package='my_robot_description', 
            executable='stuck_manager_node', 
            name='stuck_manager_node',
            parameters=helper_common_params,
            output='screen'
        ),
        # 5.2 UI Node
        Node(
            package='my_robot_description', 
            executable='stuck_ui_node', 
            name='stuck_ui_node',
            parameters=helper_common_params,
            output='screen'
        ),
        # 5.3 Goal Monitor
        Node(
            package='my_robot_description', 
            executable='goal_monitor_node', 
            name='goal_monitor_node',
            parameters=helper_common_params,
            output='screen'
        ),
        # 5.4 Laser to Sonar
        Node(
            package='my_robot_description', 
            executable='laser_to_sonar_node', 
            name='laser_to_sonar_node',
            parameters=helper_common_params,
            output='screen'
        ),
        # 5.5 Pause Mode
        Node(
            package='my_robot_description', 
            executable='pause_mode_node', 
            name='pause_mode_node',
            output='screen'
        ),
        # 5.6 Return to Home
        Node(
            package='my_robot_description', 
            executable='return_to_home_node', 
            name='return_to_home_node',
            output='screen'
        ),
        # 5.7 Go to Checkpoint
        Node(
            package='my_robot_description', 
            executable='go_to_checkpoint_node', 
            name='go_to_checkpoint_node',
            output='screen'
        )
    ]

    # =========================================================================
    # 6. VISUALIZATION (RViz)
    # =========================================================================
    rviz = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
       
    )

    # =========================================================================
    # 7. EXECUTION SEQUENCE
    # =========================================================================
    
    # รอให้ EKF พร้อมก่อน ค่อยรัน Nav2 และ Helper Nodes
    start_main_systems = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_node,
            on_start=[
                LogInfo(msg='>> EKF Started. Launching Map, Nav2 & Helpers...'),
                map_server, 
                map_lifecycle,
                
                # Unpack lists of nodes
                *nav2_nodes,  
                nav_lifecycle,
                
                rviz,
                
                *helper_nodes
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        set_env_vars,
        
        # 1. Start Simulation Environment
        robot_state_publisher, 
        gazebo, 
        spawn_robot,
        
        # 2. Start Bridges & Fixers
        bridge, 
        frame_fixer, 
        
        # 3. Start TF & Localization
        map_to_odom_tf, 
        ekf_node,
        
        # 4. Trigger the rest
        start_main_systems
    ])