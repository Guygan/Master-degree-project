import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share_dir = FindPackageShare('my_robot_description').find('my_robot_description')
    
    # Path to your combined URDF/Xacro file
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'my_robot.urdf.xacro')

    # Process the xacro file to get the robot description content
    # IMPORTANT: Use ParameterValue with value_type=str to ensure it's treated as a string
    robot_description_content = ParameterValue(
        Command(['xacro', ' ', urdf_path]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Joint State Publisher GUI Node (optional, for controlling joints manually)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 Node
    # Assuming you have an RViz config file. Create one in my_robot_description/rviz/my_robot.rviz if you don't.
    rviz_config_path = os.path.join(pkg_share_dir, 'rviz', 'my_robot.rviz') 

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # --- เพิ่ม Node สำหรับ Topic /scan และ /range ---

    # สร้าง Bridge สำหรับ LiDAR data (จาก Gazebo ไปยัง ROS)
    # Note: ในการรัน display.launch.py แยกต่างหาก คุณอาจไม่ได้รัน Gazebo Sim Server
    # Node นี้จะมีประโยชน์เมื่อใช้ display.launch.py คู่กับ Gazebo
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge', # ตั้งชื่อ node ให้ชัดเจน
        output='screen',
        arguments=[
            # Topic นี้ควรตรงกับที่ Gazebo เผยแพร่
            '/world/default/model/my_robot/link/lidar_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/world/default/model/my_robot/link/lidar_link/sensor/lidar/scan', '/scan')
        ],
    )

    # Node สำหรับแปลง LaserScan เป็น Range (Sonar)
    laser_to_sonar_node = Node(
        package='my_robot_description', # ระบุว่า Node นี้อยู่ในแพ็กเกจ my_robot_description
        executable='laser_to_sonar', # ชื่อ executable ที่กำหนดใน setup.py
        name='laser_to_sonar_node',
        output='screen',
        # สามารถส่งพารามิเตอร์ให้กับ Node ได้ที่นี่
        parameters=[
            {'pub_topic': '/range'}, 
            {'sub_topic': '/scan'}   
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node, # สามารถคอมเมนต์บรรทัดนี้ได้ถ้าไม่ต้องการ GUI ควบคุม Joint
        rviz_node,
        lidar_bridge,      # เพิ่ม LiDAR Bridge Node
        laser_to_sonar_node # เพิ่ม Laser to Sonar Node
    ])
