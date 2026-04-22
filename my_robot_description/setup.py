import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch, config, urdf, etc.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.config')),
        (os.path.join('share', package_name, 'config'), glob('config/*.mvc')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guygan',
    maintainer_email='guygan2002@gmail.com',
    description='Robot with RViz, Gazebo, and topic-based coordination nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_fixer = my_robot_description.frame_fixer:main',
            'goal_monitor_node = my_robot_description.goal_monitor_node:main',
            'laser_to_sonar_node = my_robot_description.laser_to_sonar_node:main',
            'pause_mode_node = my_robot_description.pause_mode_node:main',
            'return_to_home_node = my_robot_description.return_to_home_node:main',
            'stuck_manager_node = my_robot_description.stuck_manager_node:main',
            'stuck_ui_node = my_robot_description.stuck_ui_node:main',
            'waypoint_recorder = my_robot_description.waypoint_recorder:main',
            'farm_waypoint_navigator = my_robot_description.farm_waypoint_navigator:main',
            'go_to_checkpoint_node = my_robot_description.go_to_checkpoint_node:main',
            'experiment_runner = my_robot_description.experiment_runner:main',
            'path_recorder_node = my_robot_description.path_recorder_node:main',
        ],
    },
)
