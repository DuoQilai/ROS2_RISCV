"""Launch the SLAM project map and navigation configuration."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = lambda package: Path(get_package_share_directory(package))
    course = share('param_demo_cpp')
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='com260_navigation'),
        DeclareLaunchArgument('map', default_value='/maps/processed_map.yaml'),
        DeclareLaunchArgument('params_file', default_value=str(share('navigation_console') / 'config/nav2_params.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('robot_sim_demo') / 'launch/gazebo2.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'false', 'drive': 'false',
                              'world': str(share('auto_explorer') / 'config/project_museum.sdf'),
                              'gui_config': str(share('auto_explorer') / 'config/project.gui.config'),
                              'spawn_x': '0.0', 'spawn_y': '0.0', 'spawn_yaw': '0.0',
                              'gz_partition': LaunchConfiguration('gz_partition')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('nav2_bringup') / 'launch/bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'params_file': params, 'use_sim_time': 'true',
                              'autostart': 'true', 'use_composition': 'False'}.items()),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             arguments=['-d', str(course / 'rviz/navigation.rviz')], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
