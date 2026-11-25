"""Start Museum Gazebo, slam_toolbox and RViz for the mapping lab."""
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
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(share('robot_sim_demo') / 'worlds/museum.sdf')),
        DeclareLaunchArgument('gui_config', default_value=str(share('robot_sim_demo') / 'gui/museum.gui.config')),
        DeclareLaunchArgument('rviz_config', default_value=str(share('slam_sim_demo_ros2') / 'rviz/slam.rviz')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='com260_mapping'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('robot_sim_demo') / 'launch/gazebo2.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'false', 'drive': 'false',
                              'gz_partition': LaunchConfiguration('gz_partition'),
                              'world': LaunchConfiguration('world'),
                              'gui_config': LaunchConfiguration('gui_config')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('slam_toolbox') / 'launch/online_async_launch.py')),
            launch_arguments={'slam_params_file': str(share('slam_sim_demo_ros2') / 'params/slam_toolbox_params.yaml'),
                              'use_sim_time': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', name='mapping_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
