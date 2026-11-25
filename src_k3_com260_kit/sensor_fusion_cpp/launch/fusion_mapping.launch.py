from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('sensor_fusion_cpp')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('configuration', default_value='fusion_2d.lua'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             arguments=['-configuration_directory', str(config),
                        '-configuration_basename', LaunchConfiguration('configuration')],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='fusion_rviz',
             arguments=['-d', str(config.parent / 'rviz/fusion.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
