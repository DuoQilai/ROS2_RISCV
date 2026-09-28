"""Launch arguments for Gazebo, RViz and the C++ patrol driver on x86."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = get_package_share_directory('robot_sim_demo')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('drive', default_value='false'),
        DeclareLaunchArgument('drive_linear_speed', default_value='0.12'),
        DeclareLaunchArgument('drive_angular_speed', default_value='0.45'),
        DeclareLaunchArgument('drive_loop', default_value='true'),
        DeclareLaunchArgument('drive_duration', default_value='0.0'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('gz_partition', default_value='com260_ch06'),
        # Resolve this drive condition before the included launch sets drive=false.
        Node(package='param_demo_cpp', executable='patrol_driver', name='patrol_driver',
             condition=IfCondition(LaunchConfiguration('drive')), output='screen',
             parameters=[{
                 'linear_speed': ParameterValue(LaunchConfiguration('drive_linear_speed'), value_type=float),
                 'angular_speed': ParameterValue(LaunchConfiguration('drive_angular_speed'), value_type=float),
                 'loop': ParameterValue(LaunchConfiguration('drive_loop'), value_type=bool),
                 'duration': ParameterValue(LaunchConfiguration('drive_duration'), value_type=float),
             }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'gazebo2.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'),
                'drive': 'false', 'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'gz_partition': LaunchConfiguration('gz_partition'),
            }.items(),
        ),
    ])
