"""Compose Burger simulation and the Humble Nav2 lifecycle nodes on x86."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('param_demo_cpp')
    sim = get_package_share_directory('robot_sim_demo')
    nav2 = get_package_share_directory('nav2_bringup')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    gz_headless = LaunchConfiguration('gz_headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('gz_headless', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='com260_ch06'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'gazebo2.launch.py')),
            condition=IfCondition(use_gazebo),
            launch_arguments={
                'gui': PythonExpression(["'false' if '", gz_headless, "' == 'true' else 'true'"]),
                'rviz': 'false', 'drive': 'false', 'use_sim_time': use_sim_time,
                'gz_partition': LaunchConfiguration('gz_partition'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': os.path.join(share, 'maps', 'Software_Museum.yaml'),
                'params_file': os.path.join(share, 'config', 'nav2_params.yaml'),
                'use_sim_time': use_sim_time, 'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             condition=IfCondition(use_rviz), output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
