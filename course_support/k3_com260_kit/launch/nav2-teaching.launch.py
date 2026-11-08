"""Run the chapter 16–17 exercises with precise navigation endpoints."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory('param_demo_cpp'))
    sim = Path(get_package_share_directory('robot_sim_demo'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    params = RewrittenYaml(
        source_file=str(share / 'config/nav2_params.yaml'), convert_types=True,
        param_rewrites={
            'planner_server.ros__parameters.GridBased.tolerance': '0.15',
            'controller_server.ros__parameters.progress_checker.required_movement_radius': '0.10',
            'controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance': '0.10',
            'controller_server.ros__parameters.FollowPath.xy_goal_tolerance': '0.10',
        })
    return LaunchDescription([
        DeclareLaunchArgument('gz_partition', default_value='com260_ch16_ch17'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch/gazebo2.launch.py')),
            launch_arguments={'rviz': 'false', 'drive': 'false', 'use_sim_time': 'true',
                              'gz_partition': LaunchConfiguration('gz_partition')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/bringup_launch.py')),
            launch_arguments={'map': str(share / 'maps/Software_Museum.yaml'),
                              'params_file': params, 'use_sim_time': 'true',
                              'autostart': 'true', 'use_composition': 'False'}.items()),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz', output='screen',
             arguments=['-d', str(share / 'rviz/navigation.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
