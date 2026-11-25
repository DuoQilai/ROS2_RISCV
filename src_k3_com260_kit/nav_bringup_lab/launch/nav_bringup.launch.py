"""Start Museum navigation with the original lab initial pose and goals."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = lambda package: Path(get_package_share_directory(package))
    course = share('param_demo_cpp')
    params = RewrittenYaml(source_file=str(course / 'config/nav2_params.yaml'), convert_types=True,
                          param_rewrites={'amcl.ros__parameters.initial_pose.x': '5.0', 'amcl.ros__parameters.initial_pose.y': '0.0', 'amcl.ros__parameters.initial_pose.yaw': '-2.0'})
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='com260_navigation'),
        DeclareLaunchArgument('map', default_value=str(course / 'maps/Software_Museum.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('robot_sim_demo') / 'launch/gazebo2.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'false', 'drive': 'false',
                              'spawn_x': '5.0', 'spawn_y': '0.0', 'spawn_yaw': '-2.0',
                              'gz_partition': LaunchConfiguration('gz_partition')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('nav2_bringup') / 'launch/bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'params_file': params, 'use_sim_time': 'true',
                              'autostart': 'true', 'use_composition': 'False'}.items()),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             arguments=['-d', str(course / 'rviz/navigation.rviz')], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
