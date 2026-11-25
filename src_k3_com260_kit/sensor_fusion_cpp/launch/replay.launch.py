from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('sensor_fusion_cpp'))
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        DeclareLaunchArgument('configuration', default_value='fusion_2d.lua'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/fusion_mapping.launch.py')),
                                 launch_arguments={'use_rviz': 'true', 'configuration': LaunchConfiguration('configuration')}.items()),
        TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--clock', '--rate', '0.5',
                 '--topics', '/scan', '/imu', '/odom', '/tf', '/tf_static'], output='screen')]),
    ])
