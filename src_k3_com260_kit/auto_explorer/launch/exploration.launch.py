"""Map the enlarged COM260 project scene while Nav2 drives to frontiers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = lambda name: Path(get_package_share_directory(name))
    params = RewrittenYaml(
        source_file=str(share('navigation_console') / 'config/nav2_params.yaml'),
        param_rewrites={'max_vel_x': '0.22', 'max_speed_xy': '0.22'}, convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='com260_exploration'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('slam_bringup_lab') / 'launch/slam_bringup.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'use_rviz': LaunchConfiguration('use_rviz'),
                              'gz_partition': LaunchConfiguration('gz_partition'),
                              'world': str(share('auto_explorer') / 'config/project_museum.sdf'),
                              'gui_config': str(share('auto_explorer') / 'config/project.gui.config'),
                              'rviz_config': str(share('auto_explorer') / 'config/exploration.rviz')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('nav2_bringup') / 'launch/navigation_launch.py')),
            launch_arguments={'params_file': params, 'use_sim_time': 'true', 'autostart': 'true',
                              'use_composition': 'False'}.items()),
    ])
