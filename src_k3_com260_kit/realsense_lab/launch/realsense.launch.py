"""Choose the simulated RGB-D sensor or the RealSense D400 driver."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    simulated = LaunchConfiguration('simulation').perform(context) == 'true'
    pointcloud = LaunchConfiguration('pointcloud').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    package = 'realsense_lab' if simulated else 'realsense2_camera'
    filename = 'rgbd_sim.launch.py' if simulated else 'rs_launch.py'
    arguments = {'pointcloud': pointcloud, 'profile': profile,
                 'use_rviz': LaunchConfiguration('use_rviz'),
                 'gz_partition': LaunchConfiguration('gz_partition')} if simulated else {
                     'depth_module.depth_profile': profile,
                     'rgb_camera.color_profile': profile,
                     'pointcloud.enable': pointcloud, 'align_depth.enable': pointcloud,
                     'enable_sync': 'true'}
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('pointcloud', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('profile', default_value='640x480x30',
                              choices=['640x480x30', '1280x720x15']),
        DeclareLaunchArgument('gz_partition', default_value='com260_rgbd_acquisition'),
        OpaqueFunction(function=start),
    ])
