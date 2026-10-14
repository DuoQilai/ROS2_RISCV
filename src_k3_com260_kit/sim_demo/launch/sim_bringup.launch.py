"""Chapter 9 Burger simulation with real ros2_control and Gazebo Harmonic sensors."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_simulation(context):
    share = Path(get_package_share_directory('sim_demo'))
    assets = Path(get_package_share_directory('robot_sim_demo'))
    profile = LaunchConfiguration('sensor_profile').perform(context)
    if profile not in ('lab', 'teacher'):
        raise ValueError('sensor_profile must be lab or teacher')
    sdf = (share / 'models' / f'burger_{profile}.sdf').read_text()
    sdf = sdf.replace('@CONTROLLERS_FILE@', str(share / 'config' / 'diff_drive.yaml'))
    description = (share / 'urdf' / 'burger_control.urdf').read_text()
    world = LaunchConfiguration('world')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(assets / 'models') + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '--gui-config', LaunchConfiguration('gui_config'), world],
                       output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world],
                       output='screen', condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': sim_time}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sensor_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config' / 'bridge.yaml'), 'use_sim_time': sim_time}]),
        Node(package='ros_gz_image', executable='image_bridge', name='camera_bridge',
             arguments=['/camera/image_raw'], parameters=[{'use_sim_time': sim_time}]),
        TimerAction(period=3.0, actions=[Node(
            package='ros_gz_sim', executable='create', name='spawn_burger', output='screen',
            arguments=['-world', LaunchConfiguration('world_name'), '-string', sdf,
                       '-name', 'turtlebot3_burger', '-x', LaunchConfiguration('spawn_x'),
                       '-y', LaunchConfiguration('spawn_y'), '-z', LaunchConfiguration('spawn_z'),
                       '-Y', LaunchConfiguration('spawn_yaw')])]),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60']),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['diff_drive_controller', '--controller-manager-timeout', '60']),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(share / 'rviz' / 'sensors.rviz')],
             parameters=[{'use_sim_time': sim_time}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('sim_demo'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'empty.sdf')),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gui_config', default_value=str(Path(get_package_share_directory('robot_sim_demo')) / 'gui' / 'museum.gui.config')),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('sensor_profile', default_value='lab'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='com260_ch09_ch10'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.010'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=start_simulation),
    ])
