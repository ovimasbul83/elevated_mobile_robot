"""Select mock or physical Kinova hardware without changing the controller API."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def bringup(context):
    mode = LaunchConfiguration('mode').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context).strip()
    if mode == 'real' and not robot_ip:
        raise RuntimeError('Real hardware requires robot_ip:=<lab arm IP address>.')
    driver = Path(get_package_share_directory('kortex_bringup')) / 'launch' / 'gen3.launch.py'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(driver)),
        launch_arguments={
            'robot_type': 'gen3',
            'dof': '7',
            'robot_ip': robot_ip or '127.0.0.1',
            'use_fake_hardware': 'true' if mode == 'mock' else 'false',
            'fake_sensor_commands': 'false',
            'robot_controller': 'joint_trajectory_controller',
            'gripper': LaunchConfiguration('gripper').perform(context),
            'launch_rviz': LaunchConfiguration('launch_rviz').perform(context),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('gripper', default_value='',
                              choices=['', 'robotiq_2f_85', 'robotiq_2f_140']),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=bringup),
    ])
