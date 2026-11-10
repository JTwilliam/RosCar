"""Raspberry Pi acquisition/processing only. RViz runs on the desktop host."""
import math
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def include(package, filename, arguments=None):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=(arguments or {}).items())


def boolean(context, name):
    value = LaunchConfiguration(name).perform(context)
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')
    return value == 'true'


def mounting_tf(context, argument, child):
    parts = shlex.split(LaunchConfiguration(argument).perform(context))
    if len(parts) != 6 or not all(math.isfinite(float(v)) for v in parts):
        raise ValueError(f'{argument} requires measured x y z roll pitch yaw (m/rad)')
    named = []
    for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), parts):
        named.extend([f'--{name}', value])
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=f'base_link_to_{child}', output='screen',
                arguments=named + ['--frame-id', 'base_link', '--child-frame-id', child])


def system(context):
    bridge = LaunchConfiguration('bridge').perform(context)
    if bridge not in ('python', 'cpp'):
        raise ValueError('bridge must be python or cpp')
    bridge_package = 'learm_bridge_cpp' if bridge == 'cpp' else 'icm20948_ros2'
    lidar = boolean(context, 'enable_lidar')
    external_tf = boolean(context, 'external_mount_tf')
    # Validate both mounting transforms before returning any runnable actions.
    transforms = []
    if not external_tf:
        transforms.append(mounting_tf(context, 'imu_mount', 'imu_link'))
        if lidar:
            transforms.append(mounting_tf(context, 'lidar_mount', 'laser'))
    actions = transforms + [
        include('icm20948_ros2', 'imu.launch.py', {
            'bus': LaunchConfiguration('bus'), 'address': LaunchConfiguration('address')}),
        include(bridge_package, 'chassis_ekf.launch.py', {
            'port': LaunchConfiguration('port'), 'baud': LaunchConfiguration('baud')}),
    ]
    if lidar:
        actions.append(include('ldlidar', 'stl06n.launch.py'))
    return actions


def generate_launch_description():
    defaults = {
        'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        'baud': '115200', 'bus': '1', 'address': '104',
        'enable_lidar': 'true', 'external_mount_tf': 'false', 'bridge': 'python',
        'imu_mount': '', 'lidar_mount': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=system),
    ])
