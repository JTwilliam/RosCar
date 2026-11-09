from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('icm20948_ros2'), 'config', 'imu.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('bus', default_value='1'),
        DeclareLaunchArgument('address', default_value='104'),
        Node(
            package='icm20948_ros2', executable='imu_node', name='icm20948',
            output='screen', parameters=[LaunchConfiguration('params_file'), {
                'bus': ParameterValue(LaunchConfiguration('bus'), value_type=int),
                'address': ParameterValue(LaunchConfiguration('address'), value_type=int),
            }],
        ),
    ])
