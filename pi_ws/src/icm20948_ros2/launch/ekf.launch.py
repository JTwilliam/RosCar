from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('icm20948_ros2'), 'config', 'ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
