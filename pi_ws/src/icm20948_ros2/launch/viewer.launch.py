import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('icm20948_ros2'), 'rviz', 'imu.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='icm20948_ros2', executable='imu_viewer', name='imu_viewer',
             output='screen', parameters=[{'imu_topic': LaunchConfiguration('imu_topic')}]),
        Node(package='rviz2', executable='rviz2', name='imu_rviz', output='screen',
             arguments=['-d', config], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
