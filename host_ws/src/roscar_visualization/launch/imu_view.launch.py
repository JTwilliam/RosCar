"""Optional isolated IMU attitude display; no sensor package is imported."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('roscar_visualization'))
    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz' / 'imu.rviz')),
        Node(package='roscar_visualization', executable='imu_viewer', name='imu_viewer',
             output='screen', parameters=[{'imu_topic': LaunchConfiguration('imu_topic')}]),
        Node(package='rviz2', executable='rviz2', name='imu_rviz', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
