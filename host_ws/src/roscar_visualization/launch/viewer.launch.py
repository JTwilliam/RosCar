"""Desktop main view: display only; all acquisition stays on the Raspberry Pi."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('roscar_visualization'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz' / 'desktop.rviz')),
        Node(package='rviz2', executable='rviz2', name='roscar_rviz', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
