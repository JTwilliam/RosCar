"""C++ bridge; parameters match the Python bridge launch interface."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(package='learm_bridge_cpp', executable='serial_bridge_node',
             name='learm_serial_bridge', output='screen', parameters=[{
                 'port': LaunchConfiguration('port'),
                 'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
                 'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                 'wheel_radius_m': 0.048, 'wheel_base_m': 0.190,
                 'track_width_m': 0.205, 'encoder_counts_per_revolution': 5740.25304043815,
             }]),
    ])
