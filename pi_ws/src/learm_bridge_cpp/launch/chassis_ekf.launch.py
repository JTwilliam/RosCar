"""C++ bridge plus unchanged EKF; IMU acquisition runs separately."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    own = Path(get_package_share_directory('learm_bridge_cpp'))
    imu = Path(get_package_share_directory('icm20948_ros2'))
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('params_file', default_value=str(imu / 'config/ekf.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(own / 'launch/serial_bridge.launch.py')),
                                 launch_arguments={'port': LaunchConfiguration('port'),
                                                   'baud': LaunchConfiguration('baud'),
                                                   'publish_tf': 'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(imu / 'launch/ekf.launch.py')),
                                 launch_arguments={'params_file': LaunchConfiguration('params_file')}.items()),
    ])
