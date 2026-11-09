from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
                description="Stable USB serial device path for the STM32 link",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="STM32 USART1 baud rate",
            ),
            Node(
                package="learm_bridge",
                executable="learm_serial_bridge",
                name="learm_serial_bridge",
                output="log",
                parameters=[
                    {
                        "port": port,
                        "baud": ParameterValue(baud, value_type=int),
                        "wheel_radius_m": 0.048,
                        "wheel_base_m": 0.190,
                        "track_width_m": 0.205,
                        "encoder_counts_per_revolution": 5740.25304043815,
                    }
                ],
            ),
            Node(
                package="learm_bridge",
                executable="wheel_monitor",
                name="wheel_monitor",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
