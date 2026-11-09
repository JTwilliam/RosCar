"""只启动串口桥和里程计发布，不启动终端监视器。"""

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
                description="STM32 USB 串口稳定设备路径",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="STM32 USART1 波特率",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="独立使用时发布 TF；接入 EKF 时必须设为 false",
            ),
            Node(
                package="learm_bridge",
                executable="learm_serial_bridge",
                name="learm_serial_bridge",
                output="screen",
                parameters=[
                    {
                        "port": port,
                        "baud": ParameterValue(baud, value_type=int),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_tf"), value_type=bool
                        ),
                        "wheel_radius_m": 0.048,
                        "wheel_base_m": 0.190,
                        "track_width_m": 0.205,
                        "encoder_counts_per_revolution": 5740.25304043815,
                    }
                ],
            ),
        ]
    )
