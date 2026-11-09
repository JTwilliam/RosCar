from glob import glob
from setuptools import find_packages, setup


package_name = "learm_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="maintainer@example.invalid",
    description="ROS 2 serial bridge, mecanum odometry and monitor for LeArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "learm_serial_bridge = learm_bridge.serial_bridge_node:main",
            "wheel_monitor = learm_bridge.wheel_monitor_node:main",
        ],
    },
)
