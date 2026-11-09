from glob import glob
from setuptools import find_packages, setup

setup(
    name='icm20948_ros2',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/icm20948_ros2']),
        ('share/icm20948_ros2', ['package.xml', 'README.md', 'MECANUM.md', 'LICENSE']),
        ('share/icm20948_ros2/config', glob('config/*.yaml')),
        ('share/icm20948_ros2/launch', glob('launch/*.launch.py')),
        ('share/icm20948_ros2/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Robot maintainer',
    maintainer_email='maintainer@example.com',
    description='ICM-20948 Linux I2C driver and six-axis attitude fusion',
    license='MIT',
    entry_points={'console_scripts': [
        'imu_node = icm20948_ros2.node:main',
        'imu_viewer = icm20948_ros2.viewer:main',
    ]},
)
