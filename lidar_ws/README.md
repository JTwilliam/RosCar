# STL-06NBJ 雷达工作区

src/ldlidar 和 src/ldlidar_driver 必须相邻，后者由前者 CMake 直接编译，不是独立 ROS 包。

```bash
source /opt/ros/humble/setup.bash
cd ~/RosCar/lidar_ws
colcon build --symlink-install --packages-select ldlidar --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch ldlidar stl06n.launch.py
```

当前 stl06n.launch.py 对齐用户实机：LDLiDAR_STL06N、/dev/serial0、230400、/scan、laser、软件 min_range=0.02。软件下限不代表硬件保证最小距离。

树莓派已配置 /dev/serial0 -> ttyAMA0，需避免串口登录控制台和其他读程序占用。外部供电需共地，GPIO 信号电平必须匹配。

本 launch 不发布安装 TF；base_link -> laser 应由实测变换或模型提供。其他型号 launch 未按本机验证；笔记本直接运行 RViz 订阅 /scan，不重复启动硬件节点。

原厂说明在 references/ldlidar_vendor/README.original.md，其 catkin/ROS 版本描述不适用于本项目，本项目使用 ROS 2 Humble 和 colcon。
