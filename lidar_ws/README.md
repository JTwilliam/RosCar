# STL-06NBJ 雷达源码已迁移

雷达源码现位于 `../pi_ws/src/ldlidar` 和 `../pi_ws/src/ldlidar_driver`，两者必须保持相邻。`ldlidar_driver` 是由 `ldlidar` 的 CMake 直接编译的 SDK 源码目录，不是独立 ROS 包。旧 `lidar_ws` 不再构建；其已有 build/install/log 产物保留用于历史记录，不应继续 source 旧安装环境。

树莓派在新工作区单独构建雷达的接口如下；使用新的 pi_runtime 目录以免复用移动前的缓存：

```bash
source /opt/ros/humble/setup.bash
cd ~/RosCar/pi_ws
colcon --log-base log/pi_runtime build --base-paths src \
  --build-base build/pi_runtime --install-base install/pi_runtime \
  --symlink-install --packages-select ldlidar --cmake-args -DBUILD_TESTING=OFF
source install/pi_runtime/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch ldlidar stl06n.launch.py
```

当前 stl06n.launch.py 保留用户实机参数：LDLiDAR_STL06N、/dev/serial0、230400、/scan、laser、软件 min_range=0.02。软件下限不代表硬件保证最小距离。

树莓派已配置 /dev/serial0 -> ttyAMA0，需避免串口登录控制台和其他读程序占用。外部供电需共地，GPIO 信号电平必须匹配。

该 launch 不发布安装 TF；base_link -> laser 应由实测变换或模型提供。其他型号 launch 未按本机验证；上位机仅运行 RViz 订阅 /scan，不重复启动硬件节点。

原厂说明仍在 references/ldlidar_vendor/README.original.md，其 catkin/ROS 版本描述不适用于本项目，本项目使用 ROS 2 Humble 和 colcon。
