# 上位机显示工作区 host_ws

执行主机：**Ubuntu 22.04 Desktop + ROS 2 Humble 上位机**。该工作区只包含 `roscar_visualization` 显示包，不包含串口、I2C、雷达采集、里程计或 EKF。树莓派负责采集、处理与 ROS 发布，上位机通过 DDS 订阅并显示。

## 1. 获取并编译（Ubuntu Desktop 本地终端）

以下假设仓库位于 `~/RosCar`，按实际位置替换；目录名包含空格时需保留引号。已有仓库先检查并保存本地改动，再拉取：

```bash
cd ~/RosCar
git status
git pull --ff-only origin main
```

尚未克隆时使用：

```bash
git clone https://github.com/JTwilliam/RosCar.git ~/RosCar
```

已有 ROS Humble 环境。依赖缺失时在上位机手动安装，脚本不自动安装：

```bash
sudo apt install build-essential python3-colcon-common-extensions \
  ros-humble-ament-cmake ros-humble-rviz2 ros-humble-rclpy \
  ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-tf2-ros \
  ros-humble-visualization-msgs
```

编译显示工作区：

```bash
cd ~/RosCar
bash host_ws/build.sh
```

脚本自动加载 `/opt/ros/humble/setup.bash`，仅搜索 `host_ws/src`，使用独立路径：

```text
host_ws/build/host_runtime
host_ws/install/host_runtime
host_ws/log/host_runtime
```

它等价于在 `host_ws` 中执行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/host_runtime build \
  --base-paths src \
  --build-base build/host_runtime \
  --install-base install/host_runtime \
  --packages-select roscar_visualization
```

不要复制 WSL 或其他机器的 build/install 目录；上位机不需要编译 `pi_ws`、`lidar_ws` 或安装 `icm20948_ros2` 来显示。

## 2. 设置双机网络环境

在上位机创建本地配置，已有文件时不覆盖：

```bash
cd ~/RosCar
cp -n host_ws/host.env.example host_ws/host.env
nano host_ws/host.env
```

两端采用相同 `ROS_DOMAIN_ID`，跨机器传输应使用 `ROS_LOCALHOST_ONLY=0`；保留两端已验证的 RMW/DDS 配置。配置中的路径必须是上位机本机路径，不是树莓派路径。脚本不修改网卡、IP、防火墙、DDS配置文件或系统时钟。

环境配置默认是 `host_ws/host.env`；可用 `ROSCAR_CONFIG=/绝对路径/配置文件` 覆盖。启动脚本明确加载 `host_ws/install/host_runtime/local_setup.bash`，避免误用旧的 pi_ws 安装结果。ROS日志默认写入 `host_ws/log/ros`。

## 3. 启动主 RViz

在 **Ubuntu Desktop 本地桌面终端**执行，不要 SSH 到树莓派后执行：

```bash
cd ~/RosCar
bash host_ws/start.sh rviz --dry-run
bash host_ws/start.sh rviz
```

不写模式时默认 `rviz`。主入口只运行 RViz，默认配置位于包内 `rviz/desktop.rviz`：

- Fixed Frame：`odom`。
- 显示 `/scan`、`/odom`、`/odometry/filtered` 和 TF。
- LaserScan 使用 Best Effort QoS。

所有传感器节点、`odom → base_link` 和机器人安装 TF 都由树莓派发布；上位机不重复发布。缺少树莓派数据或TF时，RViz出现等待数据提示是预期现象。停止使用 Ctrl+C 或关闭 RViz。

可传递 launch 参数选择自己的配置：

```bash
bash host_ws/start.sh rviz 'rviz_config:=/home/user/My Config/custom.rviz'
```

## 4. 可选 IMU 姿态视图

```bash
cd ~/RosCar
bash host_ws/start.sh imu-view --dry-run
bash host_ws/start.sh imu-view
```

该模式启动包内 `imu_viewer` 和独立 RViz，订阅树莓派的 `/imu/data`，用方块和红绿蓝坐标轴显示启动相对姿态。它只发布 `/imu/pose`、`/imu/markers` 和隔离的示意参考 TF `imu_view → imu_view_reference`，不修改 `base_link → imu_link` 或里程计，不参与融合。

指定其他姿态话题：

```bash
bash host_ws/start.sh imu-view imu_topic:=/imu/data
```

`--dry-run` 会检查本地配置、安装环境和包可用性并打印命令，不启动GUI或节点。需要先完成构建；没有 `host.env` 时会明确提示配置缺失。

## 5. 包结构与来源

```text
host_ws/
├── build.sh
├── start.sh
├── host.env.example
└── src/roscar_visualization/
    ├── CMakeLists.txt
    ├── package.xml
    ├── LICENSE
    ├── launch/viewer.launch.py
    ├── launch/imu_view.launch.py
    ├── rviz/desktop.rviz
    ├── rviz/imu.rviz
    └── scripts/imu_viewer.py
```

`desktop.rviz` 来源为仓库原 `config/desktop.rviz`；`imu.rviz` 与 `imu_viewer.py` 来源为 `pi_ws/src/icm20948_ros2` 的原显示实现。完整复制显示逻辑，仅增加脚本shebang、独立执行入口和来源说明，未引入传感器包依赖。原包 MIT 许可证已复制到显示包的 `LICENSE` 并随安装保留。

本次仅创建显示工作区源码；实际构建、启动与双机通信验证应按当前主任务的验证结果记录。本工作区未声称进行上位机现场GUI或树莓派硬件验证。
