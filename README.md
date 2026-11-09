# RosCar

基于树莓派 4B、STM32F103RBT6、麦克纳姆轮、STL-06NBJ 和 ICM-20948 的 ROS 2 项目。

**维护来源：Ubuntu 主机上传。** 本项目后续源码、配置和运行文档统一在本仓库维护。

- 树莓派：Ubuntu Server 22.04、ROS 2 Humble，运行传感器、串口桥和 EKF。
- Ubuntu 笔记本：运行 RViz，查看扫描和融合里程计。
- 当前阶段：雷达、IMU、麦轮里程计和 EKF 对接；尚未完成 SLAM/Nav2 实车验证。

## 目录

```text
pi_ws/src/learm_bridge/      STM32 协议、编码器、麦轮里程计和诊断
pi_ws/src/icm20948_ros2/     I2C IMU、校准/滤波、EKF 配置和可视化
lidar_ws/src/ldlidar/       ROS 2 雷达节点与 launch
lidar_ws/src/ldlidar_driver/ 雷达 SDK，必须与 ldlidar 相邻
references/                原始 IMU/BSP、底盘运动学与厂商参考
docs/                      运行手册、验证范围和维护方式
```

## 数据流

```text
STM32 编码器 -> learm_bridge -> /odom (vx, vy, wz) --+
                                                  +-> EKF -> /odometry/filtered
ICM-20948 -> /imu/data (gyro Z) --------------------+          odom -> base_link TF
STL-06NBJ -> /scan -> 笔记本 RViz
```

EKF 使用 robot_localization 的 ekf_node，保留麦轮横移，不重复融合编码器积分位姿，暂不融合 IMU 加速度和相对航向。组合启动关闭桥接自身 TF，由 EKF 唯一发布 odom -> base_link。

## 双机职责与启动脚本

上位机为 **Ubuntu 22.04 Desktop + ROS 2 Humble**，运行 RViz；树莓派负责 STM32/IMU/雷达数据采集、里程计、EKF 和机器人 TF，通过 ROS 2 DDS 发送到上位机。主 RViz 不需要在上位机编译传感器包。

- 树莓派：`bash scripts/raspberry_pi.sh all`，也可用 imu/chassis/lidar/imu-tf/lidar-tf 分模块调试。
- 上位机：`bash scripts/desktop.sh`，加载扫描、轮式里程计、融合里程计与 TF 视图。
- 两端先复制 `scripts/robot.env.example` 为本地 `scripts/robot.env`；树莓派整套启动要求填写实测安装 TF，或明确复用已有模型 TF。两端保持 ROS_DOMAIN_ID/DDS 网络配置一致。
- 支持 `--dry-run` 检查配置和依赖，不启动节点。

完整首次编译、配置、分模块启动和网络说明见 **[双机部署与启动](docs/双机部署与启动.md)**。脚本不会安装依赖、修改网络或远程启动设备；当前继续使用原 Python 串口桥。

## 树莓派获取和编译

```bash
git clone https://github.com/JTwilliam/RosCar.git ~/RosCar
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-serial python3-smbus \
  i2c-tools ros-humble-robot-localization ros-humble-tf2-ros

cd ~/RosCar/pi_ws
colcon build --symlink-install --packages-select learm_bridge icm20948_ros2

cd ~/RosCar/lidar_ws
colcon build --symlink-install --packages-select ldlidar --cmake-args -DBUILD_TESTING=OFF
```

接线与 I2C/UART 准备见 [IMU 说明](pi_ws/src/icm20948_ros2/README.md) 和 [雷达说明](lidar_ws/README.md)。每个终端的完整命令和 RViz 设置见 **[运行手册](docs/运行手册.md)**。安装 TF 需填实测值，不能将水平安装自动等同于零旋转。

## 当前接口

| 接口 | 发布者 | 内容 |
| --- | --- | --- |
| /scan | ldlidar | 扫描，约 10 Hz，laser |
| /imu/data_raw | imu_node | 校准后的原始惯性数据 |
| /imu/data | imu_node | 滤波惯性数据与六轴姿态，约 100 Hz，imu_link |
| /odom | learm_serial_bridge | 编码器麦轮里程计，odom/base_link |
| /learm/wheel_encoders | learm_serial_bridge | FR、RR、FL、RL 原始累计计数 |
| /odometry/filtered | ekf_filter_node | 融合里程计，约 30 Hz |
| odom -> base_link | ekf_filter_node | 融合底盘位姿 |

当前网络：笔记本共享 Wi-Fi，有线 192.168.137.1，树莓派 192.168.137.100；ROS_DOMAIN_ID=30，ROS_LOCALHOST_ONLY=0。各终端保留已验证的 DDS 配置。

## 软件验证

在 ROS Humble 环境运行，无需接真实传感器：

```bash
cd ~/RosCar
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/pi_ws/src/learm_bridge:$PWD/pi_ws/src/icm20948_ros2:$PYTHONPATH" \
ROS_DOMAIN_ID=172 ROS_LOCALHOST_ONLY=1 \
/usr/bin/python3 -m pytest -q -p no:cacheprovider \
  pi_ws/src/learm_bridge/test pi_ws/src/icm20948_ros2/test
```

历史对接记录为 33 项测试通过，实际 EKF 节点配合模拟输入验证通过。用户实机反馈 IMU 约 100 Hz、雷达约 10 Hz、EKF 约 30 Hz；频率不代表精度标定或两个输入均被有效融合，详见 [进展与验证](docs/进展与验证.md)。

## C++ 迁移学习（新增并行包）

第一阶段新增 `pi_ws/src/learm_bridge_cpp`：协议、麦轮里程计、Linux 非阻塞串口和离线采样工具，提供 Makefile 与 ament_cmake 两种构建入口。原 Python 节点及上述实车启动命令继续使用。C++ ROS 2 桥接节点的源文件刻意留给学习者创建，默认不构建节点。

建议先读 **[迁移顺序与嵌入式 Linux 路线](docs/C++迁移路线.md)**，再按 **[节点练习](pi_ws/src/learm_bridge_cpp/NODE_EXERCISE.md)** 完成节点。当前笔记本是 Windows + WSL Ubuntu 22.04；验证结果见 [C++ 适配验证](docs/C++适配验证.md)，不代表 ARM 或实车验证。

## 后续维护

后续改动在本仓库完成，验证后提交并标注 **[Ubuntu主机上传]**，正常推送 origin。树莓派更新时 git pull --ff-only 后重新编译受影响工作区并重启节点。详见 [维护说明](docs/维护说明.md)。

第三方版权和参考范围见 [THIRD_PARTY.md](THIRD_PARTY.md)。
