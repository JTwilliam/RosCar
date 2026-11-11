# 树莓派工作区 pi_ws

**此目录的硬件启动命令在树莓派执行。** Raspberry Pi 4B / Ubuntu 22.04 / ROS 2 Humble。STM32 串口桥、IMU、雷达、麦轮里程计、EKF 和机器人安装 TF 都在这里运行。上位机日常显示请进入仓库的 `host_ws`。

## 1. 更新与编译（树莓派）

```bash
cd ~/RosCar
git status --short
git pull --ff-only origin main
cd pi_ws
bash build.sh
```

第一次没有仓库时先在树莓派 `git clone https://github.com/JTwilliam/RosCar.git ~/RosCar`。请使用本机源码编译，不复制笔记本的 x86_64 build/install 到 ARM。已有本地修改时先保留再拉取。

脚本自动加载 `/opt/ros/humble/setup.bash`，只搜索本工作区 `src`，编译 learm_bridge、learm_bridge_cpp、icm20948_ros2、ldlidar。输出目录为 `build/pi_runtime`、`install/pi_runtime`、`log/pi_runtime`，不使用移动前的旧缓存，也不删除旧实验产物。可从任意当前目录调用本脚本。

首次缺少依赖时，由你在树莓派终端安装（脚本不自动安装、不修改权限/网络）：

```bash
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions python3-serial python3-smbus \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-nav-msgs \
  ros-humble-std-msgs ros-humble-diagnostic-msgs ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-tf2-ros ros-humble-robot-localization
```

I2C1 物理3/5、串口接线及dialout权限需已按实际设备准备。不要用sudo运行整个ROS启动脚本。

## 2. 配置（树莓派）

```bash
cd ~/RosCar/pi_ws
cp -n pi.env.example pi.env
nano pi.env
```

- 新模板选择 `BRIDGE_IMPL=cpp`；若要保留旧桥可设 python。
- `IMU_MOUNT`、`LIDAR_MOUNT` 必须填写六个实测数值：x y z roll pitch yaw，单位m/rad。
- 已有机器人模型发布安装TF时，设 `USE_EXTERNAL_MOUNT_TF=true`；不能重复发布。
- 暂不用雷达设 `ENABLE_LIDAR=false`。
- 两端使用相同ROS_DOMAIN_ID=30、ROS_LOCALHOST_ONLY=0，保留各自主机已验证的DDS配置。
- 此配置不提交Git。旧 `scripts/robot.env` 不会自动覆盖或迁移，请将实测值及DDS设置手动复制到 `pi.env`。

## 3. 启动（树莓派）

```bash
cd ~/RosCar/pi_ws
bash start.sh all --dry-run
# IMU静止后启动，校准阶段保持约3秒静止
bash start.sh all
```

一个launch管理IMU、桥、EKF、可选雷达和安装TF，Ctrl+C停止。EKF唯一发布 odom→base_link；桥固定publish_tf=false。不要同时启动旧组合、旧桥或以下分模块入口。

分模块调试，每条长期运行命令占一个树莓派终端：

```bash
bash start.sh imu
bash start.sh imu-tf
bash start.sh chassis
bash start.sh lidar
bash start.sh lidar-tf
```

`chassis`包含桥+EKF；IMU和安装TF需要另有进程提供。更换STM32路径可修改pi.env，或 `bash start.sh chassis port:=/dev/ttyUSB0`。

## 健康遥测最小启动（树莓派）

当前仅验收底盘和串口时，无需先启IMU/EKF/雷达。更新固件健康扩展、编译C++桥，并在pi.env设 `BRIDGE_IMPL=cpp` 后：

```bash
bash start.sh bridge --dry-run
bash start.sh bridge
```

此模式只由一个桥进程打开串口，发布编码器、odom及健康诊断，默认不发布TF，不依赖robot_localization。不能与all/chassis或另一个串口桥同时运行。上位机另开终端在host_ws执行 `bash start.sh health`，它只订阅ROS话题。旧Python回退桥暂不解析扩展健康消息，使用本功能须选cpp。

阈值配置见 `src/learm_bridge_cpp/config/health.yaml`；独立桥可用 `bash start.sh bridge params_file:=/绝对路径/health.yaml`。完整行为、未知字段及实机验收见[健康遥测说明](../docs/健康遥测与监视.md)。

## 4. 检查（树莓派）

另一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/RosCar/pi_ws/install/pi_runtime/local_setup.bash
source ~/RosCar/pi_ws/pi.env
ros2 node list
ros2 topic info /odom
ros2 topic echo /diagnostics --once
ros2 topic hz /imu/data
# 上一条用Ctrl+C退出后再查其他频率
ros2 topic hz /scan
```

还应核对 `/odometry/filtered`、方向、尺度和安装TF。EKF约30Hz只说明输出运行，不单独证明两个输入有效。停止前先让底盘停止运动，再Ctrl+C。

## 5. WSL学习与软件测试（笔记本，不接硬件）

同一份树莓派源码也可在WSL x86_64做模拟测试；它不改变正式部署职责：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/e/Linux_Learning/RosCar/pi_ws
colcon --log-base log/pi_checks build --base-paths src \
  --build-base build/pi_checks --install-base install/pi_checks \
  --packages-select learm_bridge_cpp --cmake-args -DBUILD_TESTING=ON \
  -DLEARM_BUILD_STUDENT_NODE=ON -DPython3_EXECUTABLE=/usr/bin/python3
# 当前WSL的FastDDS数据通道异常；仅本次测试终端使用已安装的CycloneDDS。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
colcon --log-base log/pi_checks test \
  --build-base build/pi_checks --install-base install/pi_checks \
  --packages-select learm_bridge_cpp --event-handlers console_direct+
colcon test-result --test-result-base build/pi_checks/learm_bridge_cpp --verbose
```

## 目录迁移说明

雷达原 `lidar_ws/src/ldlidar` 与 `ldlidar_driver` 已移动到本工作区 `src`，两目录保持相邻。雷达CMake从自身位置定位SDK；不会引用移动前的绝对路径。旧lidar_ws仅保留迁移说明和本地历史产物，不再进入该目录构建。C++包仍在 `pi_ws/src/learm_bridge_cpp`，现有源码路径未改变。
