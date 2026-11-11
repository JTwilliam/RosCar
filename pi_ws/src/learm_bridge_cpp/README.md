# learm_bridge_cpp

2026-11-10工作区整理：真实树莓派部署统一按 [pi_ws操作指南](../../README.md) 使用build.sh/start.sh，输出install/pi_runtime；上位机只进入host_ws。以下cpp_bridge_full仍是独立的WSL软件测试目录，不是正式启动脚本默认overlay。

C++17 STM32 串口桥，已实现原 Python `LeArmSerialBridge` 的串口接收、COBS/CRC32C 解码、原始编码器、麦轮里程计、可选 TF、重连和诊断功能。IMU 仍为 Python，雷达和 EKF 继续复用现有组件。

## 健康状态扩展（2026-11-11）

支持0x83原44字节和schema1的128字节健康帧，新增独立健康超时、当前/历史故障、心跳/资源/机械臂/通信/安全7项标准诊断。配置 `config/health.yaml`；独立launch参数 `params_file`，EKF组合参数 `health_params_file`。上位机host_ws执行 `bash start.sh health` 只订阅话题；树莓派pi_ws执行 `bash start.sh bridge` 可只启动odom和健康，无需IMU/EKF/雷达。Python回退桥不解析健康扩展。

见[完整健康说明](../../../docs/健康遥测与监视.md)。任务heartbeat按快照采样年龄比较，健康快照不推进即使持续来帧也会STALE；监视器另行检查桥停止。未知owner和机械臂query不编造为OK。

## 编译与测试（笔记本 WSL Ubuntu 22.04 / Humble）

```bash
source /opt/ros/humble/setup.bash
cd /mnt/e/Linux_Learning/RosCar/pi_ws
colcon --log-base log/cpp_bridge_full build \
  --build-base build/cpp_bridge_full --install-base install/cpp_bridge_full \
  --packages-select learm_bridge_cpp \
  --cmake-args -DLEARM_BUILD_STUDENT_NODE=ON -DBUILD_TESTING=ON \
               -DPython3_EXECUTABLE=/usr/bin/python3
# 当前WSL的FastDDS跨进程消息传输异常；本机测试使用已安装的CycloneDDS。
# 只影响当前测试终端，不改树莓派/上位机的已验证DDS配置。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 2026-11-11本机DDS验证使用非localhost-only；实际机器人配置不变。
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
colcon --log-base log/cpp_bridge_full test \
  --build-base build/cpp_bridge_full --install-base install/cpp_bridge_full \
  --packages-select learm_bridge_cpp --event-handlers console_direct+
colcon test-result --test-result-base build/cpp_bridge_full/learm_bridge_cpp --verbose
source install/cpp_bridge_full/setup.bash
```

使用新的构建目录避免旧学习节点对象被复用；不要复制 x86_64 构建产物到树莓派。原 `LEARM_BUILD_STUDENT_NODE` 选项保留用于兼容旧命令，现在默认 ON。Makefile 仍只编译纯 Linux 核心与离线 replay，不构建 ROS 节点。

## 运行

节点名为 `learm_serial_bridge`，与 Python 一致；可执行文件名为 `serial_bridge_node`。

```bash
# WSL 学习：没有此设备时持续限频重连，可观察参数与诊断；Ctrl+C 退出。
ROS_DOMAIN_ID=172 ROS_LOCALHOST_ONLY=1 \
ros2 run learm_bridge_cpp serial_bridge_node --ros-args \
  -p port:=/dev/nonexistent-stm32 -p publish_tf:=false
```

真实硬件在树莓派运行，需在板上编译并有串口权限，先停止 Python 桥或旧组合 launch。只运行桥：

```bash
ros2 launch learm_bridge_cpp serial_bridge.launch.py \
  port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 baud:=115200 publish_tf:=false
```

桥 + EKF（另有 IMU 和安装 TF）：

```bash
ros2 launch learm_bridge_cpp chassis_ekf.launch.py
```

该组合依赖已编译的 `icm20948_ros2` 配置包和 robot_localization，并强制桥 `publish_tf=false`，由 EKF 唯一发布 `odom → base_link`。项目整套脚本可在本地配置中设 `BRIDGE_IMPL=cpp`，旧配置可保留python回退，新pi.env.example默认cpp。

## 兼容接口

- `/learm/wheel_encoders`：Int32MultiArray，FR、RR、FL、RL 原始累计值。
- `/odom`：nav_msgs/Odometry，保留 vx/vy/wz，header 使用 ROS 时钟，速度按 STM32 采样 dt。
- `/diagnostics`：DiagnosticArray，保留状态名 `learm_serial_bridge`，追加 `learm/health/*` 七项，硬件 `stm32f103rbt6`；兼容原监视器字段，包括帧率、帧龄、固件状态、电池/电机和解码计数。
- `publish_tf=true` 时发布 `odom → base_link`，直接运行与独立 launch 默认 true；EKF 组合固定 false。

所有原参数保留：port、baud、read_chunk_size、diagnostics_period_s、reconnect_period_s、wheel_radius_m、wheel_base_m、track_width_m、encoder_counts_per_revolution、odom_frame、base_frame、publish_tf、twist_covariance_diagonal。默认值与 Python 一致。串口支持常见 Linux 速率 9600/19200/38400/57600/115200/230400；其他正值会诊断打开失败并重试，不支持 pyserial 的自定义任意速率。

重连保留累计错误计数，并清除半帧与序列基线；poll 检测挂断/错误，空读不作为断线。固件采样时间倒退仍按 Python 清零位姿并重建编码器基线；32 位时钟/编码器回绕策略未额外改变。read 与定时器使用 wall/steady clock，使 ROS 仿真时间暂停不阻塞设备维护；消息时间戳仍是 ROS 时间。

## 代码阅读

| 文件 | 用途 |
| --- | --- |
| `src/serial_bridge_node.cpp` | 参数、poll timer、重连、编码器、odom/TF、诊断 |
| `src/protocol.cpp` | 有界 COBS、CRC32C、显式小端解析 |
| `src/odometry.cpp` | 麦轮映射和二维积分 |
| `src/serial_port.cpp` | Linux fd/termios/poll、RAII 关闭 |
| `test/test_ros_bridge.py` | 实际节点进程 + PTY + ROS 订阅集成测试 |

完整设计与验证见 [迁移路线](../../../docs/C++迁移路线.md)、[验证记录](../../../docs/C++适配验证.md)。[节点学习说明](NODE_EXERCISE.md) 保留作为分步阅读练习，节点已按用户最新要求补齐，无需再从空文件创建。
