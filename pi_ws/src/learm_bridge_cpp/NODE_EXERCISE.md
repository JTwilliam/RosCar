# 由你创建的 ROS 2 C++ 节点

本练习刻意没有提供 `src/serial_bridge_node.cpp`。请你亲手创建它，完成 `main → rclcpp::Node → 参数 → timer → 发布器 → 资源释放` 的流程。业务核心已经实现，可以边对照原 Python 节点边调用，暂不需要自己重写协议算法。

## 1. 先确定可验收的目标

第一步只创建名为 `learm_serial_bridge_cpp` 的 Node，声明 `port` 和 `baud`，用定时器打印心跳。成功 `ros2 run` 后再逐项接入下面功能，避免一次引入所有问题。

第二步完成串口和 `/learm/wheel_encoders`，第三步完成 `/odom`，最后添加重连和 `/diagnostics`。完整验收前继续使用原 Python 节点运行机器人。两者不能同时占用串口或向同一组话题发布。

## 2. 已实现的调用接口

头文件位于 `include/learm_bridge_cpp/`，命名空间均为 `learm_bridge_cpp`。

| 对象 | 你要调用的接口 | 注意事项 |
| --- | --- | --- |
| `SerialPort` | `open(port, baud)`、`is_open()`、`read_some(buffer.data(), buffer.size())`、`close()` | 非阻塞；没有数据返回 0；出错抛出异常；析构自动关闭；不可复制 |
| `ProtocolDecoder` | `feed(buffer.data(), bytes_read)`、`stats()` | 每次 read 不一定是一整帧，decoder 必须作为节点成员持续存在 |
| `decode_chassis_state` | `decode_chassis_state(frame)` | 先筛选 `frame.message_type == kMsgChassisState`；32 字节载荷校验失败抛出 `invalid_argument` |
| `MecanumOdometry` | `update(state.encoders, state.timestamp_ms * 0.001)` | 返回含位姿和速度的 `OdometryEstimate`，默认几何与 Python 一致 |
| `MecanumOdometry` | `reset(state.encoders, sample_time_s)` | STM32 时间戳倒退时调用；按原实现清零位姿并重建基线 |

可以在节点成员中保存 `std::array<std::uint8_t, 512>` 接收缓冲、上述三个对象、上次固件时间戳、发布器和 timer。用 `std::optional<std::uint32_t>` 表示“还没有上一帧”，比把 0 当成哨兵值更明确。

## 3. 节点中的处理顺序

1. 构造函数声明并读取参数，检查波特率和几何参数；建立发布器和 5ms 定时器。
2. timer 里检查串口是否打开。打开失败只记录变化后的错误，2 秒后重试；使用 `std::chrono::steady_clock` 安排重连。
3. 调用一次非阻塞 `read_some`，把实际读到的字节交给 decoder；不要按缓冲区总长度解码，不要在 timer 里阻塞循环等数据。
4. 遍历输出帧，筛选底盘消息，解码 `ChassisState`。上次固件采样时间大于本次时 reset，再 update。
5. 发布四个原始累计编码器计数，顺序必须为 FR、RR、FL、RL；`std_msgs::msg::Int32MultiArray::data` 是 vector，可用 `assign(begin,end)`。
6. 将 estimate 填入 `nav_msgs::msg::Odometry`：位置 x/y，平面四元数 z=`sin(yaw/2)`、w=`cos(yaw/2)`，速度 linear.x/y、angular.z。header 的 stamp 用节点 ROS 时钟 `now()`；计算 dt 用 STM32 时钟。
7. 错误关闭串口并延后重连。每次成功重新连接时重新创建 `ProtocolDecoder`，丢弃上次连接的半帧；里程计按固件时间戳处理重启。
8. 在 `main` 中调用 `rclcpp::init`、创建节点、`rclcpp::spin`、`rclcpp::shutdown`。RAII 自动释放串口，避免 `new/delete` 管理 fd。

原 Python 节点在固件重启时实际会清零里程计位姿。其注释中“只重建基线”不完全准确；C++ 此阶段保留现有行为。如需连续 odom，可另设计“保留位姿、清零速度、重建计数基线”的策略并单独验证。32 位 tick 回绕当前也按时间倒退处理；计数回绕尚未特殊补偿。

## 4. 与现有系统保持一致的参数和接口

| 参数 | 默认值 / 要求 |
| --- | --- |
| `port` | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| `baud` | 115200，8N1，无软/硬流控 |
| `wheel_radius_m` | 0.048 |
| `wheel_base_m` | 0.190 |
| `track_width_m` | 0.205 |
| `encoder_counts_per_revolution` | 5740.25304043815，来自既有尺度推算，仍需实车标定 |
| `odom_frame` / `base_frame` | `odom` / `base_link` |
| `publish_tf` | 建议本练习默认 false；EKF 运行时必须 false |
| `twist_covariance_diagonal` | `[0.0025,0.01,1e6,1e6,1e6,0.04]`，六个正有限方差，填写 covariance 索引 0、7、14、21、28、35 |

发布器用 depth=10，与原桥一致。`/odom` 的速度属于 `base_link`，必须保留 `linear.y`。EKF 只融合轮速 vx/vy/wz，不用本阶段未建模协方差的 pose。`/odometry/filtered` 仍由原 EKF 输出；启用 EKF 时只有它发布 `odom → base_link`。

后续诊断应覆盖串口连接、最近一帧年龄（超过 0.5s 视为过期）、固件 fault/safety、帧率、CRC/COBS/溢出/丢帧/重复计数。`stats()` 统计的是合法协议帧；底盘载荷长度错误需在节点另计，避免重复计算有效帧数。参考原 Python `telemetry.py` 和 `diagnostic_mapping.py` 实现完整兼容。

## 5. 编译你写的节点

执行主机：笔记本 WSL Ubuntu 22.04；已有 Humble 的 rclcpp/nav_msgs/std_msgs/diagnostic_msgs/geometry_msgs/tf2_ros 开发包。以下是本机学习命令，不会访问树莓派或串口。

```bash
source /opt/ros/humble/setup.bash
cd /mnt/e/Linux_Learning/RosCar/pi_ws
colcon --log-base log/cpp_learning build \
  --build-base build/cpp_learning --install-base install/cpp_learning \
  --packages-select learm_bridge_cpp \
  --cmake-force-configure \
  --cmake-args -DLEARM_BUILD_STUDENT_NODE=ON
source install/cpp_learning/setup.bash
ros2 pkg executables learm_bridge_cpp
```

第一次应先编译一个只有心跳的节点；不要在它尚无串口重连逻辑时直接连接底盘。CMake 已准备 `serial_bridge_node` 目标和所需依赖，开关默认 OFF。没有创建源文件就设为 ON 会明确报错，这是练习的预期行为。

本机心跳/模拟阶段可设置 `ROS_DOMAIN_ID=172 ROS_LOCALHOST_ONLY=1` 隔离现有机器人，然后 `ros2 run learm_bridge_cpp serial_bridge_node`。接入硬件前先补完参数和错误处理；运行位置改为树莓派，使用目标机实际设备路径和权限，并先停止 Python 桥。真实硬件接入步骤由你后续执行，本次未执行。

## 6. 验收清单

- 源文件由你创建，能解释构造函数、成员生命周期、shared_ptr、timer 回调和 main 各自负责什么。
- 能说明为什么 read 分块不等于协议帧，以及为什么两个时钟不能混用。
- 缺少设备时节点保持运行并限频重连；不在 ROS 回调中 `sleep` 等待串口。
- 模拟前进、左移、逆时针的 vx、vy、wz 分别为正；一批到达的多个样本速度仍按固件 dt 计算。
- 断开重连和 STM32 重启不把清零计数误认为巨大位移。
- 接入 EKF 时原 Python 桥已停，`/odom` 只有一个发布者，`odom → base_link` 只由 EKF 发布。
