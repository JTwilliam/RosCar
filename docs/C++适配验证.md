# C++ 适配验证

## 2026-11-10：完整底盘桥

已补齐 `serial_bridge_node.cpp`：原 Python 的全部桥接参数、编码器/odom/TF 发布、诊断键与故障级别、2s 接收率窗口、5ms 轮询和限频重连。新增 poll HUP 检测，重连清除半帧并保留累计统计。独立/桥+EKF launch 已安装；脚本支持 `BRIDGE_IMPL=cpp`，默认 Python。

WSL Ubuntu22.04/Humble fresh 构建目录 `pi_ws/build/cpp_bridge_full` 编译通过，3 项 CTest 通过（核心、161 条 Python 等价、实际节点 PTY/ROS 集成）。ROS 集成验证缺失设备重试、分段和批量输入、vx/vy/wz与协方差、全部诊断键、CRC/溢出、fault、帧过期、固件时间倒退、设备断开与新PTY重连、累计统计保留、TF关闭/开启、无效几何参数和SIGINT正常退出。脚本新增C++选择测试，部署流程仍需板上验证。

本机 FastDDS 出现“能发现发布者但无法收消息”，独立最小 Python DiagnosticArray/String 跨进程测试也复现，证实并非C++专有。使用本机已安装 `rmw_cyclonedds_cpp`、`ROS_LOCALHOST_ONLY=1`、清除自定义DDS配置后，集成测试通过。未改系统网络或实际机器人DDS配置。

Windows文件工具保留mtime，旧学习目录一度复用旧节点对象；改为fresh构建目录重新编译完整源码并验证。后续命令见 [包说明](../pi_ws/src/learm_bridge_cpp/README.md)。本次仍没有ARM编译或真实USB-UART/实车验证；IMU未迁移。

以下为2026-11-09第一阶段历史记录，关于“节点不存在”的描述不代表当前状态。


日期：2026-11-09。执行位置：Windows 笔记本的 WSL2 `Ubuntu-22.04`，项目 `/mnt/e/Linux_Learning/RosCar`。本次未登录树莓派、未访问真实 UART/I2C、未下载或安装依赖。

## 环境与范围

| 项目 | 实际值 |
| --- | --- |
| WSL 发行版 | Ubuntu 22.04.5 LTS |
| 内核 | 6.6.87.2-microsoft-standard-WSL2 |
| 架构 | x86_64 |
| C++ 编译器 | g++ 11.4.0（Ubuntu 11.4.0-1ubuntu1~22.04.3） |
| Make | GNU Make 4.3 |
| Python | `/usr/bin/python3` 3.10.12，pytest 6.2.5 |
| ROS | `/opt/ros/humble`，ROS_DISTRO=humble |
| 目标硬件 | Raspberry Pi 4B；本次没有获取板卡内存、系统或内核版本 |
| 构建产物 | 项目内 `pi_ws/src/learm_bridge_cpp/build/make` 与 `pi_ws/build/cpp_learning`、`install/cpp_learning`、`log/cpp_learning`；均被现有 Git 忽略规则排除 |

WSL x86_64 编译成功只说明 Linux 主机上的软件构建成立；产物是 ELF x86-64，不是树莓派 ARM 二进制。

## 已执行的验证

1. **Makefile 构建和测试通过**：`make -j2 test`，C++17，`-Wall -Wextra -Wpedantic -Werror`。生成协议/里程计/串口静态库与离线工具。
2. **协议与运动学 C++ 测试通过**：既有 STM32 实采帧、CRC32C 标准向量、分段输入、CRC 损坏、重启噪声、超长候选及恢复、重复帧、错误底盘载荷、前进/左移/逆时针、首帧基线、重复时间、参数拒绝和 int32 极值差分。
3. **Linux 串口伪终端测试通过**：用 `openpty` 模拟设备，确认 raw/115200/8N1、空读非阻塞、字节传递至协议解码、显式关闭和错误打开。没有验证 USB-UART 物理断线/热插拔、调制解调器控制线或真实设备时序。
4. **C++/Python 等价测试通过**：固定随机种子生成 161 条输出，与未修改 Python 的全部编码器、位姿、速度逐字段比较；输入包含 CRC 损坏、COBS 噪声、超长候选、时间重复、设备重启和序号回绕。帧有效/丢弃/CRC/COBS/格式/溢出/序列丢帧/重复统计也对照。浮点容差 rel=1e-10、abs=1e-11。
5. **ROS 2 包构建通过**：`colcon build --packages-select learm_bridge_cpp`，独立 build/install/log 目录，不覆盖历史构建目录。
6. **colcon/CTest 通过**：`learm_core` 和 `learm_python_parity`，共 2 tests、0 errors、0 failures、0 skipped。
7. **原 Python 回归通过**：`test_protocol.py` 与 `test_odometry.py` 共 9 项通过；没有把历史完整 33 项结果当作本次重新执行。
8. **安装索引通过**：加载安装环境后，`ros2 pkg executables learm_bridge_cpp` 返回 `learm_bridge_cpp learm_replay`，与当前未提供 ROS 节点的阶段一致。

完整可复现命令见 [迁移路线](C++迁移路线.md)。构建日志留在项目忽略目录，不纳入源码提交。

## 尚未验证

- 由学习者创建的 `src/serial_bridge_node.cpp` 尚不存在，节点默认关闭；本次没有声称该节点已编译或运行。
- 未做 ARM 原生/交叉编译、实车轮序/尺度/精度、真实串口断连与重连、IMU C++、ROS 消息兼容与 EKF 双输入联调。
- 未测量迁移前后 CPU、延迟或抖动，不能据此宣称性能提升。
- 现有 Python 实车启动流程、EKF 参数、雷达 SDK 没有被本阶段替换。

## 维护记录

改动前 RosCar 独立仓库工作区干净；已 `git fetch origin`，与 origin/main 无领先/落后，基线 `7c192b8`。上级工作区的未提交内容未纳入本项目操作。提交和推送结果以本次最终回复为准。
