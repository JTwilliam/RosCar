# Python → C++ 迁移与嵌入式 Linux 学习路线

日期：2026-11-09。源码以 RosCar 为准；当前笔记本实际是 Windows + WSL Ubuntu 22.04，并非历史文档所述的原生 Ubuntu 主机。树莓派环境沿用历史记录 Ubuntu Server 22.04 / ROS 2 Humble，本次没有登录板卡核实版本。

## 从哪里开始，按重要性排序

| 优先级 | 模块 | 原因与完成标准 | 当前安排 |
| --- | --- | --- | --- |
| 1 | STM32 协议 `protocol.py` | 所有底盘数据的入口；练习定宽整数、小端、CRC32C、COBS、分包/粘包、错误恢复；必须兼容原实采帧 | 已迁移到 `learm_bridge_cpp` 核心 |
| 2 | 麦轮里程计 `odometry.py` | 直接影响 EKF；先把符号、单位、固件 dt 验准，保留横移；可脱离 ROS 测试 | 已迁移并与 Python 数值对照 |
| 3 | Linux 串口与 `serial_bridge_node.py` 的 ROS 包装 | 串口是机器人实际入口，也是学习 fd、非阻塞 I/O、termios、RAII 的合适起点；先学系统接口再接 ROS 回调 | 串口适配已写；一个 ROS 2 桥接节点由你创建 |
| 4 | `telemetry.py` / `diagnostic_mapping.py` | 让断线、帧龄、重启、CRC 和丢帧可观察；完整替换旧桥前必须补齐 | 核心已有帧统计，ROS 诊断留到节点后半程 |
| 5 | IMU `sensor.py` | 迁移 I2C 寄存器访问、burst 读取、饱和与配置回读；先建立可注入的总线接口，再用 `/dev/i2c-1` 和 `ioctl(I2C_RDWR)` 适配真实总线 | 下一阶段；保持已有驱动可用 |
| 6 | IMU `fusion.py` 和 `node.py` | 保留静止校准、单位、四元数顺序、EMA 的 dt 缩放、Mahony 积分和无绝对 yaw 约束；需先有稳定采样 | IMU 原始读数正确后再迁移 |
| 7 | `dashboard.py`、`wheel_monitor_node.py`、viewer | 显示功能不处于采集关键链路，可继续用 Python 订阅 C++ 节点的话题 | 最后迁移，或保留 Python |
| 复用 | 雷达、EKF、launch/config | 雷达 SDK/ROS 节点已是 C++；robot_localization 是成熟组件；ROS 2 Python launch 和 YAML 是正常的配置方式 | 无需为了语言统一而重写 |

这里的优先级面向工程迁移与嵌入式学习，并不代表当前 100Hz IMU 或低速串口已经遇到 Python 性能瓶颈。先建立等价、可测的基础，再根据采样延迟、CPU 和抖动证据优化。

## 本次最小适配的边界

新增 `pi_ws/src/learm_bridge_cpp`，不替换已有 Python 包、启动配置或 EKF 参数：

```text
C++ SerialPort ──字节──> ProtocolDecoder ──Frame──> ChassisState
                                                     │
                                            MecanumOdometry
                                                     │
                                  由你写的 rclcpp 节点 → ROS 话题
```

核心使用 C++17，协议按字节读取小端字段，不把原始缓冲强转成打包结构体；累计计数相减先扩展到 int64，避免 C++ 有符号溢出。串口使用 Linux termios、非阻塞 read 和析构关闭，只提供接收 API，没有下发电机命令接口。候选帧长度限制与 Python 一致；容器使用标准库动态分配，这不是硬实时或零分配实现。

离线工具 `learm_replay` 读取**原始二进制串口数据**并输出 CSV，不打开硬件。十六进制文本需要先转成二进制，不能直接作为该工具的输入。工具返回 0 表示至少成功解码一条底盘状态，1 表示没有底盘状态，2 表示参数或文件错误；坏帧统计写到 stderr。

已知继承行为：固件采样时间倒退会清零积分位姿，tick 回绕也按倒退处理；编码器 int32 回绕未补偿；`dt <= 0` 更新基线但保留上次速度；重复帧仍会输出；桥接 pose 协方差未建模。这些需要单独设计策略，不能在迁移过程中悄悄更改。

## 先学 Make，再学 colcon

执行主机：笔记本 WSL Ubuntu 22.04。以下构建只在 E 盘项目内写 build/install/log，不下载依赖、不使用 sudo、不访问真实串口。

```bash
cd /mnt/e/Linux_Learning/RosCar/pi_ws/src/learm_bridge_cpp
make -n       # 只显示准备执行的命令
make -j2      # 编译对象、打包静态库、链接离线工具
make test     # 协议/运动学/伪终端测试 + Python 等价对照
```

阅读 Makefile 的顺序：`all` → `learm_replay` → `liblearm_bridge_core.a` → 三个 `.o` → 对应 `.cpp`。`-Iinclude` 指定头文件路径，`-c` 只编译不链接，`ar rcs` 打包对象，最后 G++ 将主程序和静态库链接起来。`-MMD -MP` 生成头文件依赖，使修改头文件也会触发相关对象重编译；`CXX` 可以替换编译器。Makefile 没有自动删除目标，已有实验数据无需为学习流程清理。

可自行查看中间产物理解工具链：

```bash
file build/make/learm_replay
ar t build/make/liblearm_bridge_core.a
nm -C build/make/liblearm_bridge_core.a
```

ROS 2 包使用 ament_cmake，由 colcon 调用 CMake，而 Makefile 是便于学习和离线验证的并行入口。以下不依赖你尚未编写的 ROS 节点：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/e/Linux_Learning/RosCar/pi_ws
colcon --log-base log/cpp_learning build \
  --build-base build/cpp_learning --install-base install/cpp_learning \
  --packages-select learm_bridge_cpp \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
colcon --log-base log/cpp_learning test \
  --build-base build/cpp_learning --install-base install/cpp_learning \
  --packages-select learm_bridge_cpp --event-handlers console_direct+
colcon test-result --test-result-base build/cpp_learning/learm_bridge_cpp --verbose
source install/cpp_learning/setup.bash
ros2 pkg executables learm_bridge_cpp
```

此时应该只看到 `learm_replay`，没有桥接节点。按 [节点练习](../pi_ws/src/learm_bridge_cpp/NODE_EXERCISE.md) 创建自己的节点并开启 CMake 开关。默认构建成功不能证明你未来写的节点也已通过编译。

## 从应用层走到内核，建议并行小实验

1. 先在 Linux 用户态理解 fd、错误码、`read`、`ioctl`、非阻塞与 `poll`。本项目 UART 已有内核 tty 驱动，通过 termios 使用它；迁移 C++ 不需要重写 USB-UART 驱动。
2. 为 IMU 编写可替换的用户态 I2C 总线接口，用 mock 验证寄存器序列后，再到树莓派读取 `/dev/i2c-1`。寄存器访问程序属于用户态设备访问层；它不等于 Linux 内核模块。
3. 在树莓派上记录 `uname -r`、发行版、架构、启动方式及匹配内核头文件，再做独立的最小可加载模块实验，练习 init/exit、日志、模块版本与加载卸载。
4. 用与机器人控制隔离的字符设备实验学习 `file_operations`、`copy_to_user`、等待队列和 poll，避免为了学习而替换现有工作的 UART/I2C 子系统。
5. 真正需要中断、IIO 缓冲或设备树绑定时，再学习 platform/I2C/IIO 驱动和设备树。用户态不应与绑定中的内核 IMU 驱动同时直接操作同一芯片。
6. 最后才安排匹配树莓派版本的内核配置、交叉编译、启动介质备份和部署。当前笔记本产物是 x86_64，不能复制到 ARM 树莓派直接运行；板上原生编译最适合第一轮，交叉编译还需要匹配的 aarch64 工具链及 sysroot。

以上内核与硬件步骤是后续学习计划，本次未安装工具链、构建内核、加载模块或修改目标板。先做完用户态 C++ 核心和一个 ROS 节点，能够把系统调用、业务库和消息框架三个层次分清，后续驱动学习会更具体。
