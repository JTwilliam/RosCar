# learm_bridge_cpp

Python 串口桥的第一阶段 C++17 迁移。提供 COBS/CRC32C 解码、底盘状态解码、麦轮里程计、Linux termios 接收适配和离线 replay。当前没有 ROS 2 桥接节点，也不会替换现有 Python 启动流程。

- [迁移优先级、Makefile 原理与内核学习路线](../../../docs/C++迁移路线.md)
- [由你创建的 ROS 2 节点练习](NODE_EXERCISE.md)
- [本次验证结果](../../../docs/C++适配验证.md)

执行主机：笔记本 WSL Ubuntu 22.04，已有 G++、GNU Make 和 Python3；ROS 构建还需 Humble。以下均为离线命令，不打开硬件串口：

```bash
cd /mnt/e/Linux_Learning/RosCar/pi_ws/src/learm_bridge_cpp
make -j2
make test
# 用你已有的原始二进制串口采样文件：
# ./build/make/learm_replay /path/to/captured_uart.bin > build/make/odom.csv
```

Makefile 编译 `.o`、生成 `liblearm_bridge_core.a`，再链接 `learm_replay`。`src/serial_bridge_node.cpp` 留空缺位，按练习自行创建；CMake 默认 `LEARM_BUILD_STUDENT_NODE=OFF`，完成后开启该选项编译你的节点。

| 文件 | 用途 |
| --- | --- |
| `include/learm_bridge_cpp/protocol.hpp` / `src/protocol.cpp` | 字节流、帧统计、底盘载荷 |
| `include/learm_bridge_cpp/odometry.hpp` / `src/odometry.cpp` | FR、RR、FL、RL 符号映射和积分 |
| `include/learm_bridge_cpp/serial_port.hpp` / `src/serial_port.cpp` | Linux 非阻塞 8N1 UART、fd 生命周期 |
| `src/replay.cpp` | 文件字节流 → CSV，固件时间倒退时 reset |
| `test/test_core.cpp` | 实采帧、CRC、恢复、运动方向、PTY 串口 |
| `test/check_python_parity.py` | 对照未修改的 Python 实现 |
| `Makefile` / `CMakeLists.txt` | 离线教学构建 / ROS 2 包构建 |

本阶段统计和里程计沿用 Python 行为；完整诊断、重连调度和 ROS 消息映射由你随后在节点中完成。目标板为 Raspberry Pi 4B，但本机 x86_64 二进制不能直接部署到 ARM。没有执行硬件串口、I2C、内核或树莓派操作。
