# 本工程健康遥测开发入口

2026-11-12归档说明：以下为迁入RosCar前的健康功能开发记录，旧绝对路径和“无独立Git远端”表述仅属历史。当前维护入口、完整安全启动修复、Keil编译和最新验收请看 [README.md](README.md)。本目录已随RosCar维护，不在旧下载目录开发。

本会话维护的FreeRTOS下位机工程是当前目录。串口/底盘采样验收基线保留，健康诊断复用USART1 COBS/CRC32C帧流。完整字段、语义和验收限制见 [docs/健康遥测.md](docs/健康遥测.md)。

## 改了什么

- HEALTH_STATE(0x83)保留原44字节前缀，新增schema1扩展，固定128字节。
- 独立观测当前故障、历史故障、最近事件、任务心跳、资源、底盘采样和命令年龄；安全状态不等于健康等级。
- 修复CRC失败计数，TX失败独立计数，累加饱和。
- Keil工程已加入RobotHealthWire.c，序列化按字段小端，不发送结构体内存。
- 不新增运动控制或修改安全许可；未编造owner或机械臂query数据。

## 软件验证（开发电脑/WSL，不是目标板）

已有ARM GCC在PATH时，Windows或Linux执行：

```bash
python tests/compile_health_arm.py
```

仅编译8个改动源为ARM对象并生成stack-usage，不链接、不烧录。

在本机WSL或其他Linux终端，已有gcc：

```bash
# WSL实际路径；其他主机替换成工程位置
cd /mnt/e/BaiduDownload/LeArm/LeArm_APP_rtos
bash tests/run_health_tests.sh
mkdir -p build/health_host_tests
gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc Core/Src/RobotHealthWire.c \
  tests/test_health_wire.c -o build/health_host_tests/test_health_wire
build/health_host_tests/test_health_wire build/health_host_tests/health_fixture.bin
gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc Core/Src/RobotProtocol.c \
  tests/test_protocol_crc.c -o build/health_host_tests/test_protocol_crc
build/health_host_tests/test_protocol_crc
```

ROS侧位于 `E:\Linux_Learning\RosCar`：树莓派pi_ws运行cpp桥，Ubuntu上位机host_ws运行 `bash start.sh health`。最小桥命令为pi_ws中的 `bash start.sh bridge`，无需IMU/EKF/雷达。监视器不打开串口，关闭它不会影响odom。ROS侧完整说明：RosCar/docs/健康遥测与监视.md。

## 下一步实机

用原MDK-ARM/LeArm.uvprojx在本机编译，确认新增源、原应用Flash地址和容量。由用户后续安排烧录并验收20ms底盘/100ms健康周期、TX丢弃、最小剩余栈和堆。不要把旧CubeMX CMake默认0x08000000布局直接当可烧录应用；它原本缺完整业务与应用链接配置，本次未改启动布局。

本次已做对象编译、主机状态/协议测试和C编码fixture经ROS桥解码；没有连接设备或烧录。历史故障只保存在本次运行RAM内，不提供断电持久化。

## Git范围

当前工程没有独立Git远端，git向上找到E:\BaiduDownload父仓库。为避免把下载目录和无关文件纳入提交，本次不在父仓库提交或推送。固件修改保存在本工程；RosCar的ROS侧修改可以独立提交到其原远端。
