# STM32F103RBT6 App：Keil 编译和烧录

这是 RosCar 当前维护的完整 App 工程，来源为用户提供的 `/mnt/data/LeArm_APP_rtos`。
后续修改本目录，不再修改下载目录或 `references` 中的旧副本。
本目录包含业务源码、ST HAL/CMSIS、FreeRTOS、Hiwonder/U8g2、厂商 `LeArm.lib`、Keil 工程和 App 分区文件。
不包含 Bootloader 工程、个人调试器设置、历史编译结果或凭据。

## 本次修复（2026-11-12，Ubuntu 主机）

- `Core/Src/main.c`：启动任务前初始化健康模块，避免心跳检查把未初始化误作超时。
- `Core/Src/freertos.c`：在创建任何业务线程之前调用 `robot_safety_init(HAL_GetTick())`；失败进入 `Error_Handler`，不继续创建业务任务。
- `Core/Src/RobotSafety.c`、`Core/Inc/RobotSafety.h`：初始化限定为内核 `Ready` 状态，即 `osKernelInitialize` 之后、`osKernelStart` 之前。CMSIS 允许此时创建互斥锁；运行期不允许重新初始化来清除故障。
- `RobotHealth.c/.h`、`SerialTask.c`：快照返回成功标记，未初始化时不发送；发送局部状态先清零，避免旧版未初始化栈数据被封装成合法 CRC 报文。
- 保留原麦轮算法、编码器顺序 FR/RR/FL/RL、USART1 115200 和 STOP/BOOT 授权流程；不把 FAULT 改成正常，不自动清故障。

正常启动顺序：

```text
main：外设初始化 → robot_health_init → osKernelInitialize
MX_FREERTOS_Init：创建互斥锁 → robot_safety_init → 创建队列/5个任务/事件
main：osKernelStart
HealthTask：确认底盘/机械臂/串口/PS2均READY → SAFE_IDLE
```

初始化完成不等于自动进入 NORMAL；只有有效运动指令和安全许可满足时才允许运动。
硬件、资源或心跳存在真实故障时仍会进入 FAULT。底盘消息中的 `fault_flags` 不等于安全模块全部内部故障，不能单凭该字段为零判断可以运动。

## Windows / Keil 操作

1. 在 Windows 的 RosCar 仓库执行 `git pull --ff-only origin main`。
2. 用 Keil µVision 打开 **`firmware/LeArm_APP_rtos/MDK-ARM/LeArm.uvprojx`**，选择目标 **LeArm**。
3. 原工程使用 **Arm Compiler 6.24**、`Keil.STM32F1xx_DFP 2.4.1`；保留原 ARM Compiler 6 配置，不切换 ARM Compiler 5。工程还声明 CMSIS 6.2.0 和 Keil ARM_Compiler 的 Event Recorder/STDOUT 组件，缺包时按 µVision 提示安装对应包。
4. 执行 **Rebuild all target files**。确认 0 errors，并查看新生成的 `MDK-ARM/LeArm/LeArm.map`，核对 App 起点和容量。
5. `Options for Target → Debug` 选择实际使用的 ST-Link 和 SWD；本仓库不提交个人 `.uvoptx`，首次拉取需要配置调试器。
6. 在对应 Flash Download 设置中使用 **Erase Sectors** 并启用 Verify；保留现有 Bootloader，避免 Full Chip Erase。
7. 使用 **Download** 下载当前 App，之后复位并按下面步骤验收。App 内原有机械臂初始化会调用回原位，复位时需让机械臂具备回位空间。

本机是 Ubuntu，未运行 Keil；Windows 的 Keil 编译、下载和板上结果由实际执行确认。GCC 构建成功不等于 Keil/实车已经通过。

## Flash 和 RAM 布局

| 区域 | 起点 | 长度/范围 |
| --- | --- | --- |
| 现有 Bootloader（本目录不提供） | `0x08000000` | 16 KiB，保留 |
| App | `0x08004000` | `0x1BC00`，111 KiB |
| 元数据 | `0x0801FC00` | 1 KiB，保留 |
| RAM | `0x20000000` | `0x5000`，20 KiB |

`MDK-ARM/LeArm/LeArm.sct` 与 Keil IROM1 配置保持一致；`main.c` 的 VTOR 为 `0x08004000`。
这是依赖已有 Bootloader 的 App，不是从 `0x08000000` 启动的整片固件。
Windows 产物由 Keil 重新生成：`MDK-ARM/LeArm/LeArm.axf`、`.hex`、`.bin`；不将历史 bin 当作本次修改结果。

## Ubuntu 软件检查

在 RosCar 仓库根目录：

```bash
python3 scripts/check_stm32_project.py
python3 scripts/test_stm32_project.py
bash firmware/LeArm_APP_rtos/tests/run_safety_tests.sh
bash firmware/LeArm_APP_rtos/tests/run_health_tests.sh
python3 scripts/test_stm32_update.py

# 已安装 ARM GCC 时，默认从本目录读取完整 Keil 源列表：
python3 scripts/build_stm32.py --output build/stm32-startup-fixed
# 若工具链不在 PATH，增加 --toolchain-bin /实际工具链/bin
```

本次完整 ARM GCC 10.3.1 构建/链接通过：124 个编译单元；App 54,956 字节；链接 RAM 18,192/20,480 字节（包含配置的 heap/stack 预留，不是运行时余量）。
新 GCC bin SHA256：`fcdb1f5f5e1b837418083fcb0a59edaaca8e85fc6daf7194788dfe57c566d288`。
厂商静态库仍有 wchar_t ABI 警告，未修改其二进制；Keil 使用原工程 ABI。没有把 GCC 产物刷入下位机。

`tests/run_safety_tests.sh` 编译实际 `RobotSafety.c`、`SafetyManager.c`、`freertos.c`，通过模拟 CMSIS 验证任务创建前初始化、失败停止启动、未初始化拒绝、FAULT 不可清除、STOP/BOOT 事务等 8 组行为。此测试没有启动真实 RTOS 或访问硬件。
健康状态 10 组、128 字节 wire fixture 和协议 CRC 回归也通过。
静态检查校验 148 个 Keil 项目输入、RTE 配置、相对路径、分区及启动顺序；GCC 链接后还核对反汇编中的实际初始化调用顺序。
旧 `/mnt/data/LeArm_APP_rtos` 尚未补齐安全初始化，会被新构建检查拒绝。

## 烧录后验收

先停止树莓派上占用 STM32 串口的桥，只保留一个串口访问者，然后在树莓派运行只读检查：

```bash
cd ~/RosCar
git pull --ff-only origin main
python3 scripts/stm32_update.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --probe
```

预期 HEALTH_STATE 为 **128 字节**；硬件初始化成功后安全状态应到 **3（SAFE_IDLE）**，无需先发送运动指令。
若仍为 7，不修改状态值或绕过安全检查，查看新健康消息的当前/历史故障、任务心跳及 last_event 来定位真实原因。

恢复树莓派桥和上位机健康显示：

```bash
# 树莓派，一个终端
cd ~/RosCar/pi_ws
bash start.sh bridge

# Ubuntu 上位机，另一个终端
cd ~/RosCar/host_ws
bash start.sh health
```

确认健康时间戳/uptime推进、5任务心跳有效、底盘约50Hz、健康约10Hz，再检查轮速计和后续 EKF。
旧 `ubuntu_20261112` 和 `ubuntu_20261112_health_init` 两个 bin 均未补齐这次安全启动修复，不再用于本次升级。

## 依赖和来源

保留各文件内 ST、ARM、FreeRTOS、Hiwonder 和 U8g2 的版权/许可证；见 [../../THIRD_PARTY.md](../../THIRD_PARTY.md)。
`LeArm.lib` 是用户原工程随附的构建输入，仅保留原样，不重新许可或提供其闭源实现。
裁去未使用的 CMSIS DSP/NN/预编译库和文档，不影响当前 Keil 源列表。
`.ioc` 和历史 `docs/健康遥测.md` 保留用于溯源；CubeMX 重新生成后应重新检查业务文件列表、App 分区和初始化顺序。
