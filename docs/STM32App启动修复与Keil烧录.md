# STM32 App 启动修复与 Keil 烧录

日期：2026-11-12。维护来源：Ubuntu 主机上传。

**代码修复已完成并纳入 `firmware/LeArm_APP_rtos`；软件测试和完整 ARM GCC 链接通过，Keil 编译、下载和板上效果尚未执行。**

## 原因与实测

树莓派被动接收10秒：底盘500帧（约50Hz）、健康99帧、CRC错误0；500个底盘状态均为FAULT=7、底盘fault_flags均0。编码器可解析，采集时车辆静止，编码器值未变化。另有1次COBS丢弃，接收记录中无序号丢失。

源码和原Keil产物都证实 `robot_safety_init()` 未被调用，链接器将其移除。`robot_safety_get_state()` 对未初始化访问返回FAULT并标记内部故障，所以通信/编码器采集仍可继续，而安全状态持续FAULT。底盘fault_flags是底盘任务字段，不包含安全模块所有内部故障；为零不能否认FAULT。

另一个缺陷是 `robot_health_init()` 漏调用：心跳检查判失败，快照函数未填充输出，而旧SerialTask继续发送局部栈内容，造成看似有变化的44字节健康报文、无效资源值和OFFLINE。CRC正确只能证明传输完整，不能证明这些字段有效。

## 改动清单

| 文件（相对 `firmware/LeArm_APP_rtos`） | 改动 |
| --- | --- |
| `Core/Src/main.c` | 在RTOS初始化前执行健康初始化（包含前一轮健康补丁） |
| `Core/Src/freertos.c` | 在内核Ready阶段、首个osThreadNew之前初始化安全模块；失败进Error_Handler |
| `Core/Src/RobotSafety.c`、`Core/Inc/RobotSafety.h` | 明确并检查Ready状态；禁止运行中重新初始化清除故障 |
| `Core/Src/RobotHealth.c`、`Core/Inc/RobotHealth.h` | 快照返回bool，未初始化/NULL失败 |
| `Core/Src/SerialTask.c` | 清零局部快照，仅成功取到快照才发送128字节扩展健康帧 |
| `tests/test_safety_startup.c`、stubs、`run_safety_tests.sh` | 使用实际源码测试启动顺序、故障保留与STOP/BOOT授权 |

仓库还新增完整Keil源文件/相对依赖、原厂库、RTE配置、App scatter及文档，过滤编译产物和个人调试配置。保留第三方版权与许可，不改麦轮算法或已有运动参数。

`scripts/check_stm32_project.py` 校验输入路径、分区、VTOR和源码启动顺序，`scripts/build_stm32.py` 默认使用仓库内App，链接后再检查反汇编中的调用顺序。未修复的外部副本会明确报缺少安全初始化，不再静默产出不完整bin。

## 验证结果

- Keil工程148个项目输入、RTE配置和include目录全部存在且位于App目录内。
- 安全/实际freertos启动回归8组通过；覆盖原缺陷复现、正常进入SAFE_IDLE、初始化失败不创建任务、非法上下文拒绝、已锁存故障不被初始化清掉、STOP双方确认与BOOT序号授权。
- 工程路径/启动防回归8项通过，含注释伪调用、缺少初始化、初始化过晚、忽略失败、越界Flash和工程外路径。
- 健康观测10组、128字节wire fixture、协议CRC和IAP离线5项测试通过。
- ARM GCC10.3.1完整124个编译单元及厂商库链接通过；反汇编确认 `health → kernel initialize → safety → workers → kernel start`。
- GCC App 54,956字节，RAM链接占用18,192/20,480字节；初始SP `0x20005000`，Reset `0x08011129`。
- GCC bin SHA256：`fcdb1f5f5e1b837418083fcb0a59edaaca8e85fc6daf7194788dfe57c566d288`；此哈希不用于要求Keil产物一致。

保留厂商库wchar_t ABI警告，未改厂商二进制。以上为Ubuntu软件验证，不是Keil/FreeRTOS板上调度或运动验收；没有刷写、复位下位机或发送运动指令。

## Keil 入口与验收

Windows拉取后打开 **`firmware/LeArm_APP_rtos/MDK-ARM/LeArm.uvprojx`**，目标LeArm，原配置Arm Compiler6.24；完整下载操作见[App README](../firmware/LeArm_APP_rtos/README.md)。

App仍从 `0x08004000` 开始，长度 `0x1BC00`，VTOR同步；保留前16KiB Bootloader及末1KiB元数据。不要用整片擦除替代App更新。此工程只提供App，不是Bootloader源码。

烧录后先被动 `stm32_update.py --probe`：应看到128字节健康报文；各硬件READY且无真实故障时应进入SAFE_IDLE=3。再启动树莓派桥与上位机health显示，核对快照/uptime推进、任务心跳及真实故障。

若仍FAULT，用新扩展健康字段定位，不把状态强改正常。owner和Arm query未实现仍应UNKNOWN。

此前54,624字节和54,752字节bin均不包含完整安全启动修复，不再使用；旧健康补丁只是历史记录。后续只维护仓库内App源码。
