# Ubuntu 本地编译 STM32 固件，经树莓派 IAP 下载

维护来源：Ubuntu主机上传。完整App现维护在RosCar仓库 `firmware/LeArm_APP_rtos`；本机仓库为 `/mnt/data/linuxproject/RosCar`，原下载目录保留作来源副本，不在 `/mnt/data` 父仓库提交。

## 当前完整修复与Keil入口

2026-11-12进一步确认 `robot_safety_init()` 也漏调用。仓库内App已补齐健康初始化、安全Ready阶段初始化及失败停止启动，新的GCC检查产物54,956字节，SHA256为 `fcdb1f5f5e1b837418083fcb0a59edaaca8e85fc6daf7194788dfe57c566d288`，位于 `build/stm32-startup-fixed`。

**下方曾SCP的54,624字节和54,752字节版本均不包含完整安全初始化，现已废弃。** 仅供历史溯源，不再刷写。新版本尚未SCP或刷入下位机。

用户本次用Keil/ST-Link更新，打开 `firmware/LeArm_APP_rtos/MDK-ARM/LeArm.uvprojx`。详见[App说明](../firmware/LeArm_APP_rtos/README.md)及[修复总结](STM32App启动修复与Keil烧录.md)。软件检查通过，Keil/板上尚未执行；旧固件缺少安全初始化会持续FAULT，不能靠重新上电或绕过RTOS升级拒绝解决。

## 2026-11-12 前两轮历史结果（以下两版产物均已废弃）

第二轮曾修复健康模块未初始化问题，生成54,752字节版本并传到树莓派；该版本仍漏安全初始化。历史 [health-startup.patch](../firmware/patches/health-startup.patch) 不包含本次完整修复，后续直接使用仓库内App，不单独应用旧补丁。

当时第二轮产物是 `/mnt/data/RosCar/build/stm32-health-init/LeArm.bin`，54,752字节，SHA256 `a4be9521d3ab535c3d3f043386a27ccc2aa17c6380ea3d3f182ec61c9400718c`；已传到树莓派 `~/stm32_ota/ubuntu_20261112_health_init/` 并校验一致。完整链接检查确认 `main` 在 `osKernelInitialize` 前调用 `robot_health_init`。仍未刷写或复位下位机，当前运行的仍是旧44字节固件。

下面54,624字节和旧目录记录仅保留首轮历史，不用于后续下载：

- Ubuntu 本地 ARM GCC 10.3.1 完整编译、链接成功：124个编译单元及原厂 `LeArm.lib`，不是仅编译健康测试对象。
- 新固件：`/mnt/data/RosCar/build/stm32/LeArm.bin`，54,624字节，SHA256为 `0e6a60d7712b060f30873a6635356b4e3a9dc8e4a2d43dd1db679674f19811e0`。
- 同时生成HEX、ELF、MAP、栈用量报告和输入文件哈希清单 `manifest.json`；这些构建产物不提交Git。
- 已SCP到 `admin@192.168.137.100:/home/admin/stm32_ota/ubuntu_20261112/`，树莓派SHA256一致，现有YMODEM下载器的 `--dry-run` 校验通过。
- 链接RAM占用18,192/20,480字节（含链接脚本预留的C堆及主栈）；FreeRTOS任务堆包含在静态区内，运行时任务栈仍需验收。
- 串口两次各被动接收3秒：每次底盘150帧，健康30帧，当前旧固件健康payload为44字节。两次均 `safety_state=7`（FAULT）；首次 `fault_flags=0x0010`（按当前源码定义为MOTOR），末次快照变为 `0x0000`，但安全状态仍是FAULT。电机命令四路均为零；零命令不能证明物理停车或I2C输出成功，故障位为零也不能证明锁存安全状态已恢复。
- **尚未刷写或复位下位机**。FAULT状态下软件升级握手会被固件拒绝；新工具也会提前拒绝。当时优先排查电机供电/I2C的建议不充分，后续已确认安全模块漏初始化可独立导致持续FAULT。旧44字节健康帧不能区分当前/历史故障，不凭这个故障位断言具体损坏位置。

## 本机编译

普通Ubuntu可安装依赖：

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi cmake ninja-build python3
cd /mnt/data/linuxproject/RosCar
python3 scripts/build_stm32.py --output build/stm32-startup-fixed
```

本次未修改系统安装：下载Ubuntu软件包并解包到用户工具链目录，因此实际使用：

```bash
cd /mnt/data/linuxproject/RosCar
python3 scripts/build_stm32.py \
  --output build/stm32-startup-fixed \
  --toolchain-bin "$HOME/.local/share/roscar-arm-toolchain/usr/bin"
python3 scripts/stm32_update.py --file build/stm32-startup-fixed/LeArm.bin
```

默认输出到仓库 `build/stm32`，可用 `--output` 指定独立目录。复用同一输出目录时保留同一编译器；切换工具链应选择新的输出目录。外部工程原来的Windows `build/Debug` 缓存不参与构建。

构建脚本从Keil工程读取C源文件、包含目录、宏和静态库，不使用原来不完整的CubeMX CMake业务源列表。GCC使用随附GNU启动文件，Keil保留ARMASM启动文件。生成目录内做两项兼容适配：`Kinematics.h`转到实际小写头文件；超声波驱动的静态数组长度从 `const static uint8_t` 改为同值enum常量，不改滤波窗口。这两项构建适配不改写源码；健康与安全启动修复直接维护在仓库内App，Keil也获得修复。

链接脚本保留ST原声明与GNU启动符号，Flash布局按Keil scatter核对：

| 区域 | 地址 | 容量 |
| --- | --- | --- |
| Bootloader | `0x08000000`～`0x08003FFF` | 16 KiB |
| App | `0x08004000`～`0x0801FBFF` | 111 KiB |
| 元数据 | `0x0801FC00`～`0x0801FFFF` | 1 KiB |
| RAM | `0x20000000`～`0x20004FFF` | 20 KiB |

原CubeMX脚本Flash起点是 `0x08000000`，不能直接当作IAP App布局使用。本工具核对App容量、初始SP、Thumb复位入口位于实际镜像内；链接区域会限制App越界。源工程已经设置VTOR为 `0x08004000`。

使用newlib-nano、nosys和现有sysmem；生成空 `_init/_fini` 满足GNU启动，不添加硬件初始化。链接仍有原厂 `LeArm.lib` 的2字节wchar_t与GCC/newlib的4字节wchar_t属性警告；检查的运动学接口没有wchar_t参数，但跨编译器静态库的板上运行尚未验收。还保留原厂未用参数、有符号/无符号比较等编译警告。没有把链接成功写成实车成功。

## 传输并核验

以下仅为后续传输示例，本次用户选择Keil，尚未传输这个新版本。Ubuntu执行：

```bash
ssh admin@192.168.137.100 'mkdir -p ~/stm32_ota/ubuntu_20261112_startup_fixed'
scp build/stm32-startup-fixed/LeArm.bin build/stm32-startup-fixed/LeArm.hex build/stm32-startup-fixed/manifest.json \
  admin@192.168.137.100:~/stm32_ota/ubuntu_20261112_startup_fixed/
sha256sum build/stm32-startup-fixed/LeArm.bin
ssh admin@192.168.137.100 \
  'sha256sum ~/stm32_ota/ubuntu_20261112_startup_fixed/LeArm.bin; python3 ~/stm32_ota/ymodem_sender.py --file ~/stm32_ota/ubuntu_20261112_startup_fixed/LeArm.bin --dry-run'
```

IP以现场配置为准；密码与密钥不写入仓库。`scp`只是复制文件，`--dry-run`只校验文件，都不会刷写STM32。

## 树莓派检查与升级

先停止占用同一串口的ROS桥和其他程序（保留它们的启动配置），然后在树莓派执行：

```bash
cd ~/RosCar
git status --short
git pull --ff-only origin main
fuser /dev/ttyUSB0
python3 scripts/stm32_update.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --probe
```

`--probe`仅接收，不发送命令，不复位。默认应用波特率115200。检查正常后，确认底盘和机械臂允许停止、复位；新固件 `robot_arm_init()` 会调用 `robot_arm_reset(2000)`，可能让机械臂回原位。

**下面命令会实际停止、复位并刷写，当前FAULT未处理时不要执行：**

```bash
cd ~/RosCar
python3 scripts/stm32_update.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --file ~/stm32_ota/ubuntu_20261112_startup_fixed/LeArm.bin --execute
```

默认 `--mode rtos`：发送STOP_ALL(0x12)，等待对应序列号的成功ACK；发送ENTER_BOOTLOADER(0x13)，payload携带刚完成的STOP序列号，再等成功ACK。任何超时/NACK立即停止，不重试运动命令，不自动切换旧协议或强制复位。随后调用树莓派原有 `/home/admin/stm32_ota/ymodem_sender.py` 的YMODEM传输，等待 `BL:OK`；可用 `--sender` 指定该脚本位置。需要 `python3-serial`，不需要安装ROS才能刷写。

旧下载器的 `--request-update` 发的是原应用 `55 55 06 1A 42 4C 44 52`，不能进入新RTOS固件的Bootloader。只有确认仍运行旧协议时才用本工具 `--mode legacy --app-baud 9600`；只有确认下位机已经进入自定义Bootloader时才用 `--mode bootloader`。这两个模式不能用于绕过RTOS故障；进入自定义Bootloader的物理步骤需要匹配实际Bootloader工程，不能把芯片ROM的BOOT0模式当成此YMODEM程序。

传输成功后再次 `--probe`，确认新健康payload为128字节、底盘数据连续、安全状态与故障值正确，再启动树莓派ROS桥并查看上位机 `/diagnostics`。`BL:OK`只证明Bootloader接受传输，不等于新应用、里程计、SLAM已经验收。

## 本次验证范围

- 全固件ARM编译与链接、Flash/RAM区域、向量表与FreeRTOS强定义SVC/PendSV/SysTick检查通过。
- 外部固件9组健康观测测试、128字节wire fixture、CRC计数/恢复测试通过。
- 新工具5项离线测试在本机和树莓派均通过：镜像边界、STOP→BOOT顺序与同一token、NACK拒绝、无关ACK拒绝、Boot拒绝不回退。测试中打印的握手成功来自模拟串口，与实机检查分开。
- Pi现有YMODEM的12项测试在本机和树莓派均通过。
- 树莓派SCP哈希与两种工具的镜像校验通过，实串口被动接收通过；树莓派RosCar已更新到包含工具的新提交。没有执行实际YMODEM升级，也没有验证GCC新固件在板上运行。
