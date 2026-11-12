# 第三方与参考代码

- pi_ws/src/ldlidar 和 pi_ws/src/ldlidar_driver 来自用户提供的 LDLiDAR ROS 2/SDK 源码，保留 LDROBOT 版权及 MIT 声明；package.xml 声明 MIT。原厂说明和 USB udev 示例保存在 references/ldlidar_vendor。GPIO UART 不依赖 USB udev 规则。
- references/ICM20948_WE 是用户提供的 Arduino 库；library.properties 标明版本 1.0.6，上游为 https://github.com/wollewald/ICM20948_WE，保留 LICENSE、作者及示例。
- references/ICM20948_WE 中 bsp_imu.c、bsp_imu.h、icm_20948_reg.h 和 references/mecanum 中代码由用户提供，用于滤波、寄存器和底盘算法参考。它们依赖其他未包含的板级文件，不构成完整 STM32 固件。
- pi_ws/src/icm20948_ros2/LICENSE 保留原 MIT 许可；learm_bridge/package.xml 声明 Apache-2.0。未用仓库级许可证覆盖不同来源文件。
- host_ws/src/roscar_visualization 的 IMU 显示实现来自本仓库 icm20948_ros2/viewer.py，保留来源说明及原 MIT LICENSE；不包含硬件采集逻辑。
- ROS 系统依赖由软件包管理器安装，不复制系统安装目录和二进制文件。
- 用户于2026-11-12要求将完整App纳入仓库：firmware/LeArm_APP_rtos来源为用户提供的/mnt/data/LeArm_APP_rtos，包含ST HAL、CMSIS、FreeRTOS、Hiwonder与U8g2源文件、启动/链接脚本、Keil工程及必需的原厂Hiwonder/Arm/src/LeArm.lib。保留原文件版权及CMSIS、ST HAL、ST Device、FreeRTOS随附许可证；U8g2的许可保留在源文件头。LeArm.lib按用户原工程保留，未取得其实现源码，不用仓库其他许可证重新授权它或Hiwonder文件。
- 只导入当前App构建依赖，裁去未用CMSIS DSP/NN/预编译库、厂商大文档和生成产物。完整来源/入口见firmware/LeArm_APP_rtos/README.md；未复制系统工具链或Keil安装目录。Linux生成的链接脚本保留原ST声明。IAP工具调用用户树莓派已有stm32_ota/ymodem_sender.py，不复制其实现。
