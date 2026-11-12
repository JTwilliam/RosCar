# RosCar

树莓派4B、STM32F103RBT6、麦克纳姆底盘、STL06N雷达、ICM-20948与ROS2 Humble项目。维护来源：Ubuntu主机上传。

## 拉取后应该进入哪个目录

| 执行机器 | 工作区 | 编译与启动指导 | 职责 |
| --- | --- | --- | --- |
| Ubuntu22.04 Desktop上位机 | `host_ws` | **[上位机指令](host_ws/README.md)** | RViz、可选IMU显示，不访问硬件 |
| Raspberry Pi4B / Ubuntu22.04 / Humble | `pi_ws` | **[树莓派指令](pi_ws/README.md)** | STM32、IMU、雷达采集，里程计、EKF、机器人TF |
| 笔记本WSL学习环境 | `pi_ws`或`host_ws` | 按各自README的模拟部分 | 本机编译测试，不代表ARM或实车验证 |

两台机器各自拉取整个Git仓库，再进入自己的工作区。不要在仓库根目录直接colcon build，不要共享或复制build/install。原lidar_ws源码已并入pi_ws，旧目录只作迁移提示。

## 上位机常用命令（仅Ubuntu桌面执行）

```bash
cd ~/RosCar
git status --short
git pull --ff-only origin main
cd host_ws
bash build.sh
cp -n host.env.example host.env
# 按本机DDS配置编辑host.env
bash start.sh --dry-run
bash start.sh
```

## 树莓派常用命令（仅树莓派执行）

```bash
cd ~/RosCar
git status --short
git pull --ff-only origin main
cd pi_ws
bash build.sh
cp -n pi.env.example pi.env
# 必须填实测安装TF及正确设备路径/DDS设置
nano pi.env
bash start.sh all --dry-run
bash start.sh all
```

首次依赖安装、分模块启动及检查命令分别放在工作区README。旧scripts/robot.env不自动搬移，请保留其中实测值并复制到新配置。新构建输出分别使用install/host_runtime与install/pi_runtime，避免旧CMake缓存；脚本按自身位置定位，不要求固定用户名或绝对仓库路径。

## 源码布局

```text
host_ws/
  README.md  build.sh  start.sh  host.env.example
  src/roscar_visualization/   纯显示包、RViz与IMU显示
pi_ws/
  README.md  build.sh  start.sh  pi.env.example
  scripts/pi_system.launch.py
  src/learm_bridge_cpp/       C++串口桥、麦轮里程计与诊断
  src/learm_bridge/           Python回退桥和监视器
  src/icm20948_ros2/          IMU采集、滤波与EKF配置
  src/ldlidar/                雷达ROS包
  src/ldlidar_driver/         相邻SDK源码，非独立ROS包
scripts/                     共用函数、兼容入口与编排测试
references/                  厂商/原固件参考，不能当完整STM32工程烧录
```

## 数据和坐标约定

STM32原始轮序FR、RR、FL、RL。桥输出`/odom`的vx、vy、wz，保留麦轮横移；IMU输出`/imu/data`，EKF只融合其wz，输出`/odometry/filtered`。只有EKF发布odom→base_link；传感器安装TF使用实测值。雷达`/scan`约10Hz，IMU约100Hz，EKF约30Hz是历史用户反馈，不等于精度验收。

两个主机通过DDS传输，ROS_DOMAIN_ID一致、ROS_LOCALHOST_ONLY=0。网络地址按Ubuntu现场配置，不能把旧Windows共享网络192.168.137.x当作必然。各自主机保留已验证的RMW/DDS设置和时间同步。

## 健康遥测与实时监视

FreeRTOS下位机HEALTH_STATE与底盘数据复用同一串口；树莓派C++桥发布标准`/diagnostics`，上位机在host_ws运行 `bash start.sh health`，Python只订阅ROS话题，不打开串口。树莓派新增 `bash start.sh bridge` 最小模式（不依赖IMU/EKF/雷达）。当前/历史故障分开，健康快照或桥诊断停止均自动STALE/OFFLINE，急停与Arm空闲不会被当作软件崩溃。

跨端说明与固件本地工程范围见[健康遥测与监视](docs/健康遥测与监视.md)。下位机源码位于独立本地工程，不随本仓库拉取。

Ubuntu现已支持从外部 `/mnt/data/LeArm_APP_rtos` 完整编译IAP应用bin，通过SCP传到树莓派，复用现有YMODEM下载器；新增RTOS STOP/BOOT握手工具。命令、分区及当前实机故障记录见[STM32本地编译与IAP](docs/STM32本地编译与IAP.md)。2026-11-12已编译/传输/核验，当前下位机FAULT且有MOTOR故障位，尚未实际刷写。

同日OFFLINE排查找到固件漏调用健康初始化，造成有效CRC报文内发送未初始化栈内容；旧54,624字节产物也受影响。已修复外部源码并保存[可复现补丁](firmware/README.md)，修复版54,752字节已传到树莓派`ubuntu_20261112_health_init`，尚未刷写。健康显示现区分帧接收与快照年龄，具体原因和未知字段边界见[健康排查](docs/健康遥测与监视.md)。

## 当前迁移状态与验证

C++底盘桥已有编码器/odom/可选TF、非阻塞串口、重连和兼容诊断；IMU仍为Python，雷达本来就是C++，EKF复用robot_localization。[C++说明](pi_ws/src/learm_bridge_cpp/README.md)、[迁移路线](docs/C++迁移路线.md)、[验证范围](docs/进展与验证.md)。

部署总览见[双机部署](docs/双机部署与启动.md)，[运行手册](docs/运行手册.md)只索引两端指令。第三方来源见[THIRD_PARTY.md](THIRD_PARTY.md)。维护时检查本地修改，验证后按约定提交并正常推送，详见[维护说明](docs/维护说明.md)。
