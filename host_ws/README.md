# 上位机显示工作区 host_ws

执行主机：**Ubuntu 22.04 Desktop + ROS 2 Humble 上位机**。该工作区只包含 `roscar_visualization` 显示包，不包含串口、I2C、雷达采集、里程计或 EKF。树莓派负责采集、处理与 ROS 发布，上位机通过 DDS 订阅并显示。

## 1. 获取并编译（Ubuntu Desktop 本地终端）

以下假设仓库位于 `~/RosCar`，按实际位置替换；目录名包含空格时需保留引号。已有仓库先检查并保存本地改动，再拉取：

```bash
cd ~/RosCar
git status
git pull --ff-only origin main
```

尚未克隆时使用：

```bash
git clone https://github.com/JTwilliam/RosCar.git ~/RosCar
```

已有 ROS Humble 环境。依赖缺失时在上位机手动安装，脚本不自动安装：

```bash
sudo apt install build-essential python3-colcon-common-extensions \
  ros-humble-ament-cmake ros-humble-rviz2 ros-humble-rclpy \
  ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-tf2-ros \
  ros-humble-visualization-msgs ros-humble-diagnostic-msgs
```

编译显示工作区：

```bash
cd ~/RosCar
bash host_ws/build.sh
```

脚本自动加载 `/opt/ros/humble/setup.bash`，仅搜索 `host_ws/src`，使用独立路径：

```text
host_ws/build/host_runtime
host_ws/install/host_runtime
host_ws/log/host_runtime
```

它等价于在 `host_ws` 中执行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/host_runtime build \
  --base-paths src \
  --build-base build/host_runtime \
  --install-base install/host_runtime \
  --packages-select roscar_visualization
```

不要复制 WSL 或其他机器的 build/install 目录；上位机不需要编译 `pi_ws`、`lidar_ws` 或安装 `icm20948_ros2` 来显示。

## 2. 设置双机网络环境

在上位机创建本地配置，已有文件时不覆盖：

```bash
cd ~/RosCar
cp -n host_ws/host.env.example host_ws/host.env
nano host_ws/host.env
```

两端采用相同 `ROS_DOMAIN_ID`，跨机器传输应使用 `ROS_LOCALHOST_ONLY=0`；保留两端已验证的 RMW/DDS 配置。配置中的路径必须是上位机本机路径，不是树莓派路径。脚本不修改网卡、IP、防火墙、DDS配置文件或系统时钟。

环境配置默认是 `host_ws/host.env`；可用 `ROSCAR_CONFIG=/绝对路径/配置文件` 覆盖。启动脚本明确加载 `host_ws/install/host_runtime/local_setup.bash`，避免误用旧的 pi_ws 安装结果。ROS日志默认写入 `host_ws/log/ros`。

## 3. 启动主 RViz

在 **Ubuntu Desktop 本地桌面终端**执行，不要 SSH 到树莓派后执行：

```bash
cd ~/RosCar
bash host_ws/start.sh rviz --dry-run
bash host_ws/start.sh rviz
```

不写模式时默认 `rviz`。主入口只运行 RViz，默认配置位于包内 `rviz/desktop.rviz`：

- Fixed Frame：`odom`。
- 显示 `/scan`、`/odom`、`/odometry/filtered` 和 TF。
- LaserScan 使用 Best Effort QoS。

所有传感器节点、`odom → base_link` 和机器人安装 TF 都由树莓派发布；上位机不重复发布。缺少树莓派数据或TF时，RViz出现等待数据提示是预期现象。停止使用 Ctrl+C 或关闭 RViz。

可传递 launch 参数选择自己的配置：

```bash
bash host_ws/start.sh rviz 'rviz_config:=/home/user/My Config/custom.rviz'
```

## 4. 可选 IMU 姿态视图

```bash
cd ~/RosCar
bash host_ws/start.sh imu-view --dry-run
bash host_ws/start.sh imu-view
```

该模式启动包内 `imu_viewer` 和独立 RViz，订阅树莓派的 `/imu/data`，用方块和红绿蓝坐标轴显示启动相对姿态。它只发布 `/imu/pose`、`/imu/markers` 和隔离的示意参考 TF `imu_view → imu_view_reference`，不修改 `base_link → imu_link` 或里程计，不参与融合。

指定其他姿态话题：

```bash
bash host_ws/start.sh imu-view imu_topic:=/imu/data
```

`--dry-run` 会检查本地配置、安装环境和包可用性并打印命令，不启动GUI或节点。需要先完成构建；没有 `host.env` 时会明确提示配置缺失。

## 5. 纯订阅健康终端（无需图形桌面）

在上位机工作区完成构建、配置host.env后：

```bash
cd ~/RosCar/host_ws
bash start.sh health --dry-run
bash start.sh health
# 可选阈值，ROS参数形式：
bash start.sh health --ros-args -p stale_after_s:=0.5 -p offline_after_s:=2.0 -p refresh_hz:=5.0
```

该模式只检查显示包、rclpy、diagnostic_msgs，不要求DISPLAY或rviz2，不打开串口、不依赖serial、不发送控制。订阅 `/diagnostics`，只接受名称为 `learm/health/link`、`chassis`、`arm`、`communication`、`safety`、`resources`、`tasks`（后六项同样带 `learm/health/` 前缀）的诊断。桥尚未提供这些诊断时显示未知和OFFLINE；普通桥诊断或其他设备消息不会让健康链路变成ONLINE。

Link状态同时检查两个单调时间年龄：桥link诊断停止更新的年龄，以及桥报告health_age_s加上自收到该诊断经过的时间。任一达到0.5s显示STALE，达到2s显示OFFLINE；桥明确上报OFFLINE/STALE时也直接降级，link诊断等级STALE不显示在线。收到新OK诊断也不能掩盖过期的health_age_s。没有health_age_s时新诊断显示UNKNOWN，随后仍按桥诊断年龄降级。ONLINE只代表报文新鲜，不代表安全允许运动或无故障。

显示固件、uptime、安全状态、owner/event、当前故障与历史故障、最近故障、各组件健康等级/detail。安全状态与诊断健康等级分开显示；例如arm的IDLE只是detail，不会被监视器推测成故障。tasks的heartbeat_age_ms/stack_free_words按已知固件顺序Default、Chassis、Serial、Arm、Health显示；若有五项task_names则优先采用。未提供阈值或字段就显示未知，不猜测。链路或组件自身诊断过期时，组件健康显示STALE，避免保留旧OK造成误解。全部五个栈值有效时才显示全体最小值；缺项时只附已知项最小值并保持全体最小为未知。

仅TTY清屏，非TTY最多每秒一个完整文本快照，Ctrl+C正常退出。stale_after_s、offline_after_s、refresh_hz必须有限且为正，offline_after_s必须大于stale_after_s。纯逻辑单元测试可直接运行，无需ROS导入：

```bash
python3 src/roscar_visualization/test_health_monitor.py
```

## 6. 包结构与来源

```text
host_ws/
├── build.sh
├── start.sh
├── host.env.example
└── src/roscar_visualization/
    ├── CMakeLists.txt
    ├── package.xml
    ├── LICENSE
    ├── launch/viewer.launch.py
    ├── launch/imu_view.launch.py
    ├── rviz/desktop.rviz
    ├── rviz/imu.rviz
    ├── scripts/imu_viewer.py
    └── scripts/health_monitor.py
```

`desktop.rviz` 来源为仓库原 `config/desktop.rviz`；`imu.rviz` 与 `imu_viewer.py` 来源为 `pi_ws/src/icm20948_ros2` 的原显示实现。完整复制显示逻辑，仅增加脚本shebang、独立执行入口和来源说明，未引入传感器包依赖。原包 MIT 许可证已复制到显示包的 `LICENSE` 并随安装保留。

健康监视完整协议、阈值和固件更新见[健康遥测与监视](../docs/健康遥测与监视.md)。本机WSL已构建并完成显示/健康单测及实际桥+监视进程E2E；双机通信验证应按目标主机记录。本工作区未声称进行上位机现场GUI或树莓派硬件验证。
