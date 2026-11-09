# ICM-20948：树莓派 Linux I²C / ROS 2 Humble

本包根据工作区 `references/ICM20948_WE/src/ICM20948_WE.{h,cpp}` 的寄存器定义、
初始化及换算方式编写 Linux 读取实现，不依赖 Arduino `Wire`，原库 MIT 许可保留于 LICENSE。
姿态估计参考你提供的 `references/ICM20948_WE/bsp_imu.c`：软件低通、六轴 Mahony 比例/有限积分反馈，
配合独立静止陀螺仪偏置校准，未使用 DMP。

## 本次接线与总线

你确认的接线是树莓派 **物理引脚 3 / 5**，对应：

| 模块信号 | 树莓派物理引脚 | GPIO |
| --- | --- | --- |
| SDA | 3 | GPIO2 / SDA1 |
| SCL | 5 | GPIO3 / SCL1 |
| GND | 例如 6 | GND |

因此默认使用 `/dev/i2c-1`，不是 I²C0。供电按模块规格连接：裸 ICM-20948 的 VDDIO
约为 1.8V，直连树莓派需要模块本身具备合适的电平转换，或外加转换器。
不要仅凭模块供电范围推断 SDA/SCL 可接受的电平。

在**树莓派** `/boot/firmware/config.txt` 的 `[all]` 区域确保存在：

```ini
[all]
dtparam=i2c_arm=on
```

只在尚未启用时修改；已有的雷达 `enable_uart=1` 和 `dtoverlay=disable-bt` 保留。
如果修改了启动配置，重启后再操作。

```bash
sudo apt update
sudo apt install i2c-tools python3-smbus python3-colcon-common-extensions \
  ros-humble-robot-localization ros-humble-tf2-ros
sudo modprobe i2c-dev
sudo usermod -aG i2c "$USER"
```

添加用户组后退出 SSH 并重新登录，检查：

```bash
groups
ls -l /dev/i2c-1
# 只在 IMU 节点停止时探测目标地址；不要同时运行其他 IMU 读取程序。
i2cdetect -y 1 0x68 0x69
```

预期 `0x68` 或 `0x69` 有响应。若均为 `--`，先检查总线、模块地址选择及供电。
若启动后仍没有 `/dev/i2c-1`，检查 `i2c-dev` 是否加载；需要时可以在
`/etc/modules-load.d/i2c-dev.conf` 写入一行 `i2c-dev` 以便重启自动加载。

## 从笔记本复制到树莓派

在**笔记本本机终端**执行（目标 IP 是本次统一的固定地址）：

```bash
ssh admin@192.168.137.100 'mkdir -p ~/RosCar/pi_ws/src'
scp -r ~/RosCar/pi_ws/src/icm20948_ros2 \
  admin@192.168.137.100:~/RosCar/pi_ws/src/
```

源码路径以你笔记本的实际位置为准。仅复制本包源码，树莓派本地编译。

## 编译与启动

在**树莓派**执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/RosCar/pi_ws
colcon build --symlink-install --packages-select icm20948_ros2
source ~/RosCar/pi_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch icm20948_ros2 imu.launch.py
```

默认地址为 `0x68`。若扫描结果是 `0x69`：

```bash
ros2 launch icm20948_ros2 imu.launch.py address:=105
```

总线也可以通过 `bus:=1` 修改。更改其余参数时编辑 `config/imu.yaml`，重启节点；
这些参数在初始化时读取，运行中设置参数不会重配置传感器。

**启动时保持整个机器人静止至少 3 秒**。节点将检查角速度与加速度的稳定程度，
移动时重新采集校准窗口，30 秒内无法完成则退出。持续缓慢匀速旋转无法可靠地与
陀螺仪偏置区分，因此不能用软件检查代替实际静止。

自动校准仅估计陀螺仪偏置，不将静止时的重力当成加速度偏置。若需要加速度计精度校准，
应另外进行六面标定。模块预热后校准通常更稳定。

## 发布接口

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 偏置校正后的角速度与加速度，未做软件低通，无姿态 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 软件低通后的角速度与加速度，加上六轴融合姿态四元数 |
| `/imu/temperature` | `sensor_msgs/msg/Temperature` | 芯片内部温度，约 1 Hz |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 校准、读取错误、发布频率等 |

IMU 话题使用 SensorDataQoS，默认约 100 Hz。硬件使用约 102.27 Hz ODR、
±4g 加速度量程、±500°/s 陀螺仪量程，默认加速度数字低通约 11.5 Hz、陀螺仪约 51 Hz。
读取数据就绪标志后连续读取 14 字节，不重复发布未更新的寄存器值。
这是轮询采集，没有 FIFO 或硬件时间戳，系统过载时会漏读中间样本。
时间戳来自树莓派 ROS 系统时钟，姿态积分使用单调时钟；总线读取失败不发布虚构数据，
连续 20 次读取失败或 2 秒没有新样本时退出。

单位和语义：

- 加速度 `m/s²`、角速度 `rad/s`、温度 `°C`。
- 加速度**保留重力**；水平且模块 Z 向上时，Z 应约为 `+9.81 m/s²`。
- `frame_id=imu_link`，向量保持模块加速度计/陀螺仪的轴向。
- 四元数将模块坐标旋转到 Z 向上的世界坐标。偏航零点随启动设置，
  **没有地理东向/北向参考，不是绝对 ENU 航向**。
- 六轴融合用重力约束横滚和俯仰，偏航仍有积分漂移；强动态加速度也可能干扰倾角。
- 默认不使用磁力计，避免未经标定的室内磁场直接影响航向；本包不发布磁场数据。
- 原始消息 `orientation_covariance[0]=-1`；融合姿态的 yaw 方差设为大值，
  EKF 模板不使用这个相对偏航角。其余协方差是可配置的初始估计，不是实验标定结果。
- IMU 采集节点不发布 TF，以免和底盘、EKF 或机器人模型冲突。

## 验证

在另一个终端加载同样的环境：

```bash
source /opt/ros/humble/setup.bash
source ~/RosCar/pi_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /diagnostics --once
```

依次进行：

1. 静止时角速度接近 0，三轴加速度模长接近 9.81，四元数模长接近 1。
2. 模块 Z 轴向上时，从上向下看逆时针转动，`angular_velocity.z` 应为正。
3. 倾斜模块后姿态应平滑响应，停止后稳定。水平旋转一圈用于观察 yaw 积分，
   不应期待长期保持绝对航向。
4. 雷达同时运行时，检查 `/scan` 仍约 10 Hz、IMU 约 100 Hz，诊断没有持续错误。

如果 `hz` 正常，但之后 `echo` 提示话题不存在，先在保持采集 launch 运行的情况下，
于第二个已加载同样 ROS/DDS 环境的终端执行：

```bash
ros2 node list
ros2 topic info /imu/data --verbose
ros2 topic echo /imu/data sensor_msgs/msg/Imu --once
```

如果 `/icm20948` 不存在或发布者为 0，查看采集 launch 终端是否退出及具体错误。
`Ctrl+C` 在 `topic hz` 终端只会停止频率检查；在采集 launch 终端会停止采集。
显式指定消息类型只能跳过类型推断，不能恢复已退出的发布节点。

## 参考代码滤波参数

你的 `bsp_imu.c` 虽然读取磁力计，但 `imu_update_attitude` 没有使用磁力计，仍是六轴姿态。
本实现保留 X/Y 重力反馈、有限积分，使用精确归一化与实际单调时钟积分。
保留原始测量话题，便于比较滤波前后输出。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `gyro_lpf_alpha` | 0.8 | 陀螺仪软件低通旧值权重 |
| `accel_lpf_alpha` | 0.7 | 加速度软件低通旧值权重 |
| `lpf_reference_dt` | 0.02 s | 定义上述权重的参考周期 |
| `fusion_gain` | 0.8 | 重力比例反馈 |
| `fusion_integral_gain` | 0.0001 | 有限积分反馈系数 |
| `fusion_integral_limit` | 2.0 | 按参考周期归一化的误差累积限幅 |
| `gyro_z_deadband` | 0.0 rad/s | 可选 Z 轴死区，默认保留慢速转动 |

每次低通使用 `alpha_actual = alpha_reference ** (dt / reference_dt)`，
从而在 100 Hz 时保持与参考周期相近的平滑时间尺度。积分累积也按 `dt/reference_dt` 缩放。
低通增加延迟：默认陀螺仪软件低通时间常数约 90 ms，加速度约 56 ms，
还有硬件低通延迟。动态导航时若响应偏慢，应减小 alpha，再根据实测噪声调节协方差。

参考代码的 Z 死区 `0.015 rad/s` 约为 `0.86°/s`；照搬会将低于此值的真实转动压成零，
因此默认关闭。需要复现时可设 `gyro_z_deadband: 0.015`，但必须验证慢速旋转里程计。
传感器量程保持本节点的 ±4g/±500°/s，按其对应比例换算，不照搬参考固件的不同量程系数。
参考文件里的磁力计偏置属于特定安装，不作为当前模块的通用标定值。

## 笔记本 RViz 查看姿态

树莓派保持 `imu.launch.py` 运行。笔记本使用相同的 Domain ID 和现有 DDS 配置。
本包提供姿态显示适配节点：订阅 `/imu/data`，输出 RViz 内置 MarkerArray 和 PoseStamped。
不需要 IMU 显示插件，不会在笔记本打开 I²C 或启动第二个采集驱动。

在**笔记本**执行：

```bash
sudo apt install ros-humble-rviz2
source /opt/ros/humble/setup.bash
cd ~/RosCar/pi_ws
colcon build --symlink-install --packages-select icm20948_ros2
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
# 如果此前使用自定义 RMW/CycloneDDS，继续加载相同的有线通信配置。
ros2 topic echo /imu/data sensor_msgs/msg/Imu --once
ros2 launch icm20948_ros2 viewer.launch.py
```

可视化中灰色长方体随模块姿态转动，红箭头为 X、绿箭头为 Y、蓝箭头为 Z。
它显示模块姿态，不显示机器人位置或安装在车身上的位置。

显示参考 `imu_view` 是虚拟 Z-up 世界，偏航参考与节点启动一致。
显示节点只发布独立的 `imu_view → imu_view_reference` 单位静态变换，
用于注册 RViz 固定坐标系；不发布或覆盖真实 `base_link → imu_link` / 里程计 TF。
标记 0.3 秒后过期，避免断流时一直保留最后一帧。

如果已有 RViz 窗口，可以只启动适配节点：

```bash
ros2 launch icm20948_ros2 viewer.launch.py rviz:=false
```

然后将 RViz `Fixed Frame` 设为 `imu_view`，添加 `MarkerArray`，话题选 `/imu/markers`。
这是独立 IMU 调试视图；查看 SLAM 地图时，使用原来的地图 RViz 窗口和 `map` 固定坐标系。

## 接入轮式里程计与 slam_toolbox

```text
ICM20948 → /imu/data ─────────────┐
                                 ├→ robot_localization → /odometry/filtered
STM32编码器 → learm_bridge → /odom ┘                     + odom → base_link TF
雷达 → /scan ──────────────────────→ slam_toolbox → /map 和 map → odom TF
```

**slam_toolbox 不直接订阅 IMU 来完成上述融合。** 它使用扫描和 TF 中的里程计位姿；
本包的 `config/ekf.yaml` 已对齐 `learm_bridge`。仅有 IMU 不能提供可靠的平面平移里程计。

因为你当前还没有确定模块相对车头方向，先只启动 IMU 节点。
在最终接入前应完成：

1. 根据实际安装测量 `base_link → imu_link` 和 `base_link → laser`。
   ROS 机器人坐标约定 X 前、Y 左、Z 上。用 URDF 或静态 TF 描述模块实际方向，
   不要用单位变换掩盖未知的安装方向。若已有 `robot_state_publisher` 发布，勿重复发布。
2. 底盘提供 `/odom`，`header.frame_id=odom`、`child_frame_id=base_link`。
   更新后的桥接节点提供正的速度协方差，并用 STM32 毫秒采样时间计算速度；
   ROS 消息时间戳仍使用树莓派 ROS 时钟，不能把设备启动毫秒数当 ROS 时间。
3. 使用 EKF 后，底盘节点的 `publish_tf=false`，交由 EKF 唯一发布 `odom → base_link`。
4. 运行 EKF，检查 `odom → base_link → laser/imu_link` 连通，
   最后接入 slam_toolbox；其 odom/base/scan 设置应与实际 TF 和 `/scan` 一致。

静态变换命令形式（所有数值都应由实际安装确定，角度为弧度）：

```text
ros2 run tf2_ros static_transform_publisher \
  --x 实测X --y 实测Y --z 实测Z \
  --roll 实测横滚 --pitch 实测俯仰 --yaw 实测偏航 \
  --frame-id base_link --child-frame-id imu_link
```

`ekf.yaml` 融合 `/odom.twist` 的轮式 `vx、vy、wz` 和 `/imu/data` 的 IMU Z 角速度。
不融合轮式积分位姿，避免重复使用同一组编码器信息；不融合 IMU 相对偏航和加速度，
也不添加 `vy=0` 约束。输入 `/odom` 和输出 `/odometry/filtered` 必须保持不同话题。
轮式角速度与 IMU 角速度按各自协方差加权，默认轮式角速度方差更大。
桥接节点的 `twist_covariance_diagonal` 顺序为 `[vx,vy,vz,wx,wy,wz]`，
默认 `[0.0025,0.01,1e6,1e6,1e6,0.04]`，是待实测调整的初始方差，不能当作标定结果。
原始 `/odom.pose` 的协方差仍未建模，当前 EKF 不使用该位姿。
桥接实际编码器映射和轮序见 [MECANUM.md](MECANUM.md)。

先安装依赖并编译（两个包都放在 `~/RosCar/pi_ws/src/`）：

```bash
sudo apt install ros-humble-robot-localization python3-serial
source /opt/ros/humble/setup.bash
cd ~/RosCar/pi_ws
colcon build --symlink-install --packages-select learm_bridge icm20948_ros2
source install/setup.bash
```

保持 `imu.launch.py` 和实际安装 TF 的发布节点运行，停止旧串口桥/轮速监视启动，
在新终端启动串口桥和 EKF（此启动文件自动设置桥接 `publish_tf=false`，不重复启动 IMU）：

```bash
source /opt/ros/humble/setup.bash
source ~/RosCar/pi_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch icm20948_ros2 chassis_ekf.launch.py
```

串口路径不同时，加 `port:=/dev/ttyUSB0` 或实际稳定路径。不要同时运行另一个串口桥。
若已有桥接节点，可使用以下独立启动方式替代组合启动：

```bash
# 桥接节点必须以 publish_tf:=false 启动
ros2 launch learm_bridge serial_bridge.launch.py publish_tf:=false
# 在另一个终端
ros2 launch icm20948_ros2 ekf.launch.py
```

在另一个使用相同 ROS 环境的终端检查：

```bash
ros2 topic echo /odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

RViz 查看原始/融合轨迹时固定坐标系设 `odom`，添加两个 Odometry，分别选择 `/odom`
和 `/odometry/filtered`；有地图后再使用 `map`。slam_toolbox 的 `base_frame` 设为
`base_link`、`odom_frame` 设为 `odom`。如果后续使用 `base_footprint`，应同步修改
机器人 TF 和 EKF 配置。系统中的其他里程计话题、底盘坐标系不同，可自定义 YAML：

```bash
ros2 launch icm20948_ros2 ekf.launch.py params_file:=/绝对路径/你的ekf.yaml
```

## 本地验证与限制

```bash
source /opt/ros/humble/setup.bash
cd ~/RosCar/pi_ws
colcon test --packages-select icm20948_ros2
colcon test-result --verbose
```

包含寄存器模拟、单位换算、错误数据处理、姿态积分及收敛、静止校准、
真实 ROS 发布/订阅配合模拟传感器的测试。对接时两个包共 33 项测试通过，
并使用实际 robot_localization 节点及当前 YAML 做模拟输入验证：
轮式三自由度速度和 IMU 角速度成功融合、输出约 30Hz、原始积分位姿被忽略，
且收到 `odom → base_link` TF。测试通过不代表实际 I²C 接线、安装方向、
轮速尺度和滑移已验证，仍需在树莓派硬件上进行上述检查。
