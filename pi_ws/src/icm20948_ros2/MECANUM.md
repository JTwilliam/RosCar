# 与提供的麦轮底盘代码对接

核对来源：仓库 `references/mecanum/mecanum_chassis.c`、`mecanum_chassis.h`，
以及新提供的 `pi_ws/src/learm_bridge/learm_bridge/odometry.py`、`serial_bridge_node.py`。

## 当前 EKF 使用的桥接算法

实际桥接使用原工程的有符号累计编码器顺序 `[FR, RR, FL, RL]`。
这组原始编码器符号不能直接代入下文要求“轮子向前滚动为正”的标准轮速公式。
设每帧计数差为 `d0,d1,d2,d3`，每 tick 米数为 `s=2*pi*r/counts_per_revolution`：

```text
dx_body = (d0 - d1 - d2 + d3) * s / 4
dy_body = (d0 + d1 + d2 + d3) * s / 4
dyaw    = (d0 + d1 - d2 - d3) * s / (2 * (wheel_base + track_width))
vx = dx_body / dt; vy = dy_body / dt; wz = dyaw / dt
```

`wheel_base` 和 `track_width` 在此为完整轴距、轮距。
默认 `r=0.048m`、轴距 `0.190m`、轮距 `0.205m`、每圈计数 `5740.25304043815`，
每 tick 约 `0.00005254m`；这些是桥接现有参数，仍需实车校准。
速度 dt 使用 STM32 的采样毫秒时间差，避免一次 USB 读取多帧时产生速度尖峰。
ROS 消息 header 使用树莓派 ROS 时间，与 IMU 使用同一时钟。

桥接 `/odom` 的速度位于 `base_link`，位姿位于 `odom`。
EKF 读取三项速度和 IMU Z 角速度，输出 `/odometry/filtered` 及 `odom → base_link`。
只让 EKF 发布该 TF；原始积分位姿不参与本次融合。

## 已确认的轮序

按代码中的注释（俯视）：

| 数组项 | 接口 | 轮子 |
| --- | --- | --- |
| `motor_speed[0]` | M1 | 右前 FR |
| `motor_speed[1]` | M2 | 右后 RR |
| `motor_speed[2]` | M3 | 左前 FL |
| `motor_speed[3]` | M4 | 左后 RL |

该文件实现麦轮的方向/速度/旋转混控，输入是：

```c
mecanum_chassis_run(uint16_t angle, uint8_t speed, int8_t rot, bool drift);
```

它不是接受 SI 单位 `vx, vy, wz` 的 ROS 底盘接口，也没有编码器里程计。
`set_motor_speed()` 的实际单位、内部符号、PID 和编码器约定不在当前文件内。
之前仅凭此文件无法确定轮速比例；现在编码器反馈和尺度由新提供的 `learm_bridge` 实现。

## 与 Nav2 对接前需处理

- Nav2 输出 `cmd_vel.linear.x`、`linear.y`，单位 m/s；`angular.z` 单位 rad/s。
- 保留横向速度，不能采用差速底盘 `vy=0` 约束。
- 该代码在 `rot != 0` 时将平移和旋转一起乘以 0.5，会在开始旋转时改变平移速度。
  对 ROS 速度接口，应按目标 twist 直接解算，超限时统一比例缩放各轮速度。
- `drift=true` 的分支只对部分轮子增加旋转，不应作为标准中心旋转的 Nav2 工作模式。
- `uint8_t speed` / `int8_t motor_speed` 会发生整数截断，角度也为整数。
  ROS 物理速度解算应使用浮点数，最终按下位机明确的速度单位转换和限幅。
- 指令速度不能代替实际编码器反馈。轮式里程计应基于编码器累计计数或实测轮速和时间。

## 标准 X 型麦轮的参考公式

采用机器人 X 前、Y 左、Z 上，轮角速度正方向统一定义为使该轮向前滚动。
实际电机/编码器极性必须测量并归一化；不能仅从数组位置推断。
以下公式适用于标准 X 型滚轮布局，需与实际滚轮安装核对。

设轮半径 `r`，车体中心到前后轮轴的距离为 `a`，到左右轮中心线的距离为 `b`，
`L=a+b`。注意 a、b 是半轴距、半轮距，单位 m。

```text
w_FL = (vx - vy - L*wz) / r
w_FR = (vx + vy + L*wz) / r
w_RL = (vx + vy - L*wz) / r
w_RR = (vx - vy + L*wz) / r
```

轮速 `w_*` 单位 rad/s。反解实测底盘速度：

```text
vx = r/4     * ( w_FL + w_FR + w_RL + w_RR)
vy = r/4     * (-w_FL + w_FR + w_RL - w_RR)
wz = r/(4*L) * (-w_FL + w_FR - w_RL + w_RR)
```

将各轮目标速度转换到 M1/M2/M3/M4 的顺序和电机极性，再传入速度闭环。
具体转换需要轮半径、轴距/轮距、编码器每轮计数（含减速与倍频）、
电机模块定义及串口协议；这些数值不应使用猜测的默认值。

## ROS 数据检查

先抬起车轮低速确认轮序和方向，再在地面进行短距离验证：

| 运动 | 预期轮式里程计（base_link 坐标） |
| --- | --- |
| 向前 | `linear.x > 0`，`linear.y` 约 0 |
| 向左横移 | `linear.y > 0`，`linear.x` 约 0 |
| 逆时针原地旋转 | `angular.z > 0`，并与 IMU 转动符号一致 |

麦轮横移易受滚轮滑移影响，横向速度的协方差通常需要按实验估计。
当前 EKF 使用编码器 vx/vy/wz 与 IMU wz，保留三自由度平面运动能力；
它不会自动修正错误轮序、比例系数或将没有实测的横向速度补出来。
