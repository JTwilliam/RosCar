# RosCar 项目维护约定

- 本仓库是该机器人项目的源码主仓库，远端为 https://github.com/JTwilliam/RosCar.git。
- 维护主机是 Ubuntu。用户已要求后续本项目持续维护此仓库；相关已完成改动应在验证后提交并正常推送，提交主题以 [Ubuntu主机上传] 开头。
- 编辑本仓库中的源码，不要回到上级目录的历史散落副本继续维护。不要在 /mnt/data 的父级 Git 仓库执行项目提交。
- 改动前检查 Git 状态及远端更新，保留用户未提交的工作。不要强制推送、覆盖远端历史或提交无关文件。
- pi_ws/src/learm_bridge 是 STM32 串口解码和麦轮里程计；pi_ws/src/icm20948_ros2 是 IMU、EKF 配置及可视化。
- lidar_ws/src/ldlidar 与 ldlidar_driver 必须保持相邻；ldlidar_driver 是源文件目录，不是独立 ROS 包。
- 所有传感器和融合在树莓派运行，笔记本仅显示。不得把本机模拟测试描述为树莓派实车验证。
- 轮子原始编码器顺序为 FR、RR、FL、RL，保留已提供的有符号映射。EKF 使用 vx、vy、wz 与 IMU wz，不能增加差速底盘 vy=0 约束。
- 桥接输入 /odom，融合输出 /odometry/filtered；只由 EKF 发布 odom -> base_link。IMU 和雷达安装 TF 必须依据实际测量。
- IMU 接物理 3/5，即 I2C1；雷达使用 /dev/serial0、230400。保留硬件参数和标定值的来源说明。
- 使用 /opt/ros/humble 和 /usr/bin/python3 做适用的软件验证，构建产物与日志不得提交。
- 更新启动流程、参数或接口时，同步更新 README.md、docs/运行手册.md 和 docs/进展与验证.md。
- 保留第三方源码版权与许可证。references 中的代码用于溯源，不把它当作完整可烧录的 STM32 固件。
- GitHub 凭据只由本机 Git/GitHub CLI 管理；认证缺失时保留已完成的本地提交，说明实际推送状态，不索要聊天中的密码或令牌。
