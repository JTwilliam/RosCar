①编译方法：使用catkin编译，执行如下操作
将功能包解压后复制到工作空间的路径下，然后执行指令“catkin_make”进行编译。
单独编译功能包：
colcon build --packages-select ldlidar

②设备别名：端口设备重命名
launch中启动的stl06n雷达默认设备名为：/dev/wheeltec_lidar，别名文件是“ldlidar_udev.sh”，
如果您使用的ttl电平转换芯片为CP2102，设备号需要改为0001；

具体修改方法请分别查看对应的驱动资料。

③运行方法
启动：stl06n串口雷达
source install/setup.bash
ros2 launch ldlidar stl06n.launch.py
启动：stl06n网口雷达
source install/setup.bash
ros2 launch ldlidar stl06nnet.launch.py

启动：stl19P串口雷达
source install/setup.bash
ros2 launch ldlidar ld06.launch.py
启动：stl19P网口雷达
source install/setup.bash
ros2 launch ldlidar ld06net.launch.py

④rviz可视化查看点云：
开启rviz2，rviz的基坐标选择laser，选择“laserscan”的可视化类型，话题选择“scan”。

⑤雷达角度分割
启用雷达角度分割：{'enable_angle_crop_func': True}
禁用雷达角度分割：{'enable_angle_crop_func': False}

## 测试：代码在ubuntun20.04 humble版本下测试，使用rviz2可视化。
