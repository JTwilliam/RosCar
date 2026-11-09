# learm_bridge

ROS 2 Humble package for the STM32F103RBT6 LeArm controller. It decodes the
USART1 COBS/CRC32C protocol, publishes mecanum wheel odometry, raw wheel counts
and controller diagnostics, and provides an optional terminal dashboard.

## ROS interfaces

- `/learm/wheel_encoders` (`std_msgs/msg/Int32MultiArray`): raw cumulative
  encoder counts in firmware order `[wheel_0, wheel_1, wheel_2, wheel_3]`.
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): firmware timestamp,
  sample sequence, measured rate, frame age, safety state, fault flags, CRC and
  framing counters.
- `/odom` (`nav_msgs/msg/Odometry`): planar pose and body velocity from the four
  wheel encoder increments. The node also publishes `odom -> base_link` TF by
  default.

The encoder topic intentionally remains raw counts. Odometry uses the confirmed
wheel order `[front_right, rear_right, front_left, rear_left]` and the mapping
from the existing Armrobot firmware. Current parameters are 48 mm wheel radius,
190 mm wheelbase, and 205 mm left/right wheel-center spacing. The 5740.253
counts/revolution value is inferred from the existing 0.05254 mm/tick calibration
and 96 mm wheel diameter; it is not an independently measured integer count.
Confirm scale by comparing `/odom` with a measured straight-line travel.

Velocities use STM32 sample timestamps for `dt`, so frames delivered together
over USB do not produce artificial velocity spikes. Message headers use the
host ROS clock. `twist_covariance_diagonal` supplies configurable positive
variances for `[vx,vy,vz,wx,wy,wz]`, initially
`[0.0025,0.01,1e6,1e6,1e6,0.04]`. These are starting estimates, not calibration
results. Integrated pose covariance is not modelled; the EKF uses twist only.

## Raspberry Pi prerequisites

Target host: Raspberry Pi 4B running Ubuntu 22.04 and ROS 2 Humble.

```bash
sudo apt update
sudo apt install python3-serial ros-humble-diagnostic-msgs ros-humble-nav-msgs ros-humble-tf2-ros
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing the `dialout` group. Confirm the stable
device name:

```bash
ls -l /dev/serial/by-id/
```

## Build

From the Raspberry Pi workspace root:

```bash
cd ~/RosCar/pi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select learm_bridge
source install/setup.bash
```

## Run odometry only

This starts the serial bridge, which publishes `/odom`, raw encoder counts and
diagnostics without a terminal monitor:

```bash
ros2 launch learm_bridge serial_bridge.launch.py \
  port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  baud:=115200
```

## Run the validation monitor

```bash
ros2 launch learm_bridge wheel_monitor.launch.py \
  port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  baud:=115200
```

The validation launch starts both the bridge and a terminal monitor. The
monitor displays encoder values plus `/odom` pose, displacement from the origin,
and accumulated translational path length. The bridge owns the serial port. Do
not run `serial_listener.py`, a serial terminal, YMODEM sender, or another
bridge process at the same time.

The monitor refreshes at 10 Hz and shows:

- four cumulative encoder counts and per-message deltas;
- measured chassis telemetry rate and last-frame age;
- STM32 sample timestamp and sample sequence;
- safety state and fault flags;
- valid, dropped, CRC-error and sequence-loss counters.

Stop both nodes with `Ctrl+C`.

## Fuse with ICM20948 and robot_localization

Build the updated `learm_bridge` and `icm20948_ros2` in the same workspace.
With the IMU driver and measured `base_link -> imu_link` mounting TF running:

```bash
ros2 launch icm20948_ros2 chassis_ekf.launch.py
```

This starts one serial bridge with `publish_tf=false` and an EKF subscribing
to `/odom` (wheel body-frame vx/vy/wz) and `/imu/data` (gyro Z). The EKF publishes
`/odometry/filtered` and is the sole publisher of `odom -> base_link` TF.
Do not run the standalone bridge or validation launch at the same time.
For separate launches, pass `publish_tf:=false` to `serial_bridge.launch.py`
and start `ros2 launch icm20948_ros2 ekf.launch.py` in another terminal.

## Offline tests

The protocol tests include a real captured STM32 frame and require no ROS
installation:

```bash
cd ~/RosCar/pi_ws/src/learm_bridge
python3 -m pytest test -q --ignore=test/test_ros_odometry.py
```

After sourcing ROS Humble, the full test suite also checks real ROS odometry
publication, firmware timing for batched serial samples, covariances and TF suppression.
