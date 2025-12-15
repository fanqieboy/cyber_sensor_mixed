# Cyber Sensor ROS2 Driver

Cyber Sensor Fixed Driver is the driver package used to connect multi-functional LiDAR products produced by Cyber Sensor.

## 1. OS requirements
  * Ubuntu 20.04 for ROS2 Foxy;
  * Ubuntu 22.04 for ROS2 Humble;

## 2. Clone Cyber Sensor ROS2 driver source code

```shell
git clone https://github.com/fanqieboy/cyber_sensor_fixed.git cybersensor_ws/src/cyber_sensor_fixed
```

## 3. Build ROS2 driver

### For ROS2 Foxy
```shell
source /opt/ros/foxy/setup.bash
cd /path/to/your/cybersensor_ws
colcon build
```

### For ROS2 Humble
```shell
source /opt/ros/humble/setup.bash
cd /path/to/your/cybersensor_ws
colcon build
```

## 4. Run ROS2 driver
```shell
cd /path/to/your/cybersensor_ws
source install/setup.sh
ros2 launch cyber_sensor_fixed rviz_fixed_launch.py
```