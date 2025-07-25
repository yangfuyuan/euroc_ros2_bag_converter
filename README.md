# euroc_ros2_bag_converter
A tool to convert the EuRoC MAV Dataset into ROS 2 bag format

## Overview

`euroc_ros2_bag_converter` is a lightweight tool to convert the [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) into [ROS 2](https://www.ros.org/) bag format (`.db3`). It allows users to replay EuRoC data using ROS 2 tools for testing and benchmarking SLAM, VIO, and sensor fusion algorithms.

The tool supports converting:
- Monocular or stereo image data into `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo`
- IMU measurements into `sensor_msgs/msg/Imu`
- Ground truth poses (if available) into `nav_msgs/msg/Odometry` and `nav_msgs/msg/Path`

---

## Features

- Converts EuRoC dataset to ROS 2 `.db3` bag
- Compatible with SLAM and VIO pipelines in ROS 2
- Easy to extend for other sensors (e.g. depth, camera info, ground truth path)

## Usage

### 1. Clone the Repository

```bash
cd $(PATH_TO_YOUR_ROS2_WS)/src
git clone https://github.com/yangfuyuan/euroc_ros2_bag_converter.git
cd ..
colcon build --symlink-install && source ./install/setup.bash && source ./install/local_setup.bash
```bash

## run
```bash
# convert
ros2 launch  euroc_ros2_bag_converter euroc_ros2_bag_converter.launch.py
# custom dataset
ros2 launch euroc_ros2_bag_converter euroc_ros2_bag_converter.launch.py euroc_root:=~/dataset/euroc/MH_05_difficult bag_path:=~/dataset/euroc/MH_05_difficult/euroc.db3
```