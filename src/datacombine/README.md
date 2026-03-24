# 基于ELF2开发板的Livox双雷达分体式扫描系统
# 目录介绍
NormalSet：livox ros driver config文件以及fast lio config 文件  
shell ： 自启动脚本  
config： 代码外参  
launch： 启动脚本  

# 双Mid360激光雷达数据融合

本仓库主要存储双激光雷达节点融合功能包

## 概述

该功能包提供了一个ROS 2节点，具有以下功能：
- 订阅两个Livox Mid-360激光雷达传感器的数据
- 基于时间戳同步两个传感器的点云数据
- 应用坐标变换来融合点云数据
- 发布融合后的点云和同步的IMU数据

## 系统要求

- ROS 2 Humble
- Livox ROS Driver 2
- Livox SDK2  
- Fast Livo  
- PCL (点云库)
- Eigen3

## 编译

```bash
cd ~/ros2_ws  # 或你的工作空间根目录
colcon build --packages-select merge_cloud
source install/setup.bash
```

## 使用方法

1. 在 `config/merge_config.yaml` 中配置变换对应的外参参数，外参标定可以使用 Livox Viewer  
2. 将合并节点（main.cpp 对应程序）中的 IP 地址更换为对应雷达的 IP  
3. 将 NormalSet 里的 config 中涉及 IP 地址的项更换为自己设备的 IP  
4. 启动节点：

```bash
ros2 run merge_cloud merge_cloud_node
```

## 话题

### 订阅的话题
- `/livox/lidar_192_168_1_151` (livox_ros_driver2/msg/CustomMsg) - 第一个激光雷达传感器
- `/livox/lidar_192_168_1_3` (livox_ros_driver2/msg/CustomMsg) - 第二个激光雷达传感器
- `/livox/imu_192_168_1_151` (sensor_msgs/msg/Imu) - IMU数据

### 发布的话题
- `/merged_cloud` (livox_ros_driver2/msg/CustomMsg) - 融合后的点云
- `/cloud_registered_body/imu` (sensor_msgs/msg/Imu) - 变换后的IMU数据

