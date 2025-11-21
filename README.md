# single_lidar_ros
## 概述

`single_lidar_ros` 是由苏州智驰领驭开发的 ROS 驱动软件包，专门用于支持 Innolidar 激光雷达。该软件包支持 ROS1 和 ROS2，提供点云数据处理，并包含雷达驱动内核及 ROS1/ROS2 扩展功能。

## 特性

- 目前支持激光雷达型号： HA25 / HA50 
- 兼容 ROS1 (Noetic, Melodic, Kinetic) 和 ROS2 (Foxy, Humble)
- 支持实时雷达数据和 pcap 文件回放
- 提供点云数据发布功能
- 可配置点云角度范围、距离过滤等参数
- 支持外参设置（平移和旋转）

## 系统要求

- Ubuntu 16.04/18.04/20.04 (ROS1)
- Ubuntu 20.04/22.04 (ROS2)
- ROS1: kinetic, melodic 或 noetic
- ROS2: foxy 或 humble
- C++14 (ROS1) / C++17 (ROS2 Humble)
- libpcap-dev (用于 pcap 文件回放)
- yaml-cpp

## 依赖库安装

### ROS 环境
根据您的系统选择对应的 ROS 版本：

**ROS1:**
- Ubuntu 16.04 - ROS kinetic desktop-full
- Ubuntu 18.04 - ROS melodic desktop-full
- Ubuntu 20.04 - ROS noetic desktop-full

**ROS2:**
- Ubuntu 20.04 - ROS2 foxy desktop
- Ubuntu 22.04 - ROS2 humble desktop

安装方式：参考 http://wiki.ros.org

**其他依赖库:**
```bash
sudo apt install libpcap-dev libyaml-cpp-dev
```

## 编译与安装

single_lidar_ros 支持多种编译方式，包括原始 CMake 编译、ROS1 catkin 编译和 ROS2 colcon 编译。

### 方法一：使用脚本自动编译（推荐）

```bash
# 编译 ROS2 版本
./build_ros2.sh

# 编译 ROS1 版本
./build_ros1.sh
```

### 方法二：手动编译（详细步骤）

#### ROS1 编译方式

1. 设置编译方法为 CATKIN
   ```bash
   cd inno_lidar_ros
   cp src/inno_lidar_ros/package_ros1.xml src/inno_lidar_ros/package.xml
   cp src/inno_lidar_msg/package_ros1.xml src/inno_lidar_msg/package.xml
   sed -i 's/set(COMPILE_METHOD .*)/set(COMPILE_METHOD CATKIN)/' src/inno_lidar_ros/CMakeLists.txt
   sed -i 's/set(COMPILE_METHOD .*)/set(COMPILE_METHOD CATKIN)/' src/inno_lidar_msg/CMakeLists.txt
   ```

2. 在工作空间目录下编译
   ```bash
   catkin_make
   source devel/setup.bash
   roslaunch inno_lidar_ros ros1_start.launch
   ```

#### ROS2 编译方式

1. 设置编译方法为 COLCON
   ```bash
   cd inno_lidar_ros
   cp src/inno_lidar_ros/package_ros2.xml src/inno_lidar_ros/package.xml
   cp src/inno_lidar_msg/package_ros2.xml src/inno_lidar_msg/package.xml
   sed -i 's/set(COMPILE_METHOD .*)/set(COMPILE_METHOD COLCON)/' src/inno_lidar_ros/CMakeLists.txt
   sed -i 's/set(COMPILE_METHOD .*)/set(COMPILE_METHOD COLCON)/' src/inno_lidar_msg/CMakeLists.txt
   ```

2. 在工作空间目录下编译
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch inno_lidar_ros ros2_start.launch.py
   ```

## 配置说明

在编译前需要根据需求配置 `config/config.yaml` 文件中的参数，具体说明如下：

```yaml
common:
  msg_source: 2                                 # 数据来源: 0无数据, 1雷达数据, 2pcap文件

lidar:
  - driver:
      lidar_type: LIDAR_HA25                    # 雷达类型 (目前支持LIDAR_HA25 LIDAR_HA50)
      msop_port: 8080                           # 雷达点云数据端口
      h_start_angle: -360                       # 水平起始角度
      h_end_angle: 360                          # 水平结束角度
      min_distance: 0.02                        # 最小距离过滤
      max_distance: 200                         # 最大距离过滤
      pcap_file: /path/to/your/pcap/file.pcap   # pcap文件路径
      pcap_repeat: true                         # pcap文件是否循环播放
      pcap_rate: 20.0                           # pcap文件循环播放速度

    ros:
      ros_frame_id: base_laser                            # ROS坐标系ID
      send_laserscan_ros: true       			    # 是否laserscan发布
      use_lidar_clock: true				    # 是否使用lidar时钟
      ros_send_laserscan_topic: /inno_imu                 # laserscan发布主题
      enable_service_ros: true			    # 是否开启服务
      ros_service_lidar_type: /ha25_device_status           # 设备状态服务
      ros_service_get_time_type: /ha25_get_device_time_status  #获取设备时间戳服务
      ros_service_set_time_type: /ha25_set_device_time_status  #设置设备时间戳服务
```
## 使用场景

1. **实时数据采集**: 将 msg_source 设置为 1，连接实际的雷达,实时数据
2. **离线数据回放**: 将 msg_source 设置为 2，指定 pcap_file 路径进行数据回放
3. **点云可视化**: 启动节点后，可通过 RViz 查看点云数据

## 注意事项

1. 确保雷达或 pcap 文件路径配置正确
2. 使用离线 pcap 文件时，确保文件路径存在且可访问
3. 检查网络配置，确保能正确连接到雷达设备

## 故障排除

1. 如果无法接收点云数据，请检查 msg_source 配置
2. 确保雷达 IP 地址和端口配置正确
3. 检查防火墙设置，确保 UDP 端口未被阻止
4. 查看终端输出日志以获取更多调试信息

## 技术支持

下载路径：https://github.com/innolidar/single_lidar_ros.git
