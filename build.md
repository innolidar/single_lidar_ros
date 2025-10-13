ROS1
#准备工作
package_ros1.xml 更名为package.xml

#依赖于ROS-catkin编译
```cmake
#=======================================
# Compile setup (CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)

catkin_make
source devel/setup.bash
roslaunch ros_lidar_sdk ros1_start.launch


ROS2
#准备工作
1 创建ros2 工作空间（ros2_workspace),在工作空间中的src文件夹下放入lidar_sdk_msg与ros_lidar_sdk 
2 将ros_lidar_sdk文件夹下的package_ros2.xml 更名为package.xml,修改CMakeLists.txt中的编译类型set(COMPILE_METHOD COLCON)
3 在ros2_workspace文件夹下，打开终端编译
#依赖于COLCON编译
#=======================================
# Compile setup (CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)

colcon build
source ./install/setup.bash
ros2 run ros_lidar_sdk lidar__sdk_node
或者
ros2 launch ros_lidar_sdk ros2_start.launch


