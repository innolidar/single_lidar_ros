ROS1
##编译方法一  
#直接编译
```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN)
#=======================================
set(COMPILE_METHOD ORIGINAL)

cd innolight_lidar_sdk
mkdir build && cd build
cmake .. && make -j4
./innolidar_sdk_node


##编译方法二
#依赖于ROS-catkin编译
```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)

catkin_make
source devel/setup.bash
roslaunch innolidar_sdk start.launch

ROS2
#依赖于COLCON编译

