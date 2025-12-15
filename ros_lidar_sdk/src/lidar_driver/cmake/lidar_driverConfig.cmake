# - Config file for the  package
# It defines the following variables
#  lidar_driver_INCLUDE_DIRS - include directories for 
#  lidar_driver_LIBRARIES    - libraries to link against
#  lidar_driver_FOUND        - found flag

if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(Boost_ARCHITECTURE "-x64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4) # 32-bit
    set(Boost_ARCHITECTURE "-x32")
  endif()
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
endif(${ENABLE_TRANSFORM})

set(lidar_driver_INCLUDE_DIRS "/home/liang/work/00_github/00_singleLidar/single_lidar_ros/ros_lidar_sdk/src/lidar_driver/src;/usr/local/ros_lidar_sdk/include")
set(lidar_driver_INCLUDE_DIRS "/home/liang/work/00_github/00_singleLidar/single_lidar_ros/ros_lidar_sdk/src/lidar_driver/src;/usr/local/ros_lidar_sdk/include")

set(lidar_driver_LIBRARIES "pthread;pcap")
set(lidar_driver_LIBRARIES "pthread;pcap")

set(lidar_driver_FOUND true)
set(lidar_driver_FOUND true)
