cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0048 NEW)
project(ros_lidar_sdk)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON) 
#=======================================
set(COMPILE_METHOD COLCON)

if(COMPILE_METHOD STREQUAL "COLCON")
option(ENABLE_ROS1 "Enable ROS1" OFF)
option(ENABLE_ROS2 "Enable ROS2" ON)
else()
option(ENABLE_ROS1 "Enable ROS1" ON)
option(ENABLE_ROS2 "Enable ROS2" OFF)
endif()

option(ENABLE_TRANSFORM "Enable transform functions" ON)
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")

  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

option(ENABLE_DOUBLE_RCVBUF "Enable double size of RCVBUF" OFF)
if(${ENABLE_DOUBLE_RCVBUF})
  add_definitions("-DENABLE_DOUBLE_RCVBUF")
endif(${ENABLE_DOUBLE_RCVBUF})

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})

#option(ENABLE_SOURCE_PACKET_LEGACY "Enable ROS Source of MSOP/DIFOP Packet v1.3.x" OFF)
#if(${ENABLE_SOURCE_PACKET_LEGACY})
#  add_definitions("-DENABLE_SOURCE_PACKET_LEGACY")
#endif(${ENABLE_SOURCE_PACKET_LEGACY})

#========================
# Project details / setup
#========================
set(PROJECT_NAME ros_lidar_sdk)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

if(${ENABLE_ROS2})
if($ENV{ROS_DISTRO} STREQUAL "humble")  # the ros2 humble requires c++17
  add_definitions(-std=c++17)
elseif($ENV{ROS_DISTRO} STREQUAL "jazzy")
  add_definitions(-std=c++17)
else()
  add_definitions(-std=c++14)
endif()
endif(${ENABLE_ROS2})


add_compile_options(-Wall)


#ROS#
if(${ENABLE_ROS1})
find_package(roscpp 1.12 QUIET)
if(roscpp_FOUND)

  message(=============================================================)
  message("-- ROS Found. ROS Support is turned On.")
  message(=============================================================)
  add_definitions(-DROS_FOUND=1)
  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

  #Catkin#
  if(${COMPILE_METHOD} STREQUAL "CATKIN")

    add_definitions(-DRUN_IN_ROS_WORKSPACE)

    find_package(catkin REQUIRED COMPONENTS
      roscpp
      sensor_msgs
      roslib
      std_msgs
      message_generation)

    #catkin_package(CATKIN_DEPENDS 
    #  sensor_msgs 
    #  roslib
    #  message_runtime)
    add_service_files(FILES 
      DeviceStatus.srv 
      GetDeviceTime.srv
      SetDeviceTime.srv)
    generate_messages(
      DEPENDENCIES 
      std_msgs
      )
    catkin_package(CATKIN_DEPENDS 
    sensor_msgs 
    message_runtime 
    roscpp 
    roslib
    std_msgs
    )
  endif(${COMPILE_METHOD} STREQUAL "CATKIN")

else(roscpp_FOUND)

  message(=============================================================)
  message("-- ROS Not Found. ROS Support is turned Off.")
  message(=============================================================)

endif(roscpp_FOUND)
endif(${ENABLE_ROS1})


if(${ENABLE_ROS2})
#ROS2#
find_package(rclcpp QUIET)
message("ROS2------------------")
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=========VERSION====================================================)
  message("-- ROS2 Found. ROS2 Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS_FOUND=2)
  include_directories(${rclcpp_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD 14)

  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(lidar_sdk_msg REQUIRED)
  find_package(std_msgs REQUIRED)  
  find_package(rosidl_default_generators REQUIRED)  
  find_package(rosidl_typesupport_cpp REQUIRED)
  #rosidl_generate_interfaces(${PROJECT_NAME}
  #"srv/DeviceStatus.srv"
  #DEPENDENCIES stdVERSION_msgs
  #)
 
  include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )

  #ament_export_include_directories(include)
else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=============================================================)
  message("-- ROS2 Not Found. ROS2 Support is turned Off.")
  message(=============================================================)

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
endif(${ENABLE_ROS2})
#Others#
find_package(yaml-cpp REQUIRED)


#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(src/lidar_driver)
find_package(lidar_driver REQUIRED)
include_directories(${lidar_driver_INCLUDE_DIRS})
  message(********************************************************)
  message("-- ${lidar_driver_INCLUDE_DIRS}")
  message(********************************************************)

#========================
# Build Setup
#========================

add_executable(lidar_sdk_node
               node/lidar_sdk_node.cpp
               src/manager/node_manager.cpp)
target_compile_options(lidar_sdk_node PRIVATE
  -O3
#  -march=native    # 生成当前CPU专属指令集
  -flto            # 链接时优化
)
set_target_properties(lidar_sdk_node PROPERTIES
  INTERPROCEDURAL_OPTIMIZATION TRUE  # 启用LTO
)
target_link_libraries(lidar_sdk_node
                      ${YAML_CPP_LIBRARIES}
                      ${lidar_driver_LIBRARIES})


if(${ENABLE_ROS1})
#Ros#
if(roscpp_FOUND)

  target_link_libraries(lidar_sdk_node 
    ${ROS_LIBS})

  if(${COMPILE_METHOD} STREQUAL "CATKIN")

    install(TARGETS lidar_sdk_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  endif()

endif(roscpp_FOUND)
endif(${ENABLE_ROS1})

if(${ENABLE_ROS2})
#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  ament_target_dependencies(lidar_sdk_node 
    rclcpp 
    std_msgs 
    sensor_msgs
    rosidl_typesupport_cpp
    lidar_sdk_msg
    )

  install(TARGETS
    lidar_sdk_node
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME})
  #ament_export_include_directories(include)
  ament_package()
  
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
endif(${ENABLE_ROS2})

