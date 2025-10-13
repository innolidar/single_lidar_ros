/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include "manager/node_manager.hpp"
#include <lidar_driver/macro/version.hpp>
#include <signal.h>

#if ROS_FOUND==1
#include <ros/ros.h>
#include <ros/package.h>
#else
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#endif

using namespace lidar;
#if ROS_FOUND==2
std::mutex g_mtx;
std::condition_variable g_cv;
#endif

static void sigHandler(int sig)
{
  TOOL_MSG << "LiDAR-Driver is stopping....." << TOOL_REND;

#if ROS_FOUND==1
  ros::shutdown();
#else
  g_cv.notify_all();
#endif
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function

  TOOL_TITLE << "********************************************************" << TOOL_REND;
  TOOL_TITLE << "**********                                    **********" << TOOL_REND;
  TOOL_TITLE << "**********    Lidar_SDK Version: v" << LIDARDRIVER_VERSION_MAJOR 
    << "." << LIDARDRIVER_VERSION_MINOR 
    << "." << LIDARDRIVER_VERSION_PATCH << "     **********" << TOOL_REND;
  TOOL_TITLE << "**********                                    **********" << TOOL_REND;
  TOOL_TITLE << "********************************************************" << TOOL_REND;

#if ROS_FOUND==1
  ros::init(argc, argv, "lidar_sdk", ros::init_options::NoSigintHandler);
#else
  rclcpp::init(argc, argv);
#endif

  std::string config_path;

#ifdef RUN_IN_ROS_WORKSPACE
   config_path = ros::package::getPath("ros_lidar_sdk");
#else
   config_path = (std::string)PROJECT_PATH;

#endif
  std::cout<<"config_path:"<<config_path<<std::endl;
   config_path += "/config/config.yaml";
#if ROS_FOUND==1
  ros::NodeHandle priv_hh("~");
  std::string path;
  priv_hh.param("config_path", path, std::string(""));
  if (!path.empty())
  {
    config_path = path;
  }
#endif

  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
  }
  catch (...)
  {
    TOOL_ERROR << "The format of config file " << config_path 
      << " is wrong. Please check (e.g. indentation)." << TOOL_REND;
    return -1;
  }
  std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
  demo_ptr->Init(config);
  demo_ptr->Start();
  TOOL_MSG << "LiDAR-Driver is running....." << TOOL_REND;
#if ROS_FOUND==1
  ros::spin();
#else
  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);
#endif

  return 0;
}
