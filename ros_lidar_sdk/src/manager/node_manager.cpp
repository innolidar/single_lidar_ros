/*********************************************************************************************************************
Copyright (c) 2023 innolight
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For innolight LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the innolight, nor Suteng Innovation Technology, nor the names of other contributors may be used
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
#include "source/source_driver.hpp"
#include "source/source_laserscan_ros.hpp"

namespace lidar
{

void NodeManager::Init(const YAML::Node& config)
{
  YAML::Node common_config = yamlSubNodeAbort(config, "common");

  int msg_source = 0;
  yamlRead<int>(common_config, "msg_source", msg_source, 0);

  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
  for (uint8_t i = 0; i < lidar_config.size(); ++i)
  {
    std::shared_ptr<Source> source;
    bool is_local{false};
    switch (msg_source)
    {
      case SourceType::MSG_FROM_LIDAR:  // online lidar

        TOOL_INFO << "------------------------------------------------------" << TOOL_REND;
        TOOL_INFO << "Receive Packets From : Online LiDAR" << TOOL_REND;
        TOOL_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << TOOL_REND;
        TOOL_INFO << "------------------------------------------------------" << TOOL_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_LIDAR);
        source->Init(lidar_config[i]);
        break;
      case SourceType::MSG_FROM_PCAP:  // pcap

        TOOL_INFO << "------------------------------------------------------" << TOOL_REND;
        TOOL_INFO << "Receive Packets From : Pcap" << TOOL_REND;
        TOOL_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << TOOL_REND;
        TOOL_INFO << "------------------------------------------------------" << TOOL_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_PCAP);
        source->Init(lidar_config[i]);
        is_local=true;
        break;
      default:
        TOOL_ERROR << "Unsupported LiDAR message source:" << msg_source << "." << TOOL_REND;
        exit(-1);
    }

    TOOL_DEBUG << "------------------------------------------------------" << TOOL_REND;
    TOOL_DEBUG << "Send LaserScan To : ROS" << TOOL_REND;
    TOOL_DEBUG << "LaserScan Topic: " << lidar_config[i]["ros"]["ros_send_laserscan_topic"].as<std::string>()
               << TOOL_REND;
    TOOL_DEBUG << "------------------------------------------------------" << TOOL_REND;

    std::shared_ptr<DestinationLaserScan> dst = std::make_shared<DestinationLaserScanRos>();
    dst->Init(lidar_config[i]);
    if(!is_local)
    {
        dst->RegisterServiceCallBack([this,source](const uint32_t &command_type,std::string &text)
        {
          return source->GetDeviceStatus(command_type,text);
        });

        dst->RegisterGetTimeServiceCallBack([this,source](uint64_t &ms)
        {
          return source->GetDeviceTimeStatus(ms);
        });

        dst->RegisterSetTimeServiceCallBack([this,source](const uint64_t &ms)
        {
          return source->SetDeviceTimeStatus(ms);
        });
    }
    
    source->RegisterLaserScanCallback(dst);
    m_sources.emplace_back(source);
  }
}

void NodeManager::Start()
{
  for (auto& iter : m_sources)
  {
    if (iter != nullptr)
    {
      iter->Start();
    }
  }
}

void NodeManager::Stop()
{
  for (auto& iter : m_sources)
  {
    if (iter != nullptr)
    {
      iter->Stop();
      iter.reset();
      iter=nullptr;
    }
  }
}

void Run()
{
  
}

NodeManager::~NodeManager()
{
  Stop();
}


}  // namespace lidar
