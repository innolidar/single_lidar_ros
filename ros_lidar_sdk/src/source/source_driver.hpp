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

#pragma once

#include "source/source.hpp"
#include <lidar_driver/api/lidar_driver.hpp>
#include <lidar_driver/utility/sync_queue.hpp>

namespace lidar
{

class SourceDriver : public Source
{
public:

  virtual void Init(const YAML::Node& config);
  virtual void Start();
  virtual void Stop();
  virtual ~SourceDriver();
  virtual bool GetDeviceStatus(const int32_t &command_type,std::string &text);
  virtual bool GetDeviceTimeStatus(uint64_t &ms);
  virtual bool SetDeviceTimeStatus(const uint64_t &ms);
  SourceDriver(SourceType src_type);

protected:

  std::shared_ptr<Laser_msg> GetLaserScan(void);
  void PutLaserScan(std::shared_ptr<Laser_msg>  msg);
  void PutException(const lidar::Error& msg);
  void ProcessLaserScan();

  bool                                              m_to_exit_process;
  std::thread                                       m_laserscan_process_thread;
  std::shared_ptr<LidarDriver>                      m_pShareDriver;
  SyncQueue<std::shared_ptr<Laser_msg>>             m_free_laserscan_queue;
  SyncQueue<std::shared_ptr<Laser_msg>>             m_laserscan_queue;
};

SourceDriver::SourceDriver(SourceType src_type)
  : Source(src_type), m_to_exit_process(false)
{

}

inline void SourceDriver::Init(const YAML::Node& config)
{
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  DriverParam driver_param;

  // input related
  yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port, 9000);
  yamlRead<std::string>(driver_config, "host_address", driver_param.input_param.host_address, "0.0.0.0");
  yamlRead<std::string>(driver_config, "group_address", driver_param.input_param.group_address, "0.0.0.0");
  yamlRead<float>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 10);
  yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, false);
  yamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
  
  // decoder related
  std::string lidar_type;
  yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
  driver_param.lidar_type = strToLidarType(lidar_type);

  // decoder
  yamlRead<bool>(driver_config,  "use_lidar_clock", driver_param.decoder_param.use_lidar_clock, false);
  yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
  yamlRead<float>(driver_config, "h_start_angle", driver_param.decoder_param.h_start_angle, 0);
  yamlRead<float>(driver_config, "h_end_angle", driver_param.decoder_param.h_end_angle, 360);
  switch (m_src_type)
  {
    case SourceType::MSG_FROM_LIDAR:
      driver_param.input_type = InputType::ONLINE_LIDAR;
      break;
    case SourceType::MSG_FROM_PCAP:
      driver_param.input_type = InputType::PCAP_FILE;
      break;
  }

  driver_param.print();
  m_pShareDriver=std::make_shared<LidarDriver>();
  m_pShareDriver->RegisterLaserScanCallback(std::bind(&SourceDriver::GetLaserScan,this),std::bind(&SourceDriver::PutLaserScan,this,std::placeholders::_1));
  m_pShareDriver->RegisterExceptionCallback(std::bind(&SourceDriver::PutException,this,std::placeholders::_1));
  m_laserscan_process_thread = std::thread(std::bind(&SourceDriver::ProcessLaserScan, this));
  //TOOL_ERROR << "Driver Initialize start...." << TOOL_REND;
  if (!m_pShareDriver->Init(driver_param))
  {
    TOOL_ERROR << "Driver Initialize Error...." << TOOL_REND;
    exit(-1);
  }
}

inline void SourceDriver::Start()
{
  m_pShareDriver->Start();
}

inline SourceDriver::~SourceDriver()
{
  Stop();
}

inline void SourceDriver::Stop()
{
  m_pShareDriver->Stop();

  m_to_exit_process = true;
  m_laserscan_process_thread.join();
}

inline std::shared_ptr<Laser_msg>  SourceDriver::GetLaserScan(void)
{
  if(m_free_laserscan_queue.GetSize()>=2)
  {
    std::shared_ptr<Laser_msg>  laser = m_free_laserscan_queue.Pop();
    if (laser!= NULL)
    {
      return laser;
    }
  }
  return std::make_shared<Laser_msg>();
}

void SourceDriver::PutLaserScan(std::shared_ptr<Laser_msg> msg)
{
  m_laserscan_queue.Push(msg);
}

bool SourceDriver::GetDeviceStatus(const int32_t &command_type,std::string &text)
{
    return m_pShareDriver->GetDeviceStatus(command_type,text);
}

bool SourceDriver::GetDeviceTimeStatus(uint64_t &ms)
{
  return m_pShareDriver->GetDeviceTimeStatus(ms);
}

bool SourceDriver::SetDeviceTimeStatus(const uint64_t &ms)
{
  return m_pShareDriver->SetDeviceTimeStatus(ms);
}

void SourceDriver::ProcessLaserScan()
{
  while (!m_to_exit_process)
  {
    std::shared_ptr<Laser_msg> msg = m_laserscan_queue.PopWait(1000);
    if (msg == NULL)
    {
      continue;
    }
    SendLaserScan(msg);
    m_free_laserscan_queue.Push(msg);
  }
}

inline void SourceDriver::PutException(const lidar::Error& msg)
{
  switch (msg.error_code_type)
  {
    case lidar::ErrCodeType::INFO_CODE:
      TOOL_INFO << msg.toString() << TOOL_REND;
      break;
    case lidar::ErrCodeType::WARNING_CODE:
      TOOL_WARNING << msg.toString() << TOOL_REND;
      break;
    case lidar::ErrCodeType::ERROR_CODE:
      TOOL_ERROR << msg.toString() << TOOL_REND;
      break;
  }
}

}  // namespace lidar
