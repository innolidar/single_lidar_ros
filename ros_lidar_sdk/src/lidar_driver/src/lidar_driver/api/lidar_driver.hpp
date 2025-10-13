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

#include <lidar_driver/driver/lidar_driver_impl.hpp>
#include <lidar_driver/msg/data_types.hpp>
#include <lidar_driver/msg/packet.hpp>

namespace lidar
{

inline std::string GetDriverVersion();
/**
 * @brief This is the InnoLight LiDAR driver interface class
 */
class LidarDriver
{
public:
  /**
   * @brief Constructor, instanciate the driver pointer
   */
  LidarDriver() 
    : m_pDriver(std::make_shared<LidarDriverImpl>())
  {

  }
  ~LidarDriver()
  {
     m_pDriver.reset();
     m_pDriver=NULL;
  }

  /**
   * @brief Register the lidar point cloud callback function to driver. When point cloud is ready, this function will be
   * called
   * @param callback The callback function
   */

  inline void RegisterLaserScanCallback(const std::function<std::shared_ptr<Laser_msg>()> &get_laserscan_callback,
                                         const std::function<void(std::shared_ptr<Laser_msg>)> &put_laserscan_callback)
  {
    m_pDriver->RegisterLaserScanCallback(get_laserscan_callback,put_laserscan_callback);
  }

  /**
   * @brief Register the exception message callback function to driver. When error occurs, this function will be called
   * @param callback The callback function
   */
  inline void RegisterExceptionCallback(const std::function<void(const Error &)> &exception_callback)
  {
    m_pDriver->RegisterExceptionCallback(exception_callback);
  }

  /**
   * @brief The initialization function, used to set up parameters and instance objects,
   *        used when get packets from online lidar or pcap
   * @param param The custom struct DriverParam
   * @return If successful, return true; else return false
   */
  inline bool Init(const DriverParam& param)
  {
    return m_pDriver->Init(param);
  }

  /**
   * @brief Start the thread to receive and decode packets
   * @return If successful, return true; else return false
   */
  inline bool Start()
  {
    return m_pDriver->Start();
  }
  /**
   * @brief Stop all threads
   */
  inline void Stop()
  {
    m_pDriver->Stop();
  }

  bool GetDeviceStatus(const int32_t &command_type,std::string &text)
  {
    return m_pDriver->GetDeviceStatus(command_type,text);
  }

  bool GetDeviceTimeStatus(uint64_t &ms)
  {
    return m_pDriver->GetDeviceTimeStatus(ms);
  }
  bool SetDeviceTimeStatus(const uint64_t &ms)
  {
    return m_pDriver->SetDeviceTimeStatus(ms);
  }

private:
  std::shared_ptr<LidarDriverImpl> m_pDriver;  ///< The driver pointer
};

}  // namespace lidar
