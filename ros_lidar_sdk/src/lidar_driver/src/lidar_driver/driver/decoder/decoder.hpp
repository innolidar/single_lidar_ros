/*********************************************************************************************************************
Copyright (c) 2023 InnoLight
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For InnoLight LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the InnoLight, nor Suteng Innovation Technology, nor the names of other contributors may be used
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
#include <lidar_driver/common/error_code.hpp>
#include <lidar_driver/common/lidar_common.hpp>
#include <lidar_driver/driver/driver_param.hpp>
#include <lidar_driver/driver/decoder/section.hpp>
#include <lidar_driver/driver/decoder/basic_attr.hpp>
#include <lidar_driver/driver/decoder/member_checker.hpp>
#include <lidar_driver/utility/tool.hpp>
#include <lidar_driver/common/lidar_tool.hpp>
//#include <lidar_driver/driver/decoder/split_strategy.hpp>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES // for VC++, required to use const M_IP in <math.h>
#endif

#define ENABLE_TRANSFORM 1

#ifdef ENABLE_TRANSFORM
// Eigen lib
#include <Eigen/Dense>
#endif

#include <cmath>
#include <functional>
#include <memory>
#include <iomanip>

namespace lidar
{

#pragma pack(push, 1)

#pragma pack(pop)

// decoder const param
struct DecoderConstParam
{
  // packet len
  uint16_t MSOP_LEN;
  // packet identity
  uint8_t MSOP_ID_LEN;
  uint8_t MSOP_ID[8];
  // packet structure
  uint16_t BLOCKS_PER_PKT;
  // distance & temperature
  float DISTANCE_RES;

  float LIDAR_START_ANGLE;         ///<: 雷达起始角度
  float LIDAR_END_ANGLE;           ///<: 雷达终止角度
  float LIDAR_ANGLE_INCREMENT;       ///<: 雷达角度分辨率
  float LIDAR_RANGE_MIN;                            ///<: 雷达最小测距范围
  float LIDAR_RANGE_MAX;                            ///<: 雷达最大测距范围
};

#define INIT_ONLY_ONCE() \
  static bool init_flag = false; \
  if (init_flag) return param; \
  init_flag = true;

class Decoder
{
public:
  explicit Decoder(const DecoderConstParam& const_param, const DecoderParam& param);
  virtual ~Decoder(){};

  virtual bool DecodeMsopPacket(const uint8_t* pkt, size_t size, uint16_t start) = 0;
  bool ProcessMsopPacket(const uint8_t* pkt, size_t size, uint16_t start_blk = 0);

  virtual bool toSplitFrame(const uint8_t* pkt, size_t size, uint16_t& start_blk) = 0;
  bool toSplite(const uint8_t* pkt, size_t size, uint16_t& start_blk);

  void EnableWritePacketTimes(bool value);

  void RegisterCallback(const std::function<void(const Error &)> &exception_callback,const std::function<void()> &splite_frame_callback);
  double GetPacketDuration();
  void SetLaserScan(std::shared_ptr<Laser_msg> msg)
  {
    msg->Clear();
    m_laser_msg = msg;
  }
  std::shared_ptr<Laser_msg> GetLaserScan()
  {
    return m_laser_msg;
  }
protected:
  double              m_sys_ts;
  bool                m_write_pkt_ts;
  DecoderParam        m_param;                    // user param
  DecoderConstParam   m_const_param;              // const param
  std::function<void()>  m_splite_frame_callback;
  std::function<void(const Error &)>    m_exception_callback;

  double          m_packet_duration;
  std::shared_ptr<Laser_msg>     m_laser_msg;          // accumulated point cloud currently
};
inline void Decoder::RegisterCallback(const std::function<void(const Error &)> &exception_callback,const std::function<void()> &splite_frame_callback)
{
  m_exception_callback    = exception_callback;
  m_splite_frame_callback = splite_frame_callback;
  m_sys_ts=tool::GetSysTime();
}

inline Decoder::Decoder(const DecoderConstParam& const_param, const DecoderParam& param)
  : m_const_param(const_param)
  , m_param(param)
  , m_write_pkt_ts(false)
  , m_laser_msg(nullptr)
  , m_packet_duration(0)
{

  
}

inline void Decoder::EnableWritePacketTimes(bool value)
{
  m_write_pkt_ts = value;
}

inline double Decoder::GetPacketDuration()
{
  return m_packet_duration;
}

inline bool Decoder::ProcessMsopPacket(const uint8_t* pkt, size_t size, uint16_t start_blk)
{
#if 0
  const static int CLOUD_POINT_MAX = 1000000;

  if (this->m_point_cloud && (this->m_point_cloud->GetSize() > CLOUD_POINT_MAX))
  {
     LIMIT_CALL(this->m_exception_callback(Error(ERRCODE_CLOUDOVERFLOW)), 1);
  }
#endif

  if (size != this->m_const_param.MSOP_LEN)
  {
     LIMIT_CALL(this->m_exception_callback(Error(ERRCODE_WRONGMSOPLEN)), 1);
     return false;
  }

  if (memcmp(pkt, this->m_const_param.MSOP_ID, this->m_const_param.MSOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->m_exception_callback(Error(ERRCODE_WRONGMSOPID)), 1);
    return false;
  }
  return DecodeMsopPacket(pkt, size, start_blk);
}

inline bool Decoder::toSplite(const uint8_t* pkt, size_t size, uint16_t& start_blk)
{
  if (size != this->m_const_param.MSOP_LEN) 
  { 
    LIMIT_CALL(this->m_exception_callback( Error(ERRCODE_WRONGMSOPLEN)), 1);
    return false;
  }

  if (memcmp(pkt, this->m_const_param.MSOP_ID, this->m_const_param.MSOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->m_exception_callback(Error(ERRCODE_WRONGMSOPID)), 1);
    return false;
  }
  return toSplitFrame(pkt, size, start_blk);
}

}  // namespace lidar
