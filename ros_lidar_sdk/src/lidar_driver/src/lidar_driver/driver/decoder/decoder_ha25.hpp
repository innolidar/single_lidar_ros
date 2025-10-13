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
#include <lidar_driver/driver/decoder/decoder.hpp>

namespace lidar
{

#pragma pack(push, 1)
typedef struct
{
  uint8_t id[2];
  uint8_t function_flag[2];
  uint16_t angle_start;
  uint16_t angle_end;
  //uint32_t ms;
  uint8_t ms[6];
  //uint16_t speed;
  uint16_t number;
}HA25MsopHeader;

typedef struct
{
  uint16_t  intensity;
  uint16_t distance;
} HA25MsopBlock;


typedef struct
{
  uint16_t crc;
}HA25MsopEnd;

typedef struct
{
  HA25MsopHeader header;
  HA25MsopBlock  blocks[54];
  HA25MsopEnd end;
} HA25MsopPacket;

struct HA25SpliteStrategy
{
public:
  HA25SpliteStrategy()
  {

  }
  ~HA25SpliteStrategy()
  {

  }
  bool IsNewFrame(const uint16_t &value)
  {
    /*if(m_start==false)
    {
      m_start=true;
      m_last_angle=value;
      return false;
    }
    if(m_last_angle>=value)
    {
        m_last_angle=value;
        return true;
    }*/
    if(m_start==false)
    {
      m_start=true;
      m_last_angle=value;
      return false;
    }
    if(value-m_last_angle>1000)
    {
        m_last_angle=value;
        return true;
    }
    m_last_angle=value;
    return false;
  }
private:
  bool m_start{false};
  uint16_t m_last_angle{0};
};

#pragma pack(pop)

class DecoderHA25 : public Decoder
{
public: 
  explicit DecoderHA25(const DecoderParam& param);
  virtual ~DecoderHA25(){};

  virtual bool toSplitFrame(const uint8_t* pkt, size_t size, uint16_t& start_blk);
  virtual bool DecodeMsopPacket(const uint8_t* pkt, size_t size, uint16_t start_blk);

#ifndef UNIT_TEST
protected:
#endif
  template <typename T_Msop, typename T_Block>
  bool toSplitFrameCommon(const uint8_t* packet, size_t size, uint16_t& start_blk);
  static DecoderConstParam& GetConstParam();
  template <typename T_BlockIterator>
  bool InternDecodeMsopPkt(const uint8_t* pkt, size_t size, uint16_t start_blk);
private:
  int64_t m_start_timestamp{0};
  int64_t m_start_sys_timestamp{0};
  HA25SpliteStrategy  m_splite_strategy;
};

inline DecoderConstParam& DecoderHA25::GetConstParam()
{
  static DecoderConstParam param = 
  {
    234    // msop len
    , 4     // msop id len
    , {0x55, 0xaa, 0x07, 0x00} // msop id
    , 54           // blocks per packet
    , 0.001f        // distance resolution
    , -135.0 * M_PI / 180.0f       // LIDAR_START_ANGLE 雷达起始角度
    ,135.0 * M_PI / 180.0f          //LIDAR_END_ANGLE 雷达终止角度
    ,0.25 * M_PI / 180.0f           //LIDAR_ANGLE_INCREMENT 雷达角度分辨率
    ,0.05f                          //LIDAR_RANGE_MIN 雷达最小测距范围
    ,30.0f                          //LIDAR_RANGE_MAX 雷达最大测距范围
  };
  float blk_ts = 55.50f;
  //param.BLOCK_DURATION = blk_ts / 1000000;

  return param;
}

template <typename T_Msop, typename T_Block>
bool DecoderHA25::toSplitFrameCommon(const uint8_t* packet, size_t size, uint16_t& start_blk)
{
  const T_Msop& pkt = *(const T_Msop*)(packet);

  if (this->m_splite_strategy.IsNewFrame(pkt.header.angle_start))
    {
      start_blk = 0;
      return true;
    }
  return false;
}

inline DecoderHA25::DecoderHA25(const DecoderParam& param)
  : Decoder(GetConstParam(), param)
{
  const static double FRAME_DURATION = 0.1; //0.05
  this->m_packet_duration = FRAME_DURATION / 20;
}

inline bool DecoderHA25::toSplitFrame(const uint8_t* pkt, size_t size, uint16_t& start_blk)
{
  return this->template toSplitFrameCommon<HA25MsopPacket, HA25MsopBlock>(pkt, size, start_blk); 
}

inline bool DecoderHA25::DecodeMsopPacket(const uint8_t* pkt, size_t size, uint16_t start_blk)
{
  bool ret = false;
  const HA25MsopPacket& packet = *(const HA25MsopPacket*)(pkt);
  uint8_t szData[8]{0};
  memcpy(szData,packet.header.ms,6);
    //m_start_timestamp=packet.header.ms;
  uint64_t  timestamp=*(uint64_t*)szData;
  if(m_start_timestamp==0)
  {
    m_start_timestamp=timestamp;
    m_start_sys_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  
  for (uint32_t i = start_blk; i < this->m_const_param.BLOCKS_PER_PKT; i++)
  {
    const HA25MsopBlock& block = packet.blocks[i];
    int32_t angle_h=packet.header.angle_start+i*25;
    if (this->m_splite_strategy.IsNewFrame(angle_h))
    {
      this->m_laser_msg->timestamp=m_start_timestamp; 
      //this->m_laser_msg->timestamp=m_start_sys_timestamp;                                      ///<:雷达时间戳(毫秒)
      this->m_laser_msg->angle_min=this->m_const_param.LIDAR_START_ANGLE;   ///<:测量起始角度(弧度制)
      this->m_laser_msg->angle_max=this->m_const_param.LIDAR_END_ANGLE;                                 ///<:测量终止角度(弧度制)
      this->m_laser_msg->angle_increment=this->m_const_param.LIDAR_ANGLE_INCREMENT;                            ///<:角度分辨率(弧度制)
      //this->m_laser_msg->time_increment{0.f};                              ///<:时间分辨率(单位 ms)
      //this->m_laser_msg->scan_time=static_cast<float>(packet.header.ms-m_start_timestamp) * 0.75f;                                  ///<:扫描周期(单位 ms)
      this->m_laser_msg->scan_time=static_cast<float>(timestamp-m_start_timestamp) * 0.75f;  
      this->m_laser_msg->time_increment=this->m_laser_msg->scan_time / (float)this->m_laser_msg->ranges.size();
      this->m_laser_msg->range_min= this->m_const_param.LIDAR_RANGE_MIN;                                  ///<:最小有效测量范围(单位 m)
      this->m_laser_msg->range_max= this->m_const_param.LIDAR_RANGE_MAX;                                    ///<:最大有效测量范围(单位 m)
      
      this->m_splite_frame_callback();
      //m_start_timestamp=packet.header.ms;
      m_start_timestamp=timestamp;
      m_start_sys_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
      ret = true;
    }
    
    this->m_laser_msg->PushData(block.intensity,block.distance*0.001);
  }
  return ret;
}

template <typename T_BlockIterator>
inline bool DecoderHA25::InternDecodeMsopPkt(const uint8_t* packet, size_t size, uint16_t start_blk)
{
  bool ret = false;
  return ret;
}

}  // namespace lidar
