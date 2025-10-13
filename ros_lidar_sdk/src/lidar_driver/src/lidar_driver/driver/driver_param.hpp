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

#include <lidar_driver/msg/data_types.hpp>
#include <lidar_driver/common/error_code.hpp>
#include <lidar_driver/common/lidar_common.hpp>
#include <lidar_driver/common/lidar_log.hpp>
#include <lidar_driver/msg/packet.hpp>
#include <string>
#include <map>
#include <string.h>
#include <functional>

namespace lidar
{
enum LidarType  ///< LiDAR type
{
  // mecha
  LIDAR_NONE = 0x00,
  LIDAR_HA25 =0x01,
  LIDAR_HA50 = 0x02,
};

inline size_t getLidarList(const LidarType** types)
{
  const static LidarType lidar_types[] = 
  {
    LIDAR_HA25,
    LIDAR_HA50,
  };

  if (types) 
  {
    *types = lidar_types;
  }

  return (sizeof(lidar_types)/sizeof(lidar_types[0]));
}


inline std::string lidarTypeToStr(const LidarType& type)
{
  std::string str = "";
  switch (type)
  {
    case LIDAR_HA25:
      str = "LIDAR_HA25";
      break;
    case LIDAR_HA50:
      str= "LIDAR_HA50";
      break;
    default:
      str = "ERROR";
      TOOL_ERROR << "TOOL_ERROR" << TOOL_REND;
  }
  return str;
}

inline LidarType strToLidarType(const std::string& type)
{
  if (type == "LIDAR_HA25")
  {
    return LIDAR_HA25;
  }
  else if (type == "LIDAR_HA50")
  {
    return LIDAR_HA50;
  }
  else
  {
    TOOL_ERROR << "Wrong lidar type: " << type << TOOL_REND;
    TOOL_ERROR << "Please give correct type: LIDAR_HA25,LIDAR_HA50" << TOOL_REND;
    exit(-1);
  }
}

enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
};

inline std::string inputTypeToStr(const InputType& type)
{
  std::string str = "";
  switch (type)
  {
    case ONLINE_LIDAR:
      str = "ONLINE_LIDAR";
      break;
    case PCAP_FILE:
      str = "PCAP_FILE";
      break;
    default:
      str = "ERROR";
      TOOL_ERROR << "TOOL_ERROR" << TOOL_REND;
  }
  return str;
}

struct DecoderParam  ///< LiDAR decoder parameter
{
  DecoderParam()
   : min_distance(0.0f), max_distance(0.0f),
     use_lidar_clock(false), dense_points(false), ts_first_point(false)
  {
  }
  float min_distance;            ///< Minimum distance of point cloud range
  float max_distance;            ///< Max distance of point cloud range
  float h_start_angle;           ///< Start angle of point cloud
  float h_end_angle;             ///< End angle of point cloud
  uint16_t num_blks_split;       ///< Number of packets in one frame, only be used when split_frame_mode=3
  bool use_lidar_clock;          ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points;             ///< true: discard NAN points; false: reserve NAN points
  bool ts_first_point;           ///< true: time-stamp point cloud with the first point; false: with the last point;
  bool use_calibrate_file;
  std::string calibrate_file;


  void print() const
  {
    TOOL_INFO << "------------------------------------------------------" << TOOL_REND;
    TOOL_INFO << "             Lidar Decoder Parameters " << TOOL_REND;
    TOOL_INFOL << "min_distance: " << min_distance << TOOL_REND;
    TOOL_INFOL << "max_distance: " << max_distance << TOOL_REND;
    TOOL_INFOL << "h_start_angle: " << h_start_angle << TOOL_REND;
    TOOL_INFOL << "h_end_angle: " << h_end_angle << TOOL_REND;
    TOOL_INFOL << "use_lidar_clock: " << use_lidar_clock << TOOL_REND;
    TOOL_INFO << "------------------------------------------------------" << TOOL_REND;
  }

};

struct InputParam  ///< The LiDAR input parameter
{
  InputParam()
   : host_address("0.0.0.0"), group_address("0.0.0.0"), msop_port(6699), difop_port(7788), 
     pcap_path(""), pcap_repeat(true), pcap_rate(1.0f), 
     use_vlan(false), user_layer_bytes(0), tail_layer_bytes(0)
  {

  }

  std::string host_address;        ///< Address of host
  std::string group_address;       ///< Address of multicast group
  uint16_t msop_port;              ///< MSOP packet port number
  uint16_t difop_port;             ///< DIFOP packet port number
  std::string pcap_path;           ///< Absolute path of pcap file
  bool pcap_repeat;                ///< true: The pcap bag will repeat play
  float pcap_rate;                 ///< Rate to read the pcap file
  bool use_vlan;                   ///< Vlan on-off
  uint16_t user_layer_bytes;       ///< Bytes of user layer. thers is no user layer if it is 0
  uint16_t tail_layer_bytes;       ///< Bytes of tail layer. thers is no tail layer if it is 0

  void print() const
  {
    TOOL_INFO << "------------------------------------------------------" << TOOL_REND;
    TOOL_INFO << "             InnoLight Input Parameters " << TOOL_REND;
    TOOL_INFOL << "msop_port: " << msop_port << TOOL_REND;
    TOOL_INFOL << "difop_port: " << difop_port << TOOL_REND;
    TOOL_INFOL << "host_address: " << host_address << TOOL_REND;
    TOOL_INFOL << "group_address: " << group_address << TOOL_REND;
    TOOL_INFOL << "pcap_path: " << pcap_path << TOOL_REND;
    TOOL_INFOL << "pcap_rate: " << pcap_rate << TOOL_REND;
    TOOL_INFOL << "pcap_repeat: " << pcap_repeat << TOOL_REND;
    TOOL_INFOL << "use_vlan: " << use_vlan << TOOL_REND;
    TOOL_INFOL << "user_layer_bytes: " << user_layer_bytes << TOOL_REND;
    TOOL_INFOL << "tail_layer_bytes: " << tail_layer_bytes << TOOL_REND;
    TOOL_INFO << "------------------------------------------------------" << TOOL_REND;
  }

};

struct DriverParam  ///< The LiDAR driver parameter
{
  DriverParam()
   : lidar_type(LIDAR_HA25), input_type(ONLINE_LIDAR), frame_id("lidar_driver")
  {
  }

  LidarType lidar_type;             ///< Lidar type
  InputType input_type;             ///< Input type
  std::string frame_id;             ///< The frame id of LiDAR mesage
  InputParam input_param;         ///< Input parameter
  DecoderParam decoder_param;     ///< Decoder parameter

  void print() const
  {
    TOOL_INFO << "------------------------------------------------------" << TOOL_REND;
    TOOL_INFO << "             Lidar Driver Parameters " << TOOL_REND;
    TOOL_INFOL << "input type: " << inputTypeToStr(input_type) << TOOL_REND;
    TOOL_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << TOOL_REND;
    TOOL_INFOL << "frame_id: "   << frame_id << TOOL_REND;
    TOOL_INFOL << "------------------------------------------------------" << TOOL_REND;

    input_param.print();
    decoder_param.print();
  }

};

struct ResolutionParam
{
  ResolutionParam()
  {
    h_angle=1.0f;
    v_angle=1.0f;
    distance_resolution=1.0f;
    calibration_distance=0.0f;

  }
  float h_angle;
  float v_angle;
  float distance_resolution;
  float calibration_distance;
};

struct DeviceInfo
{
  DeviceInfo()
  {
    memset (this, 0, sizeof(DeviceInfo));
  }

  uint8_t ssn[6];
  uint8_t mac[6];
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
  char angles[2048];
};

struct DeviceStatus
{
  DeviceStatus()
  {
    memset (this, 0, sizeof(DeviceStatus));
  }

  uint16_t rpm;
  float temperature;
};

}  // namespace lidar
