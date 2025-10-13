/*********************************************************************************************************************
Copyright (c) 2020 InnoLight
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

//#include <cstdint>
#include <vector>
#include <string>

namespace lidar
{

struct Packet
{
  double timestamp;
  uint32_t seq;
  uint8_t is_difop;
  uint8_t is_frame_begin;
  std::string frame_id;  ///< Packet message frame id

  Packet(const Packet& msg)
  {
    timestamp = msg.timestamp;
    seq = msg.seq;
    is_difop = msg.is_difop;
    is_frame_begin = msg.is_frame_begin;
    m_buf.assign(msg.m_buf.begin(), msg.m_buf.end());
  }

  Packet(size_t size = 0)
   : timestamp(0.0), 
     seq(0), 
     is_difop(0), 
     is_frame_begin(0),
     frame_id("innolidar")
  {
    m_buf.resize(size);
  }

  std::vector<uint8_t> m_buf;
};

}  // namespace lidar
