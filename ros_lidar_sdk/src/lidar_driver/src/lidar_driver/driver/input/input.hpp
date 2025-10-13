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

#include <lidar_driver/driver/driver_param.hpp>
#include <lidar_driver/utility/buffer.hpp>

#include <functional>
#include <cstring>

#define VLAN_HDR_LEN  4
#define ETH_HDR_LEN   42
#define ETH_LEN       (ETH_HDR_LEN + VLAN_HDR_LEN + 1500)
#define IP_LEN        65536 
#define UDP_HDR_LEN   8

namespace lidar
{

class Input
{
public:
  Input(const InputParam& input_param);
  virtual ~Input(){};

  inline void RegisterCallback(const std::function<void(const Error&)> &exception_callback);

  virtual bool Init() = 0;
  virtual int  RecvPacket(char* buffer,const int &len) = 0;
  virtual bool SendCommand(char *szCommand,const int &size){}
protected:
  InputParam                        m_input_param;
  std::function<void(const Error&)> m_exception_callback;
};

inline Input::Input(const InputParam& input_param)
  : m_input_param(input_param)
{

}

inline void Input::RegisterCallback(const std::function<void(const Error&)> &exception_callback)
{
  m_exception_callback   = exception_callback;
}

}  // namespace lidar
