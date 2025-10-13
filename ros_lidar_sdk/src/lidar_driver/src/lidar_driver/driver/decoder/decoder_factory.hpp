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
#include <lidar_driver/driver/decoder/decoder_ha25.hpp>
#include <lidar_driver/driver/decoder/decoder_ha50.hpp>

namespace lidar
{

class DecoderFactory
{
public:

  static std::shared_ptr<Decoder> CreateDecoder(LidarType type, const DecoderParam& param);
};

inline std::shared_ptr<Decoder> DecoderFactory::CreateDecoder(LidarType type, const DecoderParam& param)
{
  std::shared_ptr<Decoder> ret_ptr=NULL;
  switch (type)
  {
    case LIDAR_HA25:
      TOOL_DEBUG << "DecoderHA25! " << TOOL_REND;
      ret_ptr=std::make_shared<DecoderHA25>(param);
      break;
    case LIDAR_HA50:
      TOOL_DEBUG << "LIDAR_HA50! " << TOOL_REND;
      ret_ptr=std::make_shared<DecoderHA50>(param);
      break;
    default:
      TOOL_ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! " << TOOL_REND;
      exit(-1);
  }
  return ret_ptr;
}

}  // namespace lidar
