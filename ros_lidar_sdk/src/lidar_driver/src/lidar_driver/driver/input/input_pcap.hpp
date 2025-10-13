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
#include <lidar_driver/driver/input/input.hpp>

#include <sstream>

#include <pcap.h>

namespace lidar
{
class InputPcap : public Input
{
public:
  InputPcap(const InputParam& input_param)
    : Input(input_param), m_pcap(NULL), m_pcap_offset(ETH_HDR_LEN), m_pcap_tail(0)
  {
    if (input_param.use_vlan)
    {
      m_pcap_offset += VLAN_HDR_LEN;
    }

    m_pcap_offset += input_param.user_layer_bytes;
    m_pcap_tail   += input_param.tail_layer_bytes;

    std::stringstream msop_stream;
    if (input_param.use_vlan)
    {
      msop_stream << "vlan && ";
    }

    msop_stream << "(udp";

    if (input_param.msop_port != 0)
    {
      msop_stream << " dst port " << input_param.msop_port;

      if ((input_param.difop_port != 0) && (input_param.difop_port != input_param.msop_port))
      {
        msop_stream << " or " << input_param.difop_port;
      }
    }

    msop_stream << ")";

    m_msop_filter_str = msop_stream.str();
  }

  virtual bool Init();
  virtual int RecvPacket(char* buffer,const int &len);
  virtual ~InputPcap();

  void GetFilePos(fpos_t* pos);
  void SetFilePos(fpos_t* pos);

protected:

  pcap_t*     m_pcap;
  size_t      m_pcap_offset;
  size_t      m_pcap_tail;
  std::string m_msop_filter_str;
  bpf_program m_msop_filter;
};

inline bool InputPcap::Init()
{
  char errbuf[PCAP_ERRBUF_SIZE];
  m_pcap = pcap_open_offline(m_input_param.pcap_path.c_str(), errbuf);
  if (m_pcap == NULL)
  {
    m_exception_callback(Error(ERRCODE_PCAPWRONGPATH));
    return false;
  }

  pcap_compile(m_pcap, &m_msop_filter, m_msop_filter_str.c_str(), 1, 0xFFFFFFFF);
  return true;
}

inline InputPcap::~InputPcap()
{
  if (m_pcap != NULL)
  {
    pcap_close(m_pcap);
    m_pcap = NULL;
  }
}

inline void InputPcap::GetFilePos(fpos_t* pos)
{
#ifdef _WIN32
  pcap_fgetpos(m_pcap, pos);
#else
  FILE* f = pcap_file(m_pcap);
  fgetpos(f, pos);
#endif
}

inline void InputPcap::SetFilePos(fpos_t* pos)
{
#ifdef _WIN32
  pcap_fsetpos(m_pcap, pos);
#else
  FILE* f = pcap_file(m_pcap);
  fsetpos(f, pos);
#endif
}

inline int  InputPcap::RecvPacket(char* buffer,const int &len)
{
  struct pcap_pkthdr* header;
  const uint8_t* pkt_data;
  int ret = pcap_next_ex(m_pcap, &header, &pkt_data);
  if (ret < 0)  // reach file end.
  {
    pcap_close(m_pcap);
    m_pcap = NULL;

    if (m_input_param.pcap_repeat)
    {
      m_exception_callback(Error(ERRCODE_PCAPREPEAT));

      char errbuf[PCAP_ERRBUF_SIZE];
      m_pcap = pcap_open_offline(m_input_param.pcap_path.c_str(), errbuf);
      return 0;
    }
    else
    {
      m_exception_callback(Error(ERRCODE_PCAPEXIT));
      return -1;
    }
  }

  if (pcap_offline_filter(&m_msop_filter, header, pkt_data) != 0)
  {
    //ParsePacket(buffer,len,pkt_data, header->len);
    if(len>header->len)
    {
      memcpy(buffer, pkt_data + m_pcap_offset, header->len - m_pcap_offset - m_pcap_tail);
    }
    return header->len - m_pcap_offset - m_pcap_tail;
  }

  return 0;
}

}  // namespace lidar
