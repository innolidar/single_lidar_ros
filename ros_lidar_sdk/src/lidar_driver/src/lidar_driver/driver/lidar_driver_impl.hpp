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
#include <lidar_driver/msg/packet.hpp>
#include <lidar_driver/common/lidar_common.hpp>
#include <lidar_driver/macro/version.hpp>
#include <lidar_driver/utility/sync_queue.hpp>
#include <lidar_driver/utility/buffer.hpp>
#include <lidar_driver/driver/input/input_factory.hpp>
#include <lidar_driver/driver/decoder/decoder_factory.hpp>

#include <sstream>

namespace lidar
{
const int TIME_OUT=100;
enum class CommandType:int
{
  NONE=-1,
  SN=0,
  SOFTWARE_VERSION=1,
};
inline std::string GetDriverVersion()
{
  std::stringstream stream;
  stream << LIDARDRIVER_VERSION_MAJOR << "."<< LIDARDRIVER_VERSION_MINOR << "."<< LIDARDRIVER_VERSION_PATCH;
  return stream.str();
}

class LidarDriverImpl
{
public:

  LidarDriverImpl();
  ~LidarDriverImpl();

  void RegisterExceptionCallback(const std::function<void(const Error &)> exception_callback)
  {
    m_exception_callback = exception_callback;
  }
  void RegisterLaserScanCallback(const std::function<std::shared_ptr<Laser_msg>()> &get_laserscan_callback,
                                  const std::function<void(std::shared_ptr<Laser_msg>)> &put_laserscan_callback)
  {
    m_get_laserscan_callback = get_laserscan_callback;
    m_put_laserscan_callback = put_laserscan_callback;
  }
 
  bool Init(const DriverParam& param);
  bool Start();
  void Stop();
  bool GetDeviceStatus(const int32_t &command_type,std::string &text);
  bool GetDeviceTimeStatus(uint64_t &ms);
  bool SetDeviceTimeStatus(const uint64_t &ms);
private:
  void RunExceptionCallback(const Error& error);
  Buffer* PacketGet(size_t size);
  void PacketPut(Buffer* pkt);
  void SplitFrame();
  void RecvPacket();
  void ProcessPacket();
  bool GetSN(std::string &serial_number);
  bool GetSoftwareVersion(std::string &software_version);
  void ClearBuffer();
  std::shared_ptr<Laser_msg> GetLaserScan();
  void SetLaserScanHeader(std::shared_ptr<Laser_msg> msg);
  unsigned short CRC16(unsigned char *p,unsigned int len);

  bool                                      m_exit{false};
  bool                                      m_init{false};
  bool                                      m_start{false};
  uint64_t                                  m_msec_to_delay{0};
  uint32_t                                  m_pkt_seq{0};
  uint32_t                                  m_laserscan_seq{0};
  std::shared_ptr<Input>                    m_pInput{nullptr};
  std::shared_ptr<Decoder>                  m_pDecoder{nullptr};
  DriverParam                               m_driver_param{};  
  std::thread                               m_recv_thread;
  std::thread                               m_handle_thread;
  SyncQueue<Buffer*>                        m_free_pkt_queue{};
  SyncQueue<Buffer*>                        m_pkt_queue{};
  std::mutex                                m_socket_mtx;
  std::condition_variable                   m_recv_signal;
  int                                       m_read_len{0};
  uint8_t                                   m_read_buf[2048]{0x00};
  std::function<std::shared_ptr<Laser_msg>()>      m_get_laserscan_callback{nullptr};
  std::function<void(std::shared_ptr<Laser_msg>)>  m_put_laserscan_callback{nullptr};
  std::function<void(const Error&)>                m_exception_callback{nullptr};
};

inline LidarDriverImpl::LidarDriverImpl()
  : m_msec_to_delay(0), m_pkt_seq(0), m_laserscan_seq(0), m_init(false), m_start(false)
{

}

inline LidarDriverImpl::~LidarDriverImpl()
{
  Stop();
  while (1)
  {
    Buffer* buf = m_pkt_queue.Pop();
    if (buf == NULL)
    {
        break;
    }
    delete buf;
    buf=NULL;
  }

  while (1)
  {
    Buffer* buf = m_free_pkt_queue.Pop();
    if (buf == NULL)
    {
      break;
    }
    delete buf;
    buf=NULL;
  }
  m_pInput.reset();
  m_pInput = NULL;
  m_pDecoder.reset();
  m_pDecoder = NULL;
}
inline std::shared_ptr<Laser_msg> LidarDriverImpl::GetLaserScan()
{
  while (1)
  {
    std::shared_ptr<Laser_msg> msg = m_get_laserscan_callback();
    if (msg)
    {
      return msg;
    }
    
    LIMIT_CALL(RunExceptionCallback(Error(ERRCODE_LASERSCANNULL)), 1);
  }
}

inline bool LidarDriverImpl::Init(const DriverParam& param)
{
  if (m_init)
  {
    return true;
  }
  // decoder
  m_pDecoder = DecoderFactory::CreateDecoder(param.lidar_type, param.decoder_param);
  m_pDecoder->EnableWritePacketTimes(false);
  // point cloud related
  m_pDecoder->SetLaserScan(GetLaserScan());
  m_pDecoder->RegisterCallback(std::bind(&LidarDriverImpl::RunExceptionCallback,this,std::placeholders::_1),
                              std::bind(&LidarDriverImpl::SplitFrame,this));

  //// input
  m_pInput = InputFactory::CreateInput(param.input_type, param.input_param);

  m_pInput->RegisterCallback(std::bind(&LidarDriverImpl::RunExceptionCallback,this,std::placeholders::_1));
  if (!m_pInput->Init())
  {
    m_pInput.reset();
    m_pInput = NULL;
    m_pDecoder.reset();
    m_pDecoder = NULL;
    return false;
  }
  if(param.input_type==ONLINE_LIDAR)
  {
    m_msec_to_delay=0;
  }
  else if(param.input_type==PCAP_FILE)
  {
    m_msec_to_delay = (uint64_t)(m_pDecoder->GetPacketDuration() / param.input_param.pcap_rate * 1000000);
  }
  m_driver_param = param;
  m_init = true;
  return true;
}
unsigned short LidarDriverImpl::CRC16(unsigned char *p,unsigned int len)
{
  unsigned short wcrc=0xffff;
  unsigned char temp;
  for(int i=0;i<len;i++)
  {
    temp=*p&0x00ff;
    p++;
    wcrc^=temp;
    for(int j=0;j<8;j++)
    {
      if(wcrc&0x0001)
      {
          wcrc>>=1;
          wcrc^=0xA001;
      }
      else
      {
        wcrc>>=1;
      }
    }
  }
  return wcrc;
}
inline void LidarDriverImpl::ClearBuffer()
{
  memset(m_read_buf,0,sizeof(m_read_buf));
  m_read_len=0;
}
inline bool LidarDriverImpl::GetSoftwareVersion(std::string &software_version)
{
  std::cout<<"GetSoftwareVersion"<<std::endl;
  unsigned char szData[7]={0xFE,0x03,0x20,0x06,0x00,0x0F,0xFA};
  std::unique_lock<std::mutex> lock(m_socket_mtx);
  ClearBuffer();
  if(!m_pInput->SendCommand((char*)szData,7))
  {
    return false;
  }
  auto status=m_recv_signal.wait_for(lock,std::chrono::milliseconds(TIME_OUT));
  if(status==std::cv_status::timeout)
  {
    if(m_read_len==13)
    {
      unsigned char szFlag[5]={0xfe,0x03,0x20,0x06,0x00};
      if(memcmp(m_read_buf,szFlag,5)==0)
      {
        char szData[32]{0x00};
        memcpy(szData,m_read_buf+5,6);
        software_version.append(szData);
        std::cout<<"software_version:"<<software_version<<std::endl;
        ClearBuffer();
        return true;
      }
    }
    else
    {
      std::cout<<"std::cv_status::timeout m_read_len:"<<m_read_len<<std::endl;
        ClearBuffer();
        return false;
    }  
  }
  else
  {
    if(m_read_len==21)
    {
      unsigned char szFlag[5]={0xfe,0x03,0x20,0x06,0x00};
      if(memcmp(m_read_buf,szFlag,5)==0)
      {
        char szData[32]{0x00};
        memcpy(szData,m_read_buf+5,6);
        software_version.append(szData);
        std::cout<<"software_version:"<<software_version<<std::endl;
        ClearBuffer();
        return true;
      }
    }
  }
  return false;
}

inline bool LidarDriverImpl::GetSN(std::string &serial_number)
{
  std::cout<<"GetSN"<<std::endl;
  unsigned char szData[7]={0xFE,0x03,0x00,0x0E,0x00,0x09,0xF0};
  std::unique_lock<std::mutex> lock(m_socket_mtx);
  ClearBuffer();
  if(!m_pInput->SendCommand((char*)szData,7))
  {
    return false;
  }
  auto status=m_recv_signal.wait_for(lock,std::chrono::milliseconds(TIME_OUT));
  if(status==std::cv_status::timeout)
  {
    if(m_read_len==21)
    {
      unsigned char szFlag[5]={0xfe,0x03,0x00,0x0e,0x00};
      if(memcmp(m_read_buf,szFlag,5)==0)
      {
        char szData[32]{0x00};
        memcpy(szData,m_read_buf+5,11);
        serial_number.append(szData);
        std::cout<<"serial_number:"<<serial_number<<std::endl;
        ClearBuffer();
        return true;
      }
    }
    else
    {
      std::cout<<"std::cv_status::timeout m_read_len:"<<m_read_len<<std::endl;
        ClearBuffer();
        return false;
    }  
  }
  else
  {
    if(m_read_len==21)
    {
      unsigned char szFlag[5]={0xfe,0x03,0x00,0x0e,0x00};
      if(memcmp(m_read_buf,szFlag,5)==0)
      {
        char szData[32]{0x00};
        memcpy(szData,m_read_buf+5,11);
        serial_number.append(szData);
        std::cout<<"serial_number:"<<serial_number<<std::endl;
        ClearBuffer();
        return true;
      }
    }
  }
  return false;
}



inline bool LidarDriverImpl::Start()
{
  if (m_start)
  {
    return true;
  }
  if (!m_init)
  {
    m_exception_callback(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }
  m_exit = false;
  m_handle_thread = std::thread(std::bind(&LidarDriverImpl::ProcessPacket,this));
  m_recv_thread =std::thread(std::bind(&LidarDriverImpl::RecvPacket,this));
  m_start = true;
  return true;
}

inline void LidarDriverImpl::Stop()
{
  if (!m_start)
  {
    return;
  }

  m_exit = true;
  
  if(m_recv_thread.joinable())
  {
    m_recv_thread.join();
  }
  if(m_handle_thread.joinable())
  {
    m_handle_thread.join();
  }
  
  m_start = false;
}

inline bool LidarDriverImpl::GetDeviceStatus(const int32_t &command_type,std::string &text)
{
  bool status{false};
  CommandType type=(CommandType)command_type;
  switch(type)
  {
  case CommandType::SN:
    status=GetSN(text);
    break;
  case CommandType::SOFTWARE_VERSION:
    status=GetSN(text);
    break;
  }
  return status;
}

inline bool LidarDriverImpl::GetDeviceTimeStatus(uint64_t &ms)
{
  std::cout<<"GetDeviceTimeStatus"<<std::endl;
  unsigned char szData[7]={0xFE,0x03,0x21,0x06,0x00,0x00,0x00};
  uint16_t send_sum=CRC16(szData,5);
  szData[5]=send_sum;
  szData[6]=(send_sum>>8);
  std::unique_lock<std::mutex> lock(m_socket_mtx);
  ClearBuffer();
  if(!m_pInput->SendCommand((char*)szData,7))
  {
    return false;
  }
  auto status=m_recv_signal.wait_for(lock,std::chrono::milliseconds(TIME_OUT));
  if(status==std::cv_status::timeout)
  {
    if(m_read_len==13)
    {
      unsigned char szFlag[5]={0xFE,0x03,0x21,0x06,0x00};
      if(memcmp(m_read_buf,szFlag,5)==0)
      {
        unsigned char szData[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        memcpy(szData,m_read_buf+5,6);
        ms=*(uint64_t*)szData;
        //ms=m_read_buf[5]+m_read_buf[6]*256+m_read_buf[7]*256*256+m_read_buf[8]*256*256*256+m_read_buf[9]*256*256*256*256+m_read_buf[10]*256*256*256*256*256;
        std::cout<<"ms:"<<ms<<std::endl;
        ClearBuffer();
        return true;
      }
    }
    else
    {
        std::cout<<"std::cv_status::timeout m_read_len:"<<m_read_len<<std::endl;
        ClearBuffer();
        return false;
    }  
  }
  else
  {
    if(m_read_len==13)
    {
      unsigned char szFlag[5]={0xFE,0x03,0x21,0x06,0x00};
      if(memcmp(m_read_buf,szFlag,5)==0)
      {
        unsigned char szData[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        memcpy(szData,m_read_buf+5,6);
        ms=*(uint64_t*)szData;
        //ms=m_read_buf[5]+m_read_buf[6]*256+m_read_buf[7]*256*256+m_read_buf[8]*256*256*256+m_read_buf[9]*256*256*256*256+m_read_buf[10]*256*256*256*256*256;
        std::cout<<"ms:"<<ms<<std::endl;
        ClearBuffer();
        return true;
      }
    }
  }
  return false;
}

inline bool LidarDriverImpl::SetDeviceTimeStatus(const uint64_t &ms)
{
  std::cout<<"SetDeviceTimeStatus"<<std::endl;
  unsigned char szData[13]={0xFE,0x06,0x21,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  szData[5]=ms;
  szData[6]=(ms>>8);
  szData[7]=(ms>>16);
  szData[8]=(ms>>24);
  szData[9]=(ms>>32);
  szData[10]=(ms>>40);
  uint16_t send_sum=CRC16(szData,11);
  szData[11]=send_sum;
  szData[12]=(send_sum>>8);
  std::unique_lock<std::mutex> lock(m_socket_mtx);
  ClearBuffer();
  if(!m_pInput->SendCommand((char*)szData,13))
  {
    return false;
  }
  auto status=m_recv_signal.wait_for(lock,std::chrono::milliseconds(TIME_OUT));
  if(status==std::cv_status::timeout)
  {
    if(m_read_len==7)
    {
      unsigned char szFlag[7]={0xfe,0x06,0x21,0x06,0x00,0x5e,0xf6};
      if(memcmp(m_read_buf,szFlag,7)==0)
      {
        ClearBuffer();
        return true;
      }
    }
    else
    {
        std::cout<<"std::cv_status::timeout m_read_len:"<<m_read_len<<std::endl;
        ClearBuffer();
        return false;
    }  
  }
  else
  {
    if(m_read_len==7)
    {
      unsigned char szFlag[7]={0xfe,0x06,0x21,0x06,0x00,0x5e,0xf6};
      if(memcmp(m_read_buf,szFlag,7)==0)
      {
        
        ClearBuffer();
        return true;
      }
    }
  }
  return false;
}

inline void LidarDriverImpl::RunExceptionCallback(const Error& error)
{
  if (m_exception_callback)
  {
    m_exception_callback(error);
  }
}

inline Buffer* LidarDriverImpl::PacketGet(size_t size)
{
  Buffer* pkt = m_free_pkt_queue.Pop();
  if (pkt != NULL)
  {
    return pkt;
  }
  return new Buffer(size);
}

inline void LidarDriverImpl::PacketPut(Buffer* pkt)
{
  const static size_t PACKET_POOL_MAX = 8192;

  size_t sz = m_pkt_queue.Push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    LIMIT_CALL(RunExceptionCallback(Error(ERRCODE_PKTBUFOVERFLOW)), 1);
    m_pkt_queue.Clear();
  }
}
inline void LidarDriverImpl::RecvPacket()
{
  while (!m_exit)
  {
    char szData[2048]{0x00};
    int size = m_pInput->RecvPacket(szData,2048);
    if (size == 234 | size==558)
    {
      Buffer* buffer=this->PacketGet(ETH_LEN);
      memcpy(buffer->data(), szData, size);
      buffer->setData(0, size);
      PacketPut(buffer);
    }
    else if (size>0)
    {
      std::unique_lock<std::mutex> lock(m_socket_mtx);
      memcpy(m_read_buf+m_read_len,szData,size);
      m_read_len=m_read_len+size;
      lock.unlock();
      uint32_t short sum=CRC16(m_read_buf,m_read_len);
      if(memcmp(m_read_buf+m_read_len-2,&sum,2)==0)
      {
        m_recv_signal.notify_all();
      }
      
    }
    else
    {
      
    }
    inno_msleep((uint32_t)m_msec_to_delay);
  }
}

inline void LidarDriverImpl::ProcessPacket()
{
  while (!m_exit)
  {
    Buffer* pkt = m_pkt_queue.PopWait();
    if (pkt == NULL)
    {
      continue;
    }
    uint8_t* id = pkt->data();
    if (*id == 0x55)
    {
      bool pkt_to_split = m_pDecoder->ProcessMsopPacket(pkt->data(), pkt->dataSize());
    }
    else
    {
      //std::cout<<"DecodeMsopPacket "<<(uint32_t)*id<<std::endl;
      //m_pDecoder->ProcessDifopPacket(pkt->data(), pkt->dataSize());
      //RunPacketCallBack(pkt->data(), pkt->dataSize(), 0, true, false); // difop packet
    }

    m_free_pkt_queue.Push(pkt);
  }
}

inline void LidarDriverImpl::SplitFrame()
{
  std::shared_ptr<Laser_msg> msg = m_pDecoder->GetLaserScan();
  //if (msg->ranges.size() > 0)
  {
    SetLaserScanHeader(msg);
    m_put_laserscan_callback(msg);
    m_pDecoder->SetLaserScan(GetLaserScan());
  }
  //else
  {
    //RunExceptionCallback(Error(ERRCODE_ZEROPOINTS));
  }
}

inline void LidarDriverImpl::SetLaserScanHeader(std::shared_ptr<Laser_msg> msg)
{
  msg->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
  //msg->seq = m_laserscan_seq++;
 
 
  //std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
  
  // 输出系统时间
  //std::cout << std::ctime(&currentTime) << std::endl;
  //msg->timestamp = ts;
  //msg->timestamp = GetSystemTime();
  /*msg->timestamp=msg->points[0].timestamp;
  msg->is_dense = m_driver_param.decoder_param.dense_points;
  if (msg->is_dense)
  {
    msg->height = 1;
    msg->width = (uint32_t)msg->points.size();
  }
  else
  {
    msg->height = height;
    msg->width = (uint32_t)msg->points.size() / msg->height;
  }
  */
  msg->frame_id = m_driver_param.frame_id;
}

}  // namespace lidar
