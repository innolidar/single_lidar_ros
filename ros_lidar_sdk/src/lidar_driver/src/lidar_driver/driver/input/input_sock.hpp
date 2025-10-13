
#pragma once
#include "input.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>

namespace lidar
{
class InputSock : public Input
{
public:
  InputSock(const InputParam& input_param)
    : Input(input_param)
    , m_fd(-1) 
    , m_pkt_buf_len(ETH_LEN)
    , m_sock_offset(0)
    , m_sock_tail(0)
  {
    m_sock_offset += input_param.user_layer_bytes;
    m_sock_tail   += input_param.tail_layer_bytes;
  }

  virtual bool Init();
  virtual int RecvPacket(char* buffer,const int &len);
  virtual ~InputSock();
  virtual bool SendCommand(char *szCommand,const int &size);

private:
  inline int CreateSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp);
protected:
  int     m_fd;
  size_t  m_pkt_buf_len;
  size_t  m_sock_offset;
  size_t  m_sock_tail;
  socklen_t     m_addr_len;
  sockaddr_in   m_addr_serv;
  std::shared_ptr<sockaddr_in> m_addr_client{nullptr};
};

inline bool InputSock::Init()
{
  int msop_fd = -1, difop_fd = -1;

  m_fd = CreateSocket(m_input_param.msop_port, m_input_param.host_address, m_input_param.group_address);
  if (m_fd < 0)
  {
    return false;
  }
  m_input_param.print();
  return true;
}

inline InputSock::~InputSock()
{
  if (m_fd >= 0)
  {
    close(m_fd);
  }
  m_fd= -1;
}

inline int InputSock::CreateSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp)
{
  
  int ret;
  int reuse = 1;

  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    perror("socket: ");
    return -1;
  }
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 50000;    ////50毫秒
  if(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
  {
        perror("setsockopt: ");
        close(fd);
        return -1;
  }
  ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  if (ret < 0)
  {
    perror("setsockopt: ");
    close(fd);
    return -1;
  }
  
  memset(&m_addr_serv, 0, sizeof(m_addr_serv));
  m_addr_serv.sin_family = AF_INET;
  m_addr_serv.sin_port = htons(port);
  m_addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
  m_addr_len = sizeof(m_addr_serv);
  ret = bind(fd, (struct sockaddr*)&m_addr_serv, sizeof(m_addr_serv));
  if (ret < 0)
  {
    perror("bind: ");
    close(fd);
    return -1;
  }
  return fd;
}
inline bool InputSock::SendCommand(char *szCommand,const int &size)
{
    if (m_fd>=0)
    {
        int send_num = sendto(m_fd, szCommand, size, 0,(sockaddr *)m_addr_client.get(), m_addr_len);
        if(send_num < 0)
        {
            std::cout<<"sendto:"<<std::strerror(errno)<<std::endl;
            return false;
        }
        return true;
    }
    return false;
}
inline int InputSock::RecvPacket(char* buffer,const int &len)
{
  if (m_fd>=0)
  {
    sockaddr_in addr_client;
    ssize_t ret = recvfrom(m_fd, buffer, len, 0, (sockaddr *) &addr_client, &m_addr_len);
    if (ret < 0)
    {
      perror("recvfrom: ");
      return -1;
    }
    else if (ret > 0)
    {
        if(m_addr_client==nullptr)
        {
            m_addr_client=std::make_shared<sockaddr_in>(addr_client);
            std::cout << inet_ntoa(addr_client.sin_addr) << ":" << ntohs(addr_client.sin_port)<<std::endl;
        }
        //pkt->setData(m_sock_offset, ret - m_sock_offset - m_sock_tail);
        return ret;
    }
  }
  return 0;
}

}  // namespace lidar

