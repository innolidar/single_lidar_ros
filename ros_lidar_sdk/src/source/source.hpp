#pragma once
#include <lidar_driver/msg/data_types.hpp>
#include "utility/yaml_reader.hpp"
#include <functional>
namespace lidar
{
class DestinationLaserScan
{
public:
  typedef std::shared_ptr<DestinationLaserScan> Ptr;

  virtual void Init(const YAML::Node& config){}
  virtual void Start() {}
  virtual void Stop() {}
  virtual void SendLaserScan(const Laser_msg& msg) = 0;
  inline void RegisterServiceCallBack(const std::function<bool(const uint32_t &,std::string &)> &call_back)
  {
      m_call_back=call_back;
  }
  inline void RegisterGetTimeServiceCallBack(const std::function<bool(uint64_t &)> &call_back)
  {
      m_get_time_call_back=call_back;
  }
  inline void RegisterSetTimeServiceCallBack(const std::function<bool(const uint64_t &)> &call_back)
  {
      m_set_time_call_back=call_back;
  }
  virtual ~DestinationLaserScan() = default;
public:
  std::function<bool(const uint32_t &,std::string &)> m_call_back{nullptr};
  std::function<bool(uint64_t &)>                  m_get_time_call_back{nullptr};
  std::function<bool(const uint64_t &)>            m_set_time_call_back{nullptr};
};

enum SourceType
{
  MSG_FROM_LIDAR = 1,
  MSG_FROM_PCAP = 2,
};

class Source
{
public:
  typedef std::shared_ptr<Source> Ptr;

  virtual void Init(const YAML::Node& config) {}
  virtual void Start() {}
  virtual void Stop() {}
  virtual void RegisterLaserScanCallback(DestinationLaserScan::Ptr dst);
  virtual bool GetDeviceStatus(const int32_t &command_type,std::string &text)
  {
    return false;
  }
  virtual bool GetDeviceTimeStatus(uint64_t &ms)
  {
    return false;
  }
  virtual bool SetDeviceTimeStatus(const uint64_t &ms)
  {
    return false;
  }
  virtual ~Source() = default;
  Source(SourceType src_type);

protected:
  void SendLaserScan(std::shared_ptr<Laser_msg> msg);

  SourceType m_src_type;
  std::vector<DestinationLaserScan::Ptr> m_pc_cb_vec;
};

inline Source::Source(SourceType src_type)
  : m_src_type(src_type)
{
}

inline void Source::RegisterLaserScanCallback(DestinationLaserScan::Ptr dst)
{
  m_pc_cb_vec.emplace_back(dst);
}

inline void Source::SendLaserScan(std::shared_ptr<Laser_msg> msg)
{
  for (auto &iter : m_pc_cb_vec)
  {
    iter->SendLaserScan(*msg);
  }
}

}  // namespace lidar
