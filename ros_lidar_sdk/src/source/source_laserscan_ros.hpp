#pragma once
#include "source/source.hpp"
#include <chrono>
#include <ctime>
#if ROS_FOUND==1
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "ros_lidar_sdk/DeviceStatus.h"
#include "ros_lidar_sdk/GetDeviceTime.h"
#include "ros_lidar_sdk/SetDeviceTime.h"
namespace lidar
{
inline uint64_t ConvertToMilliseconds(
    uint16_t year, uint16_t month, uint16_t day,
    uint16_t hour, uint16_t minute, uint16_t second, uint16_t ms) 
{
    
    // 构造tm结构体（注意月份需-1，年份需-1900）
    struct tm timeinfo = {
        .tm_sec = static_cast<int>(second),
        .tm_min = static_cast<int>(minute),
        .tm_hour = static_cast<int>(hour),
        .tm_mday = static_cast<int>(day),
        .tm_mon = static_cast<int>(month - 1),  // 月份范围0-11
        .tm_year = static_cast<int>(year - 1900) // 年份从1900开始
    };
    std::cout<<timeinfo.tm_year<<" "<<timeinfo.tm_mon<<" "<<timeinfo.tm_mday<<" "<<timeinfo.tm_hour<<" "<<timeinfo.tm_min<<" "<<timeinfo.tm_sec<<std::endl;
    // 转换为time_t（秒级时间戳）
    time_t timestamp = timegm(&timeinfo);
    if (timestamp == -1) {
        throw std::runtime_error("Invalid time components");
    }
    
    // 转换为毫秒级时间戳
    std::cout<<"cover s:"<<static_cast<uint64_t>(timestamp)<<std::endl;
    return static_cast<uint64_t>(timestamp) * 1000 + ms;
}
inline sensor_msgs::LaserScan toRosMsg(const Laser_msg& msg, const std::string& frame_id,const bool &use_lidar_clock)
{
  static int count{0};
  sensor_msgs::LaserScan laser;

  laser.header.stamp = ros::Time::now();
  if(use_lidar_clock)
  {
    laser.header.stamp.sec=msg.timestamp/1000;
    laser.header.stamp.nsec=(msg.timestamp%1000)*1000;
  }
  laser.header.frame_id = frame_id;
  laser.header.seq = count++;
  laser.angle_min = msg.angle_min;
  laser.angle_max = msg.angle_max;
  laser.angle_increment = msg.angle_increment;
  laser.time_increment = msg.time_increment / 1000.0f;
  laser.scan_time = msg.scan_time / 1000.0f;
  laser.range_min = msg.range_min;
  laser.range_max = msg.range_max;
  laser.ranges.insert(laser.ranges.begin(),msg.ranges.begin(),msg.ranges.end());
  laser.intensities.insert(laser.intensities.begin(),msg.intensities.begin(),msg.intensities.end());
  return laser;
}

class DestinationLaserScanRos : public DestinationLaserScan
{
public:
  virtual void Init(const YAML::Node& config);
  virtual void SendLaserScan(const Laser_msg& msg);
  virtual ~DestinationLaserScanRos() = default;
  bool GetDeviceStatus(ros_lidar_sdk::DeviceStatus::Request &req,ros_lidar_sdk::DeviceStatus::Response &res);
  bool GetDeviceTimeStatus(ros_lidar_sdk::GetDeviceTime::Request &req,ros_lidar_sdk::GetDeviceTime::Response &res);
  bool SetDeviceTimeStatus(ros_lidar_sdk::SetDeviceTime::Request &req,ros_lidar_sdk::SetDeviceTime::Response &res);
private:
  std::shared_ptr<ros::NodeHandle> m_nh;
  ros::Publisher  m_pub;
  ros::ServiceServer m_service;
  ros::ServiceServer m_get_time_service;
  ros::ServiceServer m_set_time_service;
  std::string m_frame_id;
  bool m_is_enable_laserscan{false};
  bool m_is_enable_service{false};
  bool m_is_use_lidar_clock{false};
};


inline void DestinationLaserScanRos::Init(const YAML::Node& config)
{
  yamlRead<std::string>(config["ros"],"ros_frame_id", m_frame_id, "innolidar");
  yamlRead<bool>(config["ros"],"send_laserscan_ros", m_is_enable_laserscan, false);
  m_nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  if(m_is_enable_laserscan)
  {
    std::string ros_send_topic;
    yamlRead<std::string>(config["ros"],"ros_send_laserscan_topic", ros_send_topic, "ha25_laser");
    m_pub = m_nh->advertise<sensor_msgs::LaserScan>(ros_send_topic, 100,true);
    TOOL_DEBUG<<"ros_send_topic:"<<ros_send_topic<<TOOL_REND;
  }
  yamlRead<bool>(config["ros"],"use_lidar_clock", m_is_use_lidar_clock, false);
  yamlRead<bool>(config["ros"],"enable_service_ros", m_is_enable_service, false);
  if(m_is_enable_service)
  {
    

    std::string ros_service_name;
    yamlRead<std::string>(config["ros"],"ros_service_lidar_type", ros_service_name, "device_status");
    m_service=m_nh->advertiseService(ros_service_name,&DestinationLaserScanRos::GetDeviceStatus,this);

    std::string ros_get_time_service_name;
    yamlRead<std::string>(config["ros"],"ros_service_get_time_type", ros_get_time_service_name, "get_device_time");
    m_get_time_service=m_nh->advertiseService(ros_get_time_service_name,&DestinationLaserScanRos::GetDeviceTimeStatus,this);

    TOOL_DEBUG<<"ros_get_time_service_name:"<<ros_get_time_service_name<<TOOL_REND;

    std::string ros_set_time_service_name;
    yamlRead<std::string>(config["ros"],"ros_service_set_time_type", ros_set_time_service_name, "set_device_time");
    m_set_time_service=m_nh->advertiseService(ros_set_time_service_name,&DestinationLaserScanRos::SetDeviceTimeStatus,this);
    TOOL_DEBUG<<"ros_set_time_service_name:"<<ros_set_time_service_name<<TOOL_REND;
  }
  
}

inline void DestinationLaserScanRos::SendLaserScan(const Laser_msg& msg)
{
  //std::cout<<"SendLaserScan m_frame_id:"<<m_frame_id<<std::endl;
  m_pub.publish(toRosMsg(msg, m_frame_id,m_is_use_lidar_clock)); 
}

inline bool DestinationLaserScanRos::GetDeviceStatus(ros_lidar_sdk::DeviceStatus::Request &req,ros_lidar_sdk::DeviceStatus::Response &res)
{
  if(m_call_back)
  {
    std::cout<<"GetDeviceStatus type:"<<req.type<<std::endl;
    std::string text{""};
    if(m_call_back(req.type,text))
    {
      
      res.data=text;
      std::cout<<"GetDeviceStatus text:"<<text<<std::endl;
      return true;
    }
  }
  return false;
}

inline bool DestinationLaserScanRos::GetDeviceTimeStatus(ros_lidar_sdk::GetDeviceTime::Request &req,ros_lidar_sdk::GetDeviceTime::Response &res)
{
  if(m_get_time_call_back)
  {
    uint64_t ms{0};
    if(m_get_time_call_back(ms))
    {
      time_t t = ms / 1000;
      tm* utc_time = gmtime(&t);
      res.status=true;
      res.year=utc_time->tm_year+1900;
      res.month=utc_time->tm_mon+1;
      res.date=utc_time->tm_mday;
      res.hour=utc_time->tm_hour;
      res.minute=utc_time->tm_min;
      res.second=utc_time->tm_sec;
      res.ms=(ms%1000);
      std::cout<<"ms:"<<ms<<std::endl;
      return true;
    }
  }
  return false;
}

inline bool DestinationLaserScanRos::SetDeviceTimeStatus(ros_lidar_sdk::SetDeviceTime::Request &req,ros_lidar_sdk::SetDeviceTime::Response &res)
{
  if(m_set_time_call_back)
  {
    std::cout<<"SetDeviceTimeStatus"<<std::endl;
    uint64_t ms=ConvertToMilliseconds(req.year,req.month,req.date,req.hour,req.minute,req.second,req.ms);
    if(m_set_time_call_back(ms))
    {
        std::cout<<"ms:"<<ms<<std::endl;
        res.status=true;
        return true;
    }
  }
  return false;
}

}  // namespace lidar

//#endif  // ROS_FOUND

//#ifdef ROS2_FOUND
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include "lidar_sdk_msg/srv/device_status.hpp"
#include "lidar_sdk_msg/srv/get_device_time.hpp"
#include "lidar_sdk_msg/srv/set_device_time.hpp"

namespace lidar
{
inline uint64_t ConvertToMilliseconds(
    uint16_t year, uint16_t month, uint16_t day,
    uint16_t hour, uint16_t minute, uint16_t second, uint16_t ms) 
{
    
    // 构造tm结构体（注意月份需-1，年份需-1900）
    struct tm timeinfo = {
        .tm_sec = static_cast<int>(second),
        .tm_min = static_cast<int>(minute),
        .tm_hour = static_cast<int>(hour),
        .tm_mday = static_cast<int>(day),
        .tm_mon = static_cast<int>(month - 1),  // 月份范围0-11
        .tm_year = static_cast<int>(year - 1900) // 年份从1900开始
    };
    std::cout<<timeinfo.tm_year<<" "<<timeinfo.tm_mon<<" "<<timeinfo.tm_mday<<" "<<timeinfo.tm_hour<<" "<<timeinfo.tm_min<<" "<<timeinfo.tm_sec<<std::endl;
    // 转换为time_t（秒级时间戳）
    time_t timestamp = timegm(&timeinfo);
    if (timestamp == -1) {
        throw std::runtime_error("Invalid time components");
    }
    
    // 转换为毫秒级时间戳
    std::cout<<"cover s:"<<static_cast<uint64_t>(timestamp)<<std::endl;
    return static_cast<uint64_t>(timestamp) * 1000 + ms;
}
inline sensor_msgs::msg::LaserScan toRosMsg(const Laser_msg& msg, const std::string& frame_id,const rclcpp::Time &now,const bool &use_lidar_clock)
{
  //static int count{0};
  sensor_msgs::msg::LaserScan laser;
  laser.header.stamp = now; //double seconds = now.seconds();
  if(use_lidar_clock)
  {
    laser.header.stamp.sec=msg.timestamp/1000;
    laser.header.stamp.nanosec=(msg.timestamp%1000)*1000;
  }
  laser.header.frame_id = frame_id;
  //laser.header.seq = count++;
  laser.angle_min = msg.angle_min;
  laser.angle_max = msg.angle_max;
  laser.angle_increment = msg.angle_increment;
  laser.time_increment = msg.time_increment / 1000.0f;
  laser.scan_time = msg.scan_time / 1000.0f;
  laser.range_min = msg.range_min;
  laser.range_max = msg.range_max;
  laser.ranges.insert(laser.ranges.begin(),msg.ranges.begin(),msg.ranges.end());
  laser.intensities.insert(laser.intensities.begin(),msg.intensities.begin(),msg.intensities.end());
  return laser;
}

class DestinationLaserScanRos : virtual public DestinationLaserScan
{
public:

  virtual void Init(const YAML::Node& config);
  virtual void SendLaserScan(const Laser_msg& msg);
  virtual void Start() 
  {

  }
  virtual void Stop() {}
  virtual ~DestinationLaserScanRos()
  {
    if(m_ros_thread.joinable())
    {
      m_ros_thread.join();
    }
    TOOL_DEBUG<<"exit"<<TOOL_REND;
  }
  void GetDeviceStatus(const std::shared_ptr<lidar_sdk_msg::srv::DeviceStatus::Request> req,std::shared_ptr<lidar_sdk_msg::srv::DeviceStatus::Response> res);
  void GetDeviceTimeStatus(const std::shared_ptr<lidar_sdk_msg::srv::GetDeviceTime::Request> req,std::shared_ptr<lidar_sdk_msg::srv::GetDeviceTime::Response> res);
  void SetDeviceTimeStatus(const std::shared_ptr<lidar_sdk_msg::srv::SetDeviceTime::Request> req,std::shared_ptr<lidar_sdk_msg::srv::SetDeviceTime::Response> res);
private:
  std::shared_ptr<rclcpp::Node> m_node_ptr;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_pub;
  std::string m_frame_id;
  bool m_send_by_rows;
  rclcpp::Service<lidar_sdk_msg::srv::DeviceStatus>::SharedPtr m_service;
  rclcpp::Service<lidar_sdk_msg::srv::GetDeviceTime>::SharedPtr m_get_time_service;
  rclcpp::Service<lidar_sdk_msg::srv::SetDeviceTime>::SharedPtr m_set_time_service;
  bool m_is_enable_laserscan{false};
  bool m_is_enable_service{false};
  bool m_is_use_lidar_clock{false};
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor{nullptr};
  std::thread m_ros_thread;
};

inline void DestinationLaserScanRos::Init(const YAML::Node& config)
{

  yamlRead<std::string>(config["ros"],"ros_frame_id", m_frame_id, "base_laser");
  static int node_index = 0;
  std::stringstream node_name;
  node_name << "lidar_laser_destination_" << node_index++;
  m_node_ptr.reset(new rclcpp::Node(node_name.str()));
  yamlRead<bool>(config["ros"],"send_laserscan_ros", m_is_enable_laserscan, false);
  
  if(m_is_enable_laserscan)
  {
    std::string ros_send_topic;
    yamlRead<std::string>(config["ros"],"ros_send_laserscan_topic", ros_send_topic, "ha25_laser");
    m_pub = m_node_ptr->create_publisher<sensor_msgs::msg::LaserScan>(ros_send_topic, 100);
    TOOL_DEBUG<<"ros_send_topic:"<<ros_send_topic<<TOOL_REND;
  }
  yamlRead<bool>(config["ros"],"use_lidar_clock", m_is_use_lidar_clock, false);
  yamlRead<bool>(config["ros"],"enable_service_ros", m_is_enable_service, false);
  if(m_is_enable_service)
  {
    std::string ros_service_name;
    yamlRead<std::string>(config["ros"],"ros_service_lidar_type", ros_service_name, "device_status");
    //std::bind(&YourNodeClass::handle_request, this, _1, _2)
    m_service=m_node_ptr->create_service<lidar_sdk_msg::srv::DeviceStatus>(ros_service_name,std::bind(&DestinationLaserScanRos::GetDeviceStatus,this,std::placeholders::_1,std::placeholders::_2));
    /*m_service=m_node_ptr->create_service<ros_lidar_sdk::srv::DeviceStatus>(ros_service_name,[this](const std::shared_ptr<ros_lidar_sdk::srv::DeviceStatus::Request> req,std::shared_ptr<ros_lidar_sdk::srv::DeviceStatus::Response> res)
    {
        return this->GetDeviceStatus(req,res);
    });*/
    std::string ros_get_time_service_name;
    yamlRead<std::string>(config["ros"],"ros_service_time_type", ros_get_time_service_name, "device_get_time_status");
    m_get_time_service=m_node_ptr->create_service<lidar_sdk_msg::srv::GetDeviceTime>(ros_get_time_service_name,std::bind(&DestinationLaserScanRos::GetDeviceTimeStatus,this,std::placeholders::_1,std::placeholders::_2));
    
    std::string ros_set_time_service_name;
    yamlRead<std::string>(config["ros"],"ros_service_time_type", ros_set_time_service_name, "device_set_time_status");
    m_set_time_service=m_node_ptr->create_service<lidar_sdk_msg::srv::SetDeviceTime>(ros_set_time_service_name,std::bind(&DestinationLaserScanRos::SetDeviceTimeStatus,this,std::placeholders::_1,std::placeholders::_2));

    m_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    m_executor->add_node(m_node_ptr);
    m_ros_thread=std::thread([this](){
      m_executor->spin();
    });
    
    TOOL_DEBUG<<"ros_service_name:"<<ros_service_name<<TOOL_REND;
  }
  //rclcpp::spin(m_node_ptr);
}

inline void DestinationLaserScanRos::SendLaserScan(const Laser_msg& msg)
{
  m_pub->publish(toRosMsg(msg, m_frame_id,m_node_ptr->get_clock()->now(),m_is_use_lidar_clock));
}

void DestinationLaserScanRos::GetDeviceStatus(const std::shared_ptr<lidar_sdk_msg::srv::DeviceStatus::Request> req,std::shared_ptr<lidar_sdk_msg::srv::DeviceStatus::Response> res)
{
  if(m_call_back)
  {
    std::string text{""};
    if(m_call_back(req->type,text))
    {
      res->data=text;
      return ;
    }
  }
  return ;
}
void DestinationLaserScanRos::GetDeviceTimeStatus(const std::shared_ptr<lidar_sdk_msg::srv::GetDeviceTime::Request> req,std::shared_ptr<lidar_sdk_msg::srv::GetDeviceTime::Response> res)
{
  if(m_get_time_call_back)
  {
    uint64_t ms{0};
    if(m_get_time_call_back(ms))
    {
      time_t t = ms / 1000;
      tm* utc_time = gmtime(&t);
      res->status=true;
      res->year=utc_time->tm_year+1900;
      res->month=utc_time->tm_mon+1;
      res->date=utc_time->tm_mday;
      res->hour=utc_time->tm_hour;
      res->minute=utc_time->tm_min;
      res->second=utc_time->tm_sec;
      res->ms=(ms%1000);
      return ;
    }
  }
}
void DestinationLaserScanRos::SetDeviceTimeStatus(const std::shared_ptr<lidar_sdk_msg::srv::SetDeviceTime::Request> req,std::shared_ptr<lidar_sdk_msg::srv::SetDeviceTime::Response> res)
{
  if(m_set_time_call_back)
  {
    uint64_t ms=ConvertToMilliseconds(req->year,req->month,req->date,req->hour,req->minute,req->second,req->ms);
    //uint64_t ms=timegm(&ms);
    if(m_set_time_call_back(ms))
    {
        res->status=true;
        return ;
    }
  }
  return ;
}

}  // namespace lidar
#endif
