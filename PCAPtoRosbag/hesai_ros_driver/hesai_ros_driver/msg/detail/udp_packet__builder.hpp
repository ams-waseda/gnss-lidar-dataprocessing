// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hesai_ros_driver:msg/UdpPacket.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__UDP_PACKET__BUILDER_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__UDP_PACKET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hesai_ros_driver/msg/detail/udp_packet__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hesai_ros_driver
{

namespace msg
{

namespace builder
{

class Init_UdpPacket_size
{
public:
  explicit Init_UdpPacket_size(::hesai_ros_driver::msg::UdpPacket & msg)
  : msg_(msg)
  {}
  ::hesai_ros_driver::msg::UdpPacket size(::hesai_ros_driver::msg::UdpPacket::_size_type arg)
  {
    msg_.size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hesai_ros_driver::msg::UdpPacket msg_;
};

class Init_UdpPacket_data
{
public:
  explicit Init_UdpPacket_data(::hesai_ros_driver::msg::UdpPacket & msg)
  : msg_(msg)
  {}
  Init_UdpPacket_size data(::hesai_ros_driver::msg::UdpPacket::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_UdpPacket_size(msg_);
  }

private:
  ::hesai_ros_driver::msg::UdpPacket msg_;
};

class Init_UdpPacket_stamp
{
public:
  Init_UdpPacket_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UdpPacket_data stamp(::hesai_ros_driver::msg::UdpPacket::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_UdpPacket_data(msg_);
  }

private:
  ::hesai_ros_driver::msg::UdpPacket msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hesai_ros_driver::msg::UdpPacket>()
{
  return hesai_ros_driver::msg::builder::Init_UdpPacket_stamp();
}

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__UDP_PACKET__BUILDER_HPP_
