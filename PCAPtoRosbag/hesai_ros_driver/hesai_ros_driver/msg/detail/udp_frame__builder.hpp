// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hesai_ros_driver:msg/UdpFrame.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__BUILDER_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hesai_ros_driver/msg/detail/udp_frame__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hesai_ros_driver
{

namespace msg
{

namespace builder
{

class Init_UdpFrame_packets
{
public:
  explicit Init_UdpFrame_packets(::hesai_ros_driver::msg::UdpFrame & msg)
  : msg_(msg)
  {}
  ::hesai_ros_driver::msg::UdpFrame packets(::hesai_ros_driver::msg::UdpFrame::_packets_type arg)
  {
    msg_.packets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hesai_ros_driver::msg::UdpFrame msg_;
};

class Init_UdpFrame_header
{
public:
  Init_UdpFrame_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UdpFrame_packets header(::hesai_ros_driver::msg::UdpFrame::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UdpFrame_packets(msg_);
  }

private:
  ::hesai_ros_driver::msg::UdpFrame msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hesai_ros_driver::msg::UdpFrame>()
{
  return hesai_ros_driver::msg::builder::Init_UdpFrame_header();
}

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__BUILDER_HPP_
