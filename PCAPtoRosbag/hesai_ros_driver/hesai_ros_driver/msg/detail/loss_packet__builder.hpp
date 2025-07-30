// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hesai_ros_driver:msg/LossPacket.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__BUILDER_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hesai_ros_driver/msg/detail/loss_packet__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hesai_ros_driver
{

namespace msg
{

namespace builder
{

class Init_LossPacket_total_packet_loss_count
{
public:
  explicit Init_LossPacket_total_packet_loss_count(::hesai_ros_driver::msg::LossPacket & msg)
  : msg_(msg)
  {}
  ::hesai_ros_driver::msg::LossPacket total_packet_loss_count(::hesai_ros_driver::msg::LossPacket::_total_packet_loss_count_type arg)
  {
    msg_.total_packet_loss_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hesai_ros_driver::msg::LossPacket msg_;
};

class Init_LossPacket_total_packet_count
{
public:
  Init_LossPacket_total_packet_count()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LossPacket_total_packet_loss_count total_packet_count(::hesai_ros_driver::msg::LossPacket::_total_packet_count_type arg)
  {
    msg_.total_packet_count = std::move(arg);
    return Init_LossPacket_total_packet_loss_count(msg_);
  }

private:
  ::hesai_ros_driver::msg::LossPacket msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hesai_ros_driver::msg::LossPacket>()
{
  return hesai_ros_driver::msg::builder::Init_LossPacket_total_packet_count();
}

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__BUILDER_HPP_
