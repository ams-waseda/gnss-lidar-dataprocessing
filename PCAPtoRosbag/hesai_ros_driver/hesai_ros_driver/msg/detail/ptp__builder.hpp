// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hesai_ros_driver:msg/Ptp.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__PTP__BUILDER_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__PTP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hesai_ros_driver/msg/detail/ptp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hesai_ros_driver
{

namespace msg
{

namespace builder
{

class Init_Ptp_ptp_status
{
public:
  explicit Init_Ptp_ptp_status(::hesai_ros_driver::msg::Ptp & msg)
  : msg_(msg)
  {}
  ::hesai_ros_driver::msg::Ptp ptp_status(::hesai_ros_driver::msg::Ptp::_ptp_status_type arg)
  {
    msg_.ptp_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hesai_ros_driver::msg::Ptp msg_;
};

class Init_Ptp_ptp_lock_offset
{
public:
  Init_Ptp_ptp_lock_offset()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ptp_ptp_status ptp_lock_offset(::hesai_ros_driver::msg::Ptp::_ptp_lock_offset_type arg)
  {
    msg_.ptp_lock_offset = std::move(arg);
    return Init_Ptp_ptp_status(msg_);
  }

private:
  ::hesai_ros_driver::msg::Ptp msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hesai_ros_driver::msg::Ptp>()
{
  return hesai_ros_driver::msg::builder::Init_Ptp_ptp_lock_offset();
}

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__PTP__BUILDER_HPP_
