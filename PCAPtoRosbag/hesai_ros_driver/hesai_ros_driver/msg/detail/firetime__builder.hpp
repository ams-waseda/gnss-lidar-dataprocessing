// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hesai_ros_driver:msg/Firetime.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__FIRETIME__BUILDER_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__FIRETIME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hesai_ros_driver/msg/detail/firetime__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hesai_ros_driver
{

namespace msg
{

namespace builder
{

class Init_Firetime_data
{
public:
  Init_Firetime_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hesai_ros_driver::msg::Firetime data(::hesai_ros_driver::msg::Firetime::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hesai_ros_driver::msg::Firetime msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hesai_ros_driver::msg::Firetime>()
{
  return hesai_ros_driver::msg::builder::Init_Firetime_data();
}

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__FIRETIME__BUILDER_HPP_
