// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hesai_ros_driver:msg/LossPacket.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__TRAITS_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hesai_ros_driver/msg/detail/loss_packet__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hesai_ros_driver
{

namespace msg
{

inline void to_flow_style_yaml(
  const LossPacket & msg,
  std::ostream & out)
{
  out << "{";
  // member: total_packet_count
  {
    out << "total_packet_count: ";
    rosidl_generator_traits::value_to_yaml(msg.total_packet_count, out);
    out << ", ";
  }

  // member: total_packet_loss_count
  {
    out << "total_packet_loss_count: ";
    rosidl_generator_traits::value_to_yaml(msg.total_packet_loss_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LossPacket & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: total_packet_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_packet_count: ";
    rosidl_generator_traits::value_to_yaml(msg.total_packet_count, out);
    out << "\n";
  }

  // member: total_packet_loss_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_packet_loss_count: ";
    rosidl_generator_traits::value_to_yaml(msg.total_packet_loss_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LossPacket & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hesai_ros_driver

namespace rosidl_generator_traits
{

[[deprecated("use hesai_ros_driver::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hesai_ros_driver::msg::LossPacket & msg,
  std::ostream & out, size_t indentation = 0)
{
  hesai_ros_driver::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hesai_ros_driver::msg::to_yaml() instead")]]
inline std::string to_yaml(const hesai_ros_driver::msg::LossPacket & msg)
{
  return hesai_ros_driver::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hesai_ros_driver::msg::LossPacket>()
{
  return "hesai_ros_driver::msg::LossPacket";
}

template<>
inline const char * name<hesai_ros_driver::msg::LossPacket>()
{
  return "hesai_ros_driver/msg/LossPacket";
}

template<>
struct has_fixed_size<hesai_ros_driver::msg::LossPacket>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hesai_ros_driver::msg::LossPacket>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hesai_ros_driver::msg::LossPacket>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__TRAITS_HPP_
