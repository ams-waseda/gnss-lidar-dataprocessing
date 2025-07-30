// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hesai_ros_driver:msg/UdpFrame.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__TRAITS_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hesai_ros_driver/msg/detail/udp_frame__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'packets'
#include "hesai_ros_driver/msg/detail/udp_packet__traits.hpp"

namespace hesai_ros_driver
{

namespace msg
{

inline void to_flow_style_yaml(
  const UdpFrame & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: packets
  {
    if (msg.packets.size() == 0) {
      out << "packets: []";
    } else {
      out << "packets: [";
      size_t pending_items = msg.packets.size();
      for (auto item : msg.packets) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UdpFrame & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: packets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.packets.size() == 0) {
      out << "packets: []\n";
    } else {
      out << "packets:\n";
      for (auto item : msg.packets) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UdpFrame & msg, bool use_flow_style = false)
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
  const hesai_ros_driver::msg::UdpFrame & msg,
  std::ostream & out, size_t indentation = 0)
{
  hesai_ros_driver::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hesai_ros_driver::msg::to_yaml() instead")]]
inline std::string to_yaml(const hesai_ros_driver::msg::UdpFrame & msg)
{
  return hesai_ros_driver::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hesai_ros_driver::msg::UdpFrame>()
{
  return "hesai_ros_driver::msg::UdpFrame";
}

template<>
inline const char * name<hesai_ros_driver::msg::UdpFrame>()
{
  return "hesai_ros_driver/msg/UdpFrame";
}

template<>
struct has_fixed_size<hesai_ros_driver::msg::UdpFrame>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hesai_ros_driver::msg::UdpFrame>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hesai_ros_driver::msg::UdpFrame>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__TRAITS_HPP_
