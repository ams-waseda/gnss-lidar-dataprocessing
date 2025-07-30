// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hesai_ros_driver:msg/UdpFrame.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__STRUCT_H_
#define HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'packets'
#include "hesai_ros_driver/msg/detail/udp_packet__struct.h"

/// Struct defined in msg/UdpFrame in the package hesai_ros_driver.
typedef struct hesai_ros_driver__msg__UdpFrame
{
  std_msgs__msg__Header header;
  hesai_ros_driver__msg__UdpPacket__Sequence packets;
} hesai_ros_driver__msg__UdpFrame;

// Struct for a sequence of hesai_ros_driver__msg__UdpFrame.
typedef struct hesai_ros_driver__msg__UdpFrame__Sequence
{
  hesai_ros_driver__msg__UdpFrame * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hesai_ros_driver__msg__UdpFrame__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__STRUCT_H_
