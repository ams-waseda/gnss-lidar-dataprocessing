// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hesai_ros_driver:msg/LossPacket.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__STRUCT_H_
#define HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LossPacket in the package hesai_ros_driver.
typedef struct hesai_ros_driver__msg__LossPacket
{
  uint32_t total_packet_count;
  uint32_t total_packet_loss_count;
} hesai_ros_driver__msg__LossPacket;

// Struct for a sequence of hesai_ros_driver__msg__LossPacket.
typedef struct hesai_ros_driver__msg__LossPacket__Sequence
{
  hesai_ros_driver__msg__LossPacket * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hesai_ros_driver__msg__LossPacket__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__STRUCT_H_
