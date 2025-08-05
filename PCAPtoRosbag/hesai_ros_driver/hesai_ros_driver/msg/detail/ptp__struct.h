// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hesai_ros_driver:msg/Ptp.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__PTP__STRUCT_H_
#define HESAI_ROS_DRIVER__MSG__DETAIL__PTP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Ptp in the package hesai_ros_driver.
typedef struct hesai_ros_driver__msg__Ptp
{
  uint8_t ptp_lock_offset;
  uint8_t ptp_status[16];
} hesai_ros_driver__msg__Ptp;

// Struct for a sequence of hesai_ros_driver__msg__Ptp.
typedef struct hesai_ros_driver__msg__Ptp__Sequence
{
  hesai_ros_driver__msg__Ptp * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hesai_ros_driver__msg__Ptp__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__PTP__STRUCT_H_
