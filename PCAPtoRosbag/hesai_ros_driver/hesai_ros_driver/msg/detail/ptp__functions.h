// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hesai_ros_driver:msg/Ptp.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__PTP__FUNCTIONS_H_
#define HESAI_ROS_DRIVER__MSG__DETAIL__PTP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hesai_ros_driver/msg/rosidl_generator_c__visibility_control.h"

#include "hesai_ros_driver/msg/detail/ptp__struct.h"

/// Initialize msg/Ptp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hesai_ros_driver__msg__Ptp
 * )) before or use
 * hesai_ros_driver__msg__Ptp__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
bool
hesai_ros_driver__msg__Ptp__init(hesai_ros_driver__msg__Ptp * msg);

/// Finalize msg/Ptp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
void
hesai_ros_driver__msg__Ptp__fini(hesai_ros_driver__msg__Ptp * msg);

/// Create msg/Ptp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hesai_ros_driver__msg__Ptp__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
hesai_ros_driver__msg__Ptp *
hesai_ros_driver__msg__Ptp__create();

/// Destroy msg/Ptp message.
/**
 * It calls
 * hesai_ros_driver__msg__Ptp__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
void
hesai_ros_driver__msg__Ptp__destroy(hesai_ros_driver__msg__Ptp * msg);

/// Check for msg/Ptp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
bool
hesai_ros_driver__msg__Ptp__are_equal(const hesai_ros_driver__msg__Ptp * lhs, const hesai_ros_driver__msg__Ptp * rhs);

/// Copy a msg/Ptp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
bool
hesai_ros_driver__msg__Ptp__copy(
  const hesai_ros_driver__msg__Ptp * input,
  hesai_ros_driver__msg__Ptp * output);

/// Initialize array of msg/Ptp messages.
/**
 * It allocates the memory for the number of elements and calls
 * hesai_ros_driver__msg__Ptp__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
bool
hesai_ros_driver__msg__Ptp__Sequence__init(hesai_ros_driver__msg__Ptp__Sequence * array, size_t size);

/// Finalize array of msg/Ptp messages.
/**
 * It calls
 * hesai_ros_driver__msg__Ptp__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
void
hesai_ros_driver__msg__Ptp__Sequence__fini(hesai_ros_driver__msg__Ptp__Sequence * array);

/// Create array of msg/Ptp messages.
/**
 * It allocates the memory for the array and calls
 * hesai_ros_driver__msg__Ptp__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
hesai_ros_driver__msg__Ptp__Sequence *
hesai_ros_driver__msg__Ptp__Sequence__create(size_t size);

/// Destroy array of msg/Ptp messages.
/**
 * It calls
 * hesai_ros_driver__msg__Ptp__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
void
hesai_ros_driver__msg__Ptp__Sequence__destroy(hesai_ros_driver__msg__Ptp__Sequence * array);

/// Check for msg/Ptp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
bool
hesai_ros_driver__msg__Ptp__Sequence__are_equal(const hesai_ros_driver__msg__Ptp__Sequence * lhs, const hesai_ros_driver__msg__Ptp__Sequence * rhs);

/// Copy an array of msg/Ptp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hesai_ros_driver
bool
hesai_ros_driver__msg__Ptp__Sequence__copy(
  const hesai_ros_driver__msg__Ptp__Sequence * input,
  hesai_ros_driver__msg__Ptp__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__PTP__FUNCTIONS_H_
