// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hesai_ros_driver:msg/Ptp.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__PTP__STRUCT_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__PTP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hesai_ros_driver__msg__Ptp __attribute__((deprecated))
#else
# define DEPRECATED__hesai_ros_driver__msg__Ptp __declspec(deprecated)
#endif

namespace hesai_ros_driver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Ptp_
{
  using Type = Ptp_<ContainerAllocator>;

  explicit Ptp_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ptp_lock_offset = 0;
      std::fill<typename std::array<uint8_t, 16>::iterator, uint8_t>(this->ptp_status.begin(), this->ptp_status.end(), 0);
    }
  }

  explicit Ptp_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ptp_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ptp_lock_offset = 0;
      std::fill<typename std::array<uint8_t, 16>::iterator, uint8_t>(this->ptp_status.begin(), this->ptp_status.end(), 0);
    }
  }

  // field types and members
  using _ptp_lock_offset_type =
    uint8_t;
  _ptp_lock_offset_type ptp_lock_offset;
  using _ptp_status_type =
    std::array<uint8_t, 16>;
  _ptp_status_type ptp_status;

  // setters for named parameter idiom
  Type & set__ptp_lock_offset(
    const uint8_t & _arg)
  {
    this->ptp_lock_offset = _arg;
    return *this;
  }
  Type & set__ptp_status(
    const std::array<uint8_t, 16> & _arg)
  {
    this->ptp_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hesai_ros_driver::msg::Ptp_<ContainerAllocator> *;
  using ConstRawPtr =
    const hesai_ros_driver::msg::Ptp_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::Ptp_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::Ptp_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hesai_ros_driver__msg__Ptp
    std::shared_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hesai_ros_driver__msg__Ptp
    std::shared_ptr<hesai_ros_driver::msg::Ptp_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Ptp_ & other) const
  {
    if (this->ptp_lock_offset != other.ptp_lock_offset) {
      return false;
    }
    if (this->ptp_status != other.ptp_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Ptp_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Ptp_

// alias to use template instance with default allocator
using Ptp =
  hesai_ros_driver::msg::Ptp_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__PTP__STRUCT_HPP_
