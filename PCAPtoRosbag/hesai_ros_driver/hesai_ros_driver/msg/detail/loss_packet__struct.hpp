// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hesai_ros_driver:msg/LossPacket.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__STRUCT_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hesai_ros_driver__msg__LossPacket __attribute__((deprecated))
#else
# define DEPRECATED__hesai_ros_driver__msg__LossPacket __declspec(deprecated)
#endif

namespace hesai_ros_driver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LossPacket_
{
  using Type = LossPacket_<ContainerAllocator>;

  explicit LossPacket_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_packet_count = 0ul;
      this->total_packet_loss_count = 0ul;
    }
  }

  explicit LossPacket_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_packet_count = 0ul;
      this->total_packet_loss_count = 0ul;
    }
  }

  // field types and members
  using _total_packet_count_type =
    uint32_t;
  _total_packet_count_type total_packet_count;
  using _total_packet_loss_count_type =
    uint32_t;
  _total_packet_loss_count_type total_packet_loss_count;

  // setters for named parameter idiom
  Type & set__total_packet_count(
    const uint32_t & _arg)
  {
    this->total_packet_count = _arg;
    return *this;
  }
  Type & set__total_packet_loss_count(
    const uint32_t & _arg)
  {
    this->total_packet_loss_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hesai_ros_driver::msg::LossPacket_<ContainerAllocator> *;
  using ConstRawPtr =
    const hesai_ros_driver::msg::LossPacket_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::LossPacket_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::LossPacket_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hesai_ros_driver__msg__LossPacket
    std::shared_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hesai_ros_driver__msg__LossPacket
    std::shared_ptr<hesai_ros_driver::msg::LossPacket_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LossPacket_ & other) const
  {
    if (this->total_packet_count != other.total_packet_count) {
      return false;
    }
    if (this->total_packet_loss_count != other.total_packet_loss_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const LossPacket_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LossPacket_

// alias to use template instance with default allocator
using LossPacket =
  hesai_ros_driver::msg::LossPacket_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__LOSS_PACKET__STRUCT_HPP_
