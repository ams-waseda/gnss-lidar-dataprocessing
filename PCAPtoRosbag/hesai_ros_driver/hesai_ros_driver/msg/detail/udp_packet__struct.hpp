// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hesai_ros_driver:msg/UdpPacket.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__UDP_PACKET__STRUCT_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__UDP_PACKET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hesai_ros_driver__msg__UdpPacket __attribute__((deprecated))
#else
# define DEPRECATED__hesai_ros_driver__msg__UdpPacket __declspec(deprecated)
#endif

namespace hesai_ros_driver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UdpPacket_
{
  using Type = UdpPacket_<ContainerAllocator>;

  explicit UdpPacket_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->size = 0ul;
    }
  }

  explicit UdpPacket_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->size = 0ul;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;
  using _size_type =
    uint32_t;
  _size_type size;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__size(
    const uint32_t & _arg)
  {
    this->size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hesai_ros_driver::msg::UdpPacket_<ContainerAllocator> *;
  using ConstRawPtr =
    const hesai_ros_driver::msg::UdpPacket_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hesai_ros_driver__msg__UdpPacket
    std::shared_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hesai_ros_driver__msg__UdpPacket
    std::shared_ptr<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UdpPacket_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    return true;
  }
  bool operator!=(const UdpPacket_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UdpPacket_

// alias to use template instance with default allocator
using UdpPacket =
  hesai_ros_driver::msg::UdpPacket_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__UDP_PACKET__STRUCT_HPP_
