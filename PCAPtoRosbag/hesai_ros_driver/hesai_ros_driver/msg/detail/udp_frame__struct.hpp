// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hesai_ros_driver:msg/UdpFrame.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__STRUCT_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'packets'
#include "hesai_ros_driver/msg/detail/udp_packet__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hesai_ros_driver__msg__UdpFrame __attribute__((deprecated))
#else
# define DEPRECATED__hesai_ros_driver__msg__UdpFrame __declspec(deprecated)
#endif

namespace hesai_ros_driver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UdpFrame_
{
  using Type = UdpFrame_<ContainerAllocator>;

  explicit UdpFrame_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit UdpFrame_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _packets_type =
    std::vector<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>>>;
  _packets_type packets;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__packets(
    const std::vector<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hesai_ros_driver::msg::UdpPacket_<ContainerAllocator>>> & _arg)
  {
    this->packets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hesai_ros_driver::msg::UdpFrame_<ContainerAllocator> *;
  using ConstRawPtr =
    const hesai_ros_driver::msg::UdpFrame_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::UdpFrame_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::UdpFrame_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hesai_ros_driver__msg__UdpFrame
    std::shared_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hesai_ros_driver__msg__UdpFrame
    std::shared_ptr<hesai_ros_driver::msg::UdpFrame_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UdpFrame_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->packets != other.packets) {
      return false;
    }
    return true;
  }
  bool operator!=(const UdpFrame_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UdpFrame_

// alias to use template instance with default allocator
using UdpFrame =
  hesai_ros_driver::msg::UdpFrame_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__UDP_FRAME__STRUCT_HPP_
