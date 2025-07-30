// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hesai_ros_driver:msg/Firetime.idl
// generated code does not contain a copyright notice

#ifndef HESAI_ROS_DRIVER__MSG__DETAIL__FIRETIME__STRUCT_HPP_
#define HESAI_ROS_DRIVER__MSG__DETAIL__FIRETIME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hesai_ros_driver__msg__Firetime __attribute__((deprecated))
#else
# define DEPRECATED__hesai_ros_driver__msg__Firetime __declspec(deprecated)
#endif

namespace hesai_ros_driver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Firetime_
{
  using Type = Firetime_<ContainerAllocator>;

  explicit Firetime_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 512>::iterator, double>(this->data.begin(), this->data.end(), 0.0);
    }
  }

  explicit Firetime_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 512>::iterator, double>(this->data.begin(), this->data.end(), 0.0);
    }
  }

  // field types and members
  using _data_type =
    std::array<double, 512>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::array<double, 512> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hesai_ros_driver::msg::Firetime_<ContainerAllocator> *;
  using ConstRawPtr =
    const hesai_ros_driver::msg::Firetime_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::Firetime_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hesai_ros_driver::msg::Firetime_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hesai_ros_driver__msg__Firetime
    std::shared_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hesai_ros_driver__msg__Firetime
    std::shared_ptr<hesai_ros_driver::msg::Firetime_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Firetime_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Firetime_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Firetime_

// alias to use template instance with default allocator
using Firetime =
  hesai_ros_driver::msg::Firetime_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hesai_ros_driver

#endif  // HESAI_ROS_DRIVER__MSG__DETAIL__FIRETIME__STRUCT_HPP_
