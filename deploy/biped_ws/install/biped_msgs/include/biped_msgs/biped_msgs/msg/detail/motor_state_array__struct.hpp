// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from biped_msgs:msg/MotorStateArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/motor_state_array.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__STRUCT_HPP_
#define BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'motors'
#include "biped_msgs/msg/detail/motor_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__biped_msgs__msg__MotorStateArray __attribute__((deprecated))
#else
# define DEPRECATED__biped_msgs__msg__MotorStateArray __declspec(deprecated)
#endif

namespace biped_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorStateArray_
{
  using Type = MotorStateArray_<ContainerAllocator>;

  explicit MotorStateArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MotorStateArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motors_type =
    std::vector<biped_msgs::msg::MotorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<biped_msgs::msg::MotorState_<ContainerAllocator>>>;
  _motors_type motors;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motors(
    const std::vector<biped_msgs::msg::MotorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<biped_msgs::msg::MotorState_<ContainerAllocator>>> & _arg)
  {
    this->motors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    biped_msgs::msg::MotorStateArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const biped_msgs::msg::MotorStateArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MotorStateArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MotorStateArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__biped_msgs__msg__MotorStateArray
    std::shared_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__biped_msgs__msg__MotorStateArray
    std::shared_ptr<biped_msgs::msg::MotorStateArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorStateArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motors != other.motors) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorStateArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorStateArray_

// alias to use template instance with default allocator
using MotorStateArray =
  biped_msgs::msg::MotorStateArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__STRUCT_HPP_
