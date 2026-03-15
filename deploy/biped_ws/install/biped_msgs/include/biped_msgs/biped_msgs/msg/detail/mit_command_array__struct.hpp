// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from biped_msgs:msg/MITCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/mit_command_array.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__STRUCT_HPP_
#define BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__STRUCT_HPP_

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
// Member 'commands'
#include "biped_msgs/msg/detail/mit_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__biped_msgs__msg__MITCommandArray __attribute__((deprecated))
#else
# define DEPRECATED__biped_msgs__msg__MITCommandArray __declspec(deprecated)
#endif

namespace biped_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MITCommandArray_
{
  using Type = MITCommandArray_<ContainerAllocator>;

  explicit MITCommandArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MITCommandArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _commands_type =
    std::vector<biped_msgs::msg::MITCommand_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<biped_msgs::msg::MITCommand_<ContainerAllocator>>>;
  _commands_type commands;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__commands(
    const std::vector<biped_msgs::msg::MITCommand_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<biped_msgs::msg::MITCommand_<ContainerAllocator>>> & _arg)
  {
    this->commands = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    biped_msgs::msg::MITCommandArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const biped_msgs::msg::MITCommandArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MITCommandArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MITCommandArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__biped_msgs__msg__MITCommandArray
    std::shared_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__biped_msgs__msg__MITCommandArray
    std::shared_ptr<biped_msgs::msg::MITCommandArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MITCommandArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->commands != other.commands) {
      return false;
    }
    return true;
  }
  bool operator!=(const MITCommandArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MITCommandArray_

// alias to use template instance with default allocator
using MITCommandArray =
  biped_msgs::msg::MITCommandArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__STRUCT_HPP_
