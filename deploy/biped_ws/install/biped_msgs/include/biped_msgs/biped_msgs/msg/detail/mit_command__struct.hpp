// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from biped_msgs:msg/MITCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/mit_command.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__STRUCT_HPP_
#define BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__biped_msgs__msg__MITCommand __attribute__((deprecated))
#else
# define DEPRECATED__biped_msgs__msg__MITCommand __declspec(deprecated)
#endif

namespace biped_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MITCommand_
{
  using Type = MITCommand_<ContainerAllocator>;

  explicit MITCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->torque_ff = 0.0f;
    }
  }

  explicit MITCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->torque_ff = 0.0f;
    }
  }

  // field types and members
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _position_type =
    float;
  _position_type position;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _kp_type =
    float;
  _kp_type kp;
  using _kd_type =
    float;
  _kd_type kd;
  using _torque_ff_type =
    float;
  _torque_ff_type torque_ff;

  // setters for named parameter idiom
  Type & set__joint_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_name = _arg;
    return *this;
  }
  Type & set__position(
    const float & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__kp(
    const float & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const float & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__torque_ff(
    const float & _arg)
  {
    this->torque_ff = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    biped_msgs::msg::MITCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const biped_msgs::msg::MITCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MITCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MITCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__biped_msgs__msg__MITCommand
    std::shared_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__biped_msgs__msg__MITCommand
    std::shared_ptr<biped_msgs::msg::MITCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MITCommand_ & other) const
  {
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->torque_ff != other.torque_ff) {
      return false;
    }
    return true;
  }
  bool operator!=(const MITCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MITCommand_

// alias to use template instance with default allocator
using MITCommand =
  biped_msgs::msg::MITCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__STRUCT_HPP_
