// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from biped_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/motor_state.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
#define BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__biped_msgs__msg__MotorState __attribute__((deprecated))
#else
# define DEPRECATED__biped_msgs__msg__MotorState __declspec(deprecated)
#endif

namespace biped_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorState_
{
  using Type = MotorState_<ContainerAllocator>;

  explicit MotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->can_id = 0;
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->torque = 0.0f;
      this->temperature = 0.0f;
      this->fault_code = 0;
      this->mode_status = 0;
    }
  }

  explicit MotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->can_id = 0;
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->torque = 0.0f;
      this->temperature = 0.0f;
      this->fault_code = 0;
      this->mode_status = 0;
    }
  }

  // field types and members
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _can_id_type =
    uint8_t;
  _can_id_type can_id;
  using _position_type =
    float;
  _position_type position;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _torque_type =
    float;
  _torque_type torque;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _fault_code_type =
    uint8_t;
  _fault_code_type fault_code;
  using _mode_status_type =
    uint8_t;
  _mode_status_type mode_status;

  // setters for named parameter idiom
  Type & set__joint_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_name = _arg;
    return *this;
  }
  Type & set__can_id(
    const uint8_t & _arg)
  {
    this->can_id = _arg;
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
  Type & set__torque(
    const float & _arg)
  {
    this->torque = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__fault_code(
    const uint8_t & _arg)
  {
    this->fault_code = _arg;
    return *this;
  }
  Type & set__mode_status(
    const uint8_t & _arg)
  {
    this->mode_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    biped_msgs::msg::MotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const biped_msgs::msg::MotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<biped_msgs::msg::MotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<biped_msgs::msg::MotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::MotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::MotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<biped_msgs::msg::MotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<biped_msgs::msg::MotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__biped_msgs__msg__MotorState
    std::shared_ptr<biped_msgs::msg::MotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__biped_msgs__msg__MotorState
    std::shared_ptr<biped_msgs::msg::MotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorState_ & other) const
  {
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->can_id != other.can_id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->fault_code != other.fault_code) {
      return false;
    }
    if (this->mode_status != other.mode_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorState_

// alias to use template instance with default allocator
using MotorState =
  biped_msgs::msg::MotorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
