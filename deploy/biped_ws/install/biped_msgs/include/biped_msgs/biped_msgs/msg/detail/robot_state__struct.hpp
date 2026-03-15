// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from biped_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/robot_state.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
#define BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__biped_msgs__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__biped_msgs__msg__RobotState __declspec(deprecated)
#endif

namespace biped_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fsm_state = "";
      this->safety_ok = false;
      this->all_motors_enabled = false;
      this->battery_voltage = 0.0f;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    fsm_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fsm_state = "";
      this->safety_ok = false;
      this->all_motors_enabled = false;
      this->battery_voltage = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fsm_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _fsm_state_type fsm_state;
  using _safety_ok_type =
    bool;
  _safety_ok_type safety_ok;
  using _all_motors_enabled_type =
    bool;
  _all_motors_enabled_type all_motors_enabled;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _active_faults_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _active_faults_type active_faults;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fsm_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->fsm_state = _arg;
    return *this;
  }
  Type & set__safety_ok(
    const bool & _arg)
  {
    this->safety_ok = _arg;
    return *this;
  }
  Type & set__all_motors_enabled(
    const bool & _arg)
  {
    this->all_motors_enabled = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__active_faults(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->active_faults = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    biped_msgs::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const biped_msgs::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<biped_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<biped_msgs::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      biped_msgs::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<biped_msgs::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<biped_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<biped_msgs::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__biped_msgs__msg__RobotState
    std::shared_ptr<biped_msgs::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__biped_msgs__msg__RobotState
    std::shared_ptr<biped_msgs::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fsm_state != other.fsm_state) {
      return false;
    }
    if (this->safety_ok != other.safety_ok) {
      return false;
    }
    if (this->all_motors_enabled != other.all_motors_enabled) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->active_faults != other.active_faults) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  biped_msgs::msg::RobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
