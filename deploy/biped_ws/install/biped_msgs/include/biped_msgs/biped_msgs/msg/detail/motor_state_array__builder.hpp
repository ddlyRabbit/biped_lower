// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from biped_msgs:msg/MotorStateArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/motor_state_array.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__BUILDER_HPP_
#define BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "biped_msgs/msg/detail/motor_state_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace biped_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorStateArray_motors
{
public:
  explicit Init_MotorStateArray_motors(::biped_msgs::msg::MotorStateArray & msg)
  : msg_(msg)
  {}
  ::biped_msgs::msg::MotorStateArray motors(::biped_msgs::msg::MotorStateArray::_motors_type arg)
  {
    msg_.motors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::biped_msgs::msg::MotorStateArray msg_;
};

class Init_MotorStateArray_header
{
public:
  Init_MotorStateArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorStateArray_motors header(::biped_msgs::msg::MotorStateArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorStateArray_motors(msg_);
  }

private:
  ::biped_msgs::msg::MotorStateArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::biped_msgs::msg::MotorStateArray>()
{
  return biped_msgs::msg::builder::Init_MotorStateArray_header();
}

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__BUILDER_HPP_
