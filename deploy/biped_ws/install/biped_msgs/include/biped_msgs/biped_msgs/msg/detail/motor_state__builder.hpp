// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from biped_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/motor_state.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "biped_msgs/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace biped_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorState_mode_status
{
public:
  explicit Init_MotorState_mode_status(::biped_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::biped_msgs::msg::MotorState mode_status(::biped_msgs::msg::MotorState::_mode_status_type arg)
  {
    msg_.mode_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

class Init_MotorState_fault_code
{
public:
  explicit Init_MotorState_fault_code(::biped_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_mode_status fault_code(::biped_msgs::msg::MotorState::_fault_code_type arg)
  {
    msg_.fault_code = std::move(arg);
    return Init_MotorState_mode_status(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

class Init_MotorState_temperature
{
public:
  explicit Init_MotorState_temperature(::biped_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_fault_code temperature(::biped_msgs::msg::MotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_MotorState_fault_code(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

class Init_MotorState_torque
{
public:
  explicit Init_MotorState_torque(::biped_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_temperature torque(::biped_msgs::msg::MotorState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_MotorState_temperature(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

class Init_MotorState_velocity
{
public:
  explicit Init_MotorState_velocity(::biped_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_torque velocity(::biped_msgs::msg::MotorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorState_torque(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

class Init_MotorState_position
{
public:
  explicit Init_MotorState_position(::biped_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_velocity position(::biped_msgs::msg::MotorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorState_velocity(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

class Init_MotorState_can_id
{
public:
  explicit Init_MotorState_can_id(::biped_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_position can_id(::biped_msgs::msg::MotorState::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_MotorState_position(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

class Init_MotorState_joint_name
{
public:
  Init_MotorState_joint_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_can_id joint_name(::biped_msgs::msg::MotorState::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_MotorState_can_id(msg_);
  }

private:
  ::biped_msgs::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::biped_msgs::msg::MotorState>()
{
  return biped_msgs::msg::builder::Init_MotorState_joint_name();
}

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
