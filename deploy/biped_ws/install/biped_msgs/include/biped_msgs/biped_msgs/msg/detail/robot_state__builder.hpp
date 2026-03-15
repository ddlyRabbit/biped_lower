// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from biped_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/robot_state.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "biped_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace biped_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_active_faults
{
public:
  explicit Init_RobotState_active_faults(::biped_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::biped_msgs::msg::RobotState active_faults(::biped_msgs::msg::RobotState::_active_faults_type arg)
  {
    msg_.active_faults = std::move(arg);
    return std::move(msg_);
  }

private:
  ::biped_msgs::msg::RobotState msg_;
};

class Init_RobotState_battery_voltage
{
public:
  explicit Init_RobotState_battery_voltage(::biped_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_active_faults battery_voltage(::biped_msgs::msg::RobotState::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_RobotState_active_faults(msg_);
  }

private:
  ::biped_msgs::msg::RobotState msg_;
};

class Init_RobotState_all_motors_enabled
{
public:
  explicit Init_RobotState_all_motors_enabled(::biped_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_battery_voltage all_motors_enabled(::biped_msgs::msg::RobotState::_all_motors_enabled_type arg)
  {
    msg_.all_motors_enabled = std::move(arg);
    return Init_RobotState_battery_voltage(msg_);
  }

private:
  ::biped_msgs::msg::RobotState msg_;
};

class Init_RobotState_safety_ok
{
public:
  explicit Init_RobotState_safety_ok(::biped_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_all_motors_enabled safety_ok(::biped_msgs::msg::RobotState::_safety_ok_type arg)
  {
    msg_.safety_ok = std::move(arg);
    return Init_RobotState_all_motors_enabled(msg_);
  }

private:
  ::biped_msgs::msg::RobotState msg_;
};

class Init_RobotState_fsm_state
{
public:
  explicit Init_RobotState_fsm_state(::biped_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_safety_ok fsm_state(::biped_msgs::msg::RobotState::_fsm_state_type arg)
  {
    msg_.fsm_state = std::move(arg);
    return Init_RobotState_safety_ok(msg_);
  }

private:
  ::biped_msgs::msg::RobotState msg_;
};

class Init_RobotState_header
{
public:
  Init_RobotState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_fsm_state header(::biped_msgs::msg::RobotState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotState_fsm_state(msg_);
  }

private:
  ::biped_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::biped_msgs::msg::RobotState>()
{
  return biped_msgs::msg::builder::Init_RobotState_header();
}

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
