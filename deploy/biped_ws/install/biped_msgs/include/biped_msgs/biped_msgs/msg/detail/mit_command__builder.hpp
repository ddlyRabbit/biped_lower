// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from biped_msgs:msg/MITCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/mit_command.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__BUILDER_HPP_
#define BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "biped_msgs/msg/detail/mit_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace biped_msgs
{

namespace msg
{

namespace builder
{

class Init_MITCommand_torque_ff
{
public:
  explicit Init_MITCommand_torque_ff(::biped_msgs::msg::MITCommand & msg)
  : msg_(msg)
  {}
  ::biped_msgs::msg::MITCommand torque_ff(::biped_msgs::msg::MITCommand::_torque_ff_type arg)
  {
    msg_.torque_ff = std::move(arg);
    return std::move(msg_);
  }

private:
  ::biped_msgs::msg::MITCommand msg_;
};

class Init_MITCommand_kd
{
public:
  explicit Init_MITCommand_kd(::biped_msgs::msg::MITCommand & msg)
  : msg_(msg)
  {}
  Init_MITCommand_torque_ff kd(::biped_msgs::msg::MITCommand::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_MITCommand_torque_ff(msg_);
  }

private:
  ::biped_msgs::msg::MITCommand msg_;
};

class Init_MITCommand_kp
{
public:
  explicit Init_MITCommand_kp(::biped_msgs::msg::MITCommand & msg)
  : msg_(msg)
  {}
  Init_MITCommand_kd kp(::biped_msgs::msg::MITCommand::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MITCommand_kd(msg_);
  }

private:
  ::biped_msgs::msg::MITCommand msg_;
};

class Init_MITCommand_velocity
{
public:
  explicit Init_MITCommand_velocity(::biped_msgs::msg::MITCommand & msg)
  : msg_(msg)
  {}
  Init_MITCommand_kp velocity(::biped_msgs::msg::MITCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MITCommand_kp(msg_);
  }

private:
  ::biped_msgs::msg::MITCommand msg_;
};

class Init_MITCommand_position
{
public:
  explicit Init_MITCommand_position(::biped_msgs::msg::MITCommand & msg)
  : msg_(msg)
  {}
  Init_MITCommand_velocity position(::biped_msgs::msg::MITCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MITCommand_velocity(msg_);
  }

private:
  ::biped_msgs::msg::MITCommand msg_;
};

class Init_MITCommand_joint_name
{
public:
  Init_MITCommand_joint_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MITCommand_position joint_name(::biped_msgs::msg::MITCommand::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_MITCommand_position(msg_);
  }

private:
  ::biped_msgs::msg::MITCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::biped_msgs::msg::MITCommand>()
{
  return biped_msgs::msg::builder::Init_MITCommand_joint_name();
}

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__BUILDER_HPP_
