// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from biped_msgs:msg/MITCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/mit_command_array.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__BUILDER_HPP_
#define BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "biped_msgs/msg/detail/mit_command_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace biped_msgs
{

namespace msg
{

namespace builder
{

class Init_MITCommandArray_commands
{
public:
  explicit Init_MITCommandArray_commands(::biped_msgs::msg::MITCommandArray & msg)
  : msg_(msg)
  {}
  ::biped_msgs::msg::MITCommandArray commands(::biped_msgs::msg::MITCommandArray::_commands_type arg)
  {
    msg_.commands = std::move(arg);
    return std::move(msg_);
  }

private:
  ::biped_msgs::msg::MITCommandArray msg_;
};

class Init_MITCommandArray_header
{
public:
  Init_MITCommandArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MITCommandArray_commands header(::biped_msgs::msg::MITCommandArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MITCommandArray_commands(msg_);
  }

private:
  ::biped_msgs::msg::MITCommandArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::biped_msgs::msg::MITCommandArray>()
{
  return biped_msgs::msg::builder::Init_MITCommandArray_header();
}

}  // namespace biped_msgs

#endif  // BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__BUILDER_HPP_
