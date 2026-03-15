// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from biped_msgs:msg/MITCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/mit_command.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__TRAITS_HPP_
#define BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "biped_msgs/msg/detail/mit_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace biped_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MITCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_name
  {
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: kp
  {
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << ", ";
  }

  // member: kd
  {
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << ", ";
  }

  // member: torque_ff
  {
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MITCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << "\n";
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << "\n";
  }

  // member: torque_ff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MITCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace biped_msgs

namespace rosidl_generator_traits
{

[[deprecated("use biped_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const biped_msgs::msg::MITCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  biped_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use biped_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const biped_msgs::msg::MITCommand & msg)
{
  return biped_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<biped_msgs::msg::MITCommand>()
{
  return "biped_msgs::msg::MITCommand";
}

template<>
inline const char * name<biped_msgs::msg::MITCommand>()
{
  return "biped_msgs/msg/MITCommand";
}

template<>
struct has_fixed_size<biped_msgs::msg::MITCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<biped_msgs::msg::MITCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<biped_msgs::msg::MITCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BIPED_MSGS__MSG__DETAIL__MIT_COMMAND__TRAITS_HPP_
