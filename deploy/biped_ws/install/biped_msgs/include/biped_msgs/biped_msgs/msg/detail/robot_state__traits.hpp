// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from biped_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/robot_state.hpp"


#ifndef BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "biped_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace biped_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: fsm_state
  {
    out << "fsm_state: ";
    rosidl_generator_traits::value_to_yaml(msg.fsm_state, out);
    out << ", ";
  }

  // member: safety_ok
  {
    out << "safety_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_ok, out);
    out << ", ";
  }

  // member: all_motors_enabled
  {
    out << "all_motors_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.all_motors_enabled, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: active_faults
  {
    if (msg.active_faults.size() == 0) {
      out << "active_faults: []";
    } else {
      out << "active_faults: [";
      size_t pending_items = msg.active_faults.size();
      for (auto item : msg.active_faults) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: fsm_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fsm_state: ";
    rosidl_generator_traits::value_to_yaml(msg.fsm_state, out);
    out << "\n";
  }

  // member: safety_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_ok, out);
    out << "\n";
  }

  // member: all_motors_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "all_motors_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.all_motors_enabled, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: active_faults
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_faults.size() == 0) {
      out << "active_faults: []\n";
    } else {
      out << "active_faults:\n";
      for (auto item : msg.active_faults) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
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
  const biped_msgs::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  biped_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use biped_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const biped_msgs::msg::RobotState & msg)
{
  return biped_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<biped_msgs::msg::RobotState>()
{
  return "biped_msgs::msg::RobotState";
}

template<>
inline const char * name<biped_msgs::msg::RobotState>()
{
  return "biped_msgs/msg/RobotState";
}

template<>
struct has_fixed_size<biped_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<biped_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<biped_msgs::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
