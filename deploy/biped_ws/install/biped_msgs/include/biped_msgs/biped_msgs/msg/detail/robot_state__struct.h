// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from biped_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/robot_state.h"


#ifndef BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'fsm_state'
// Member 'active_faults'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotState in the package biped_msgs.
typedef struct biped_msgs__msg__RobotState
{
  std_msgs__msg__Header header;
  /// IDLE, CALIBRATE, STAND, WALK, ESTOP
  rosidl_runtime_c__String fsm_state;
  bool safety_ok;
  bool all_motors_enabled;
  float battery_voltage;
  rosidl_runtime_c__String__Sequence active_faults;
} biped_msgs__msg__RobotState;

// Struct for a sequence of biped_msgs__msg__RobotState.
typedef struct biped_msgs__msg__RobotState__Sequence
{
  biped_msgs__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} biped_msgs__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BIPED_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
