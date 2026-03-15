// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from biped_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/motor_state.h"


#ifndef BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'joint_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorState in the package biped_msgs.
typedef struct biped_msgs__msg__MotorState
{
  rosidl_runtime_c__String joint_name;
  uint8_t can_id;
  /// absolute encoder position (rad, calibrated)
  float position;
  /// current velocity (rad/s)
  float velocity;
  /// current torque (Nm)
  float torque;
  /// motor temp (°C)
  float temperature;
  /// RS04 fault flags
  uint8_t fault_code;
  /// 0=Reset, 1=Calibration, 2=Run
  uint8_t mode_status;
} biped_msgs__msg__MotorState;

// Struct for a sequence of biped_msgs__msg__MotorState.
typedef struct biped_msgs__msg__MotorState__Sequence
{
  biped_msgs__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} biped_msgs__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BIPED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
