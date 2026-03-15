// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from biped_msgs:msg/MotorStateArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/motor_state_array.h"


#ifndef BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__STRUCT_H_
#define BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__STRUCT_H_

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
// Member 'motors'
#include "biped_msgs/msg/detail/motor_state__struct.h"

/// Struct defined in msg/MotorStateArray in the package biped_msgs.
typedef struct biped_msgs__msg__MotorStateArray
{
  std_msgs__msg__Header header;
  biped_msgs__msg__MotorState__Sequence motors;
} biped_msgs__msg__MotorStateArray;

// Struct for a sequence of biped_msgs__msg__MotorStateArray.
typedef struct biped_msgs__msg__MotorStateArray__Sequence
{
  biped_msgs__msg__MotorStateArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} biped_msgs__msg__MotorStateArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BIPED_MSGS__MSG__DETAIL__MOTOR_STATE_ARRAY__STRUCT_H_
