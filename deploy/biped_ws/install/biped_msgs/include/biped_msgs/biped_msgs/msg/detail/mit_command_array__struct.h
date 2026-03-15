// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from biped_msgs:msg/MITCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "biped_msgs/msg/mit_command_array.h"


#ifndef BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__STRUCT_H_
#define BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__STRUCT_H_

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
// Member 'commands'
#include "biped_msgs/msg/detail/mit_command__struct.h"

/// Struct defined in msg/MITCommandArray in the package biped_msgs.
typedef struct biped_msgs__msg__MITCommandArray
{
  std_msgs__msg__Header header;
  biped_msgs__msg__MITCommand__Sequence commands;
} biped_msgs__msg__MITCommandArray;

// Struct for a sequence of biped_msgs__msg__MITCommandArray.
typedef struct biped_msgs__msg__MITCommandArray__Sequence
{
  biped_msgs__msg__MITCommandArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} biped_msgs__msg__MITCommandArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BIPED_MSGS__MSG__DETAIL__MIT_COMMAND_ARRAY__STRUCT_H_
