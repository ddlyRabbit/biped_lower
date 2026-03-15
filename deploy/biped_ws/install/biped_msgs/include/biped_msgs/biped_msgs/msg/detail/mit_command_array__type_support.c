// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from biped_msgs:msg/MITCommandArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "biped_msgs/msg/detail/mit_command_array__rosidl_typesupport_introspection_c.h"
#include "biped_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "biped_msgs/msg/detail/mit_command_array__functions.h"
#include "biped_msgs/msg/detail/mit_command_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `commands`
#include "biped_msgs/msg/mit_command.h"
// Member `commands`
#include "biped_msgs/msg/detail/mit_command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  biped_msgs__msg__MITCommandArray__init(message_memory);
}

void biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_fini_function(void * message_memory)
{
  biped_msgs__msg__MITCommandArray__fini(message_memory);
}

size_t biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__size_function__MITCommandArray__commands(
  const void * untyped_member)
{
  const biped_msgs__msg__MITCommand__Sequence * member =
    (const biped_msgs__msg__MITCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__get_const_function__MITCommandArray__commands(
  const void * untyped_member, size_t index)
{
  const biped_msgs__msg__MITCommand__Sequence * member =
    (const biped_msgs__msg__MITCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__get_function__MITCommandArray__commands(
  void * untyped_member, size_t index)
{
  biped_msgs__msg__MITCommand__Sequence * member =
    (biped_msgs__msg__MITCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__fetch_function__MITCommandArray__commands(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const biped_msgs__msg__MITCommand * item =
    ((const biped_msgs__msg__MITCommand *)
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__get_const_function__MITCommandArray__commands(untyped_member, index));
  biped_msgs__msg__MITCommand * value =
    (biped_msgs__msg__MITCommand *)(untyped_value);
  *value = *item;
}

void biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__assign_function__MITCommandArray__commands(
  void * untyped_member, size_t index, const void * untyped_value)
{
  biped_msgs__msg__MITCommand * item =
    ((biped_msgs__msg__MITCommand *)
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__get_function__MITCommandArray__commands(untyped_member, index));
  const biped_msgs__msg__MITCommand * value =
    (const biped_msgs__msg__MITCommand *)(untyped_value);
  *item = *value;
}

bool biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__resize_function__MITCommandArray__commands(
  void * untyped_member, size_t size)
{
  biped_msgs__msg__MITCommand__Sequence * member =
    (biped_msgs__msg__MITCommand__Sequence *)(untyped_member);
  biped_msgs__msg__MITCommand__Sequence__fini(member);
  return biped_msgs__msg__MITCommand__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biped_msgs__msg__MITCommandArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "commands",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biped_msgs__msg__MITCommandArray, commands),  // bytes offset in struct
    NULL,  // default value
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__size_function__MITCommandArray__commands,  // size() function pointer
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__get_const_function__MITCommandArray__commands,  // get_const(index) function pointer
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__get_function__MITCommandArray__commands,  // get(index) function pointer
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__fetch_function__MITCommandArray__commands,  // fetch(index, &value) function pointer
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__assign_function__MITCommandArray__commands,  // assign(index, value) function pointer
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__resize_function__MITCommandArray__commands  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_members = {
  "biped_msgs__msg",  // message namespace
  "MITCommandArray",  // message name
  2,  // number of fields
  sizeof(biped_msgs__msg__MITCommandArray),
  false,  // has_any_key_member_
  biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_member_array,  // message members
  biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_init_function,  // function to initialize message memory (memory has to be allocated)
  biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_type_support_handle = {
  0,
  &biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_members,
  get_message_typesupport_handle_function,
  &biped_msgs__msg__MITCommandArray__get_type_hash,
  &biped_msgs__msg__MITCommandArray__get_type_description,
  &biped_msgs__msg__MITCommandArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_biped_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, biped_msgs, msg, MITCommandArray)() {
  biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, biped_msgs, msg, MITCommand)();
  if (!biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_type_support_handle.typesupport_identifier) {
    biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &biped_msgs__msg__MITCommandArray__rosidl_typesupport_introspection_c__MITCommandArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
