// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from biped_msgs:msg/MITCommandArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "biped_msgs/msg/detail/mit_command_array__functions.h"
#include "biped_msgs/msg/detail/mit_command_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace biped_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MITCommandArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) biped_msgs::msg::MITCommandArray(_init);
}

void MITCommandArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<biped_msgs::msg::MITCommandArray *>(message_memory);
  typed_message->~MITCommandArray();
}

size_t size_function__MITCommandArray__commands(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<biped_msgs::msg::MITCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MITCommandArray__commands(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<biped_msgs::msg::MITCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MITCommandArray__commands(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<biped_msgs::msg::MITCommand> *>(untyped_member);
  return &member[index];
}

void fetch_function__MITCommandArray__commands(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const biped_msgs::msg::MITCommand *>(
    get_const_function__MITCommandArray__commands(untyped_member, index));
  auto & value = *reinterpret_cast<biped_msgs::msg::MITCommand *>(untyped_value);
  value = item;
}

void assign_function__MITCommandArray__commands(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<biped_msgs::msg::MITCommand *>(
    get_function__MITCommandArray__commands(untyped_member, index));
  const auto & value = *reinterpret_cast<const biped_msgs::msg::MITCommand *>(untyped_value);
  item = value;
}

void resize_function__MITCommandArray__commands(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<biped_msgs::msg::MITCommand> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MITCommandArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biped_msgs::msg::MITCommandArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "commands",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<biped_msgs::msg::MITCommand>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biped_msgs::msg::MITCommandArray, commands),  // bytes offset in struct
    nullptr,  // default value
    size_function__MITCommandArray__commands,  // size() function pointer
    get_const_function__MITCommandArray__commands,  // get_const(index) function pointer
    get_function__MITCommandArray__commands,  // get(index) function pointer
    fetch_function__MITCommandArray__commands,  // fetch(index, &value) function pointer
    assign_function__MITCommandArray__commands,  // assign(index, value) function pointer
    resize_function__MITCommandArray__commands  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MITCommandArray_message_members = {
  "biped_msgs::msg",  // message namespace
  "MITCommandArray",  // message name
  2,  // number of fields
  sizeof(biped_msgs::msg::MITCommandArray),
  false,  // has_any_key_member_
  MITCommandArray_message_member_array,  // message members
  MITCommandArray_init_function,  // function to initialize message memory (memory has to be allocated)
  MITCommandArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MITCommandArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MITCommandArray_message_members,
  get_message_typesupport_handle_function,
  &biped_msgs__msg__MITCommandArray__get_type_hash,
  &biped_msgs__msg__MITCommandArray__get_type_description,
  &biped_msgs__msg__MITCommandArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace biped_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<biped_msgs::msg::MITCommandArray>()
{
  return &::biped_msgs::msg::rosidl_typesupport_introspection_cpp::MITCommandArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, biped_msgs, msg, MITCommandArray)() {
  return &::biped_msgs::msg::rosidl_typesupport_introspection_cpp::MITCommandArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
