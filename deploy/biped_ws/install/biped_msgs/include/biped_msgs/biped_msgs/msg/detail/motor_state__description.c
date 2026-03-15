// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from biped_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#include "biped_msgs/msg/detail/motor_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_biped_msgs
const rosidl_type_hash_t *
biped_msgs__msg__MotorState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x03, 0x2b, 0xf8, 0x9c, 0x4c, 0x59, 0x12, 0x9a,
      0xe1, 0x4a, 0xe6, 0x56, 0x52, 0x56, 0xf4, 0xe9,
      0xec, 0xe5, 0x41, 0x45, 0xc7, 0x24, 0xd3, 0x04,
      0x33, 0x75, 0xfe, 0x5d, 0x16, 0x9a, 0xf8, 0xe2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char biped_msgs__msg__MotorState__TYPE_NAME[] = "biped_msgs/msg/MotorState";

// Define type names, field names, and default values
static char biped_msgs__msg__MotorState__FIELD_NAME__joint_name[] = "joint_name";
static char biped_msgs__msg__MotorState__FIELD_NAME__can_id[] = "can_id";
static char biped_msgs__msg__MotorState__FIELD_NAME__position[] = "position";
static char biped_msgs__msg__MotorState__FIELD_NAME__velocity[] = "velocity";
static char biped_msgs__msg__MotorState__FIELD_NAME__torque[] = "torque";
static char biped_msgs__msg__MotorState__FIELD_NAME__temperature[] = "temperature";
static char biped_msgs__msg__MotorState__FIELD_NAME__fault_code[] = "fault_code";
static char biped_msgs__msg__MotorState__FIELD_NAME__mode_status[] = "mode_status";

static rosidl_runtime_c__type_description__Field biped_msgs__msg__MotorState__FIELDS[] = {
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__joint_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__can_id, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__torque, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__fault_code, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorState__FIELD_NAME__mode_status, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
biped_msgs__msg__MotorState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {biped_msgs__msg__MotorState__TYPE_NAME, 25, 25},
      {biped_msgs__msg__MotorState__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string joint_name\n"
  "uint8 can_id\n"
  "float32 position        # absolute encoder position (rad, calibrated)\n"
  "float32 velocity        # current velocity (rad/s)\n"
  "float32 torque          # current torque (Nm)\n"
  "float32 temperature     # motor temp (\\xc2\\xb0C)\n"
  "uint8 fault_code        # RS04 fault flags\n"
  "uint8 mode_status       # 0=Reset, 1=Calibration, 2=Run";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
biped_msgs__msg__MotorState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {biped_msgs__msg__MotorState__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 339, 339},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
biped_msgs__msg__MotorState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *biped_msgs__msg__MotorState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
