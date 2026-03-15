// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from biped_msgs:msg/MITCommand.idl
// generated code does not contain a copyright notice

#include "biped_msgs/msg/detail/mit_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_biped_msgs
const rosidl_type_hash_t *
biped_msgs__msg__MITCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x72, 0xd8, 0x7c, 0xd3, 0x66, 0x45, 0x22, 0xd0,
      0xa0, 0x29, 0x7e, 0x69, 0xf4, 0xf9, 0x21, 0xc2,
      0x4d, 0xaa, 0x2e, 0x18, 0x08, 0x68, 0x0d, 0x45,
      0xea, 0x31, 0x36, 0x94, 0xa7, 0x14, 0xc1, 0x0a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char biped_msgs__msg__MITCommand__TYPE_NAME[] = "biped_msgs/msg/MITCommand";

// Define type names, field names, and default values
static char biped_msgs__msg__MITCommand__FIELD_NAME__joint_name[] = "joint_name";
static char biped_msgs__msg__MITCommand__FIELD_NAME__position[] = "position";
static char biped_msgs__msg__MITCommand__FIELD_NAME__velocity[] = "velocity";
static char biped_msgs__msg__MITCommand__FIELD_NAME__kp[] = "kp";
static char biped_msgs__msg__MITCommand__FIELD_NAME__kd[] = "kd";
static char biped_msgs__msg__MITCommand__FIELD_NAME__torque_ff[] = "torque_ff";

static rosidl_runtime_c__type_description__Field biped_msgs__msg__MITCommand__FIELDS[] = {
  {
    {biped_msgs__msg__MITCommand__FIELD_NAME__joint_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MITCommand__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MITCommand__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MITCommand__FIELD_NAME__kp, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MITCommand__FIELD_NAME__kd, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MITCommand__FIELD_NAME__torque_ff, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
biped_msgs__msg__MITCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {biped_msgs__msg__MITCommand__TYPE_NAME, 25, 25},
      {biped_msgs__msg__MITCommand__FIELDS, 6, 6},
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
  "float32 position        # target position (rad)\n"
  "float32 velocity        # target velocity (rad/s), typically 0\n"
  "float32 kp              # position gain\n"
  "float32 kd              # velocity gain\n"
  "float32 torque_ff       # feedforward torque (Nm), typically 0";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
biped_msgs__msg__MITCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {biped_msgs__msg__MITCommand__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 272, 272},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
biped_msgs__msg__MITCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *biped_msgs__msg__MITCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
