// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from biped_msgs:msg/MotorStateArray.idl
// generated code does not contain a copyright notice

#include "biped_msgs/msg/detail/motor_state_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_biped_msgs
const rosidl_type_hash_t *
biped_msgs__msg__MotorStateArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb4, 0x5b, 0x8e, 0x71, 0xd2, 0x76, 0x0f, 0x99,
      0xb1, 0x3d, 0x6c, 0x5d, 0x24, 0xac, 0x34, 0x56,
      0x50, 0x9b, 0x82, 0x4d, 0xf9, 0x38, 0x4a, 0x58,
      0xeb, 0x8d, 0x1f, 0x79, 0x64, 0x99, 0x53, 0xce,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "biped_msgs/msg/detail/motor_state__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t biped_msgs__msg__MotorState__EXPECTED_HASH = {1, {
    0x03, 0x2b, 0xf8, 0x9c, 0x4c, 0x59, 0x12, 0x9a,
    0xe1, 0x4a, 0xe6, 0x56, 0x52, 0x56, 0xf4, 0xe9,
    0xec, 0xe5, 0x41, 0x45, 0xc7, 0x24, 0xd3, 0x04,
    0x33, 0x75, 0xfe, 0x5d, 0x16, 0x9a, 0xf8, 0xe2,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char biped_msgs__msg__MotorStateArray__TYPE_NAME[] = "biped_msgs/msg/MotorStateArray";
static char biped_msgs__msg__MotorState__TYPE_NAME[] = "biped_msgs/msg/MotorState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char biped_msgs__msg__MotorStateArray__FIELD_NAME__header[] = "header";
static char biped_msgs__msg__MotorStateArray__FIELD_NAME__motors[] = "motors";

static rosidl_runtime_c__type_description__Field biped_msgs__msg__MotorStateArray__FIELDS[] = {
  {
    {biped_msgs__msg__MotorStateArray__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {biped_msgs__msg__MotorStateArray__FIELD_NAME__motors, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {biped_msgs__msg__MotorState__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription biped_msgs__msg__MotorStateArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {biped_msgs__msg__MotorState__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
biped_msgs__msg__MotorStateArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {biped_msgs__msg__MotorStateArray__TYPE_NAME, 30, 30},
      {biped_msgs__msg__MotorStateArray__FIELDS, 2, 2},
    },
    {biped_msgs__msg__MotorStateArray__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&biped_msgs__msg__MotorState__EXPECTED_HASH, biped_msgs__msg__MotorState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = biped_msgs__msg__MotorState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "biped_msgs/MotorState[] motors";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
biped_msgs__msg__MotorStateArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {biped_msgs__msg__MotorStateArray__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 54, 54},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
biped_msgs__msg__MotorStateArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *biped_msgs__msg__MotorStateArray__get_individual_type_description_source(NULL),
    sources[1] = *biped_msgs__msg__MotorState__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
