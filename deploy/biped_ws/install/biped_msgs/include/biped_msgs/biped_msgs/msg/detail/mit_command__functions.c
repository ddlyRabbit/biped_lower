// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from biped_msgs:msg/MITCommand.idl
// generated code does not contain a copyright notice
#include "biped_msgs/msg/detail/mit_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `joint_name`
#include "rosidl_runtime_c/string_functions.h"

bool
biped_msgs__msg__MITCommand__init(biped_msgs__msg__MITCommand * msg)
{
  if (!msg) {
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__init(&msg->joint_name)) {
    biped_msgs__msg__MITCommand__fini(msg);
    return false;
  }
  // position
  // velocity
  // kp
  // kd
  // torque_ff
  return true;
}

void
biped_msgs__msg__MITCommand__fini(biped_msgs__msg__MITCommand * msg)
{
  if (!msg) {
    return;
  }
  // joint_name
  rosidl_runtime_c__String__fini(&msg->joint_name);
  // position
  // velocity
  // kp
  // kd
  // torque_ff
}

bool
biped_msgs__msg__MITCommand__are_equal(const biped_msgs__msg__MITCommand * lhs, const biped_msgs__msg__MITCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->joint_name), &(rhs->joint_name)))
  {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // kp
  if (lhs->kp != rhs->kp) {
    return false;
  }
  // kd
  if (lhs->kd != rhs->kd) {
    return false;
  }
  // torque_ff
  if (lhs->torque_ff != rhs->torque_ff) {
    return false;
  }
  return true;
}

bool
biped_msgs__msg__MITCommand__copy(
  const biped_msgs__msg__MITCommand * input,
  biped_msgs__msg__MITCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__copy(
      &(input->joint_name), &(output->joint_name)))
  {
    return false;
  }
  // position
  output->position = input->position;
  // velocity
  output->velocity = input->velocity;
  // kp
  output->kp = input->kp;
  // kd
  output->kd = input->kd;
  // torque_ff
  output->torque_ff = input->torque_ff;
  return true;
}

biped_msgs__msg__MITCommand *
biped_msgs__msg__MITCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biped_msgs__msg__MITCommand * msg = (biped_msgs__msg__MITCommand *)allocator.allocate(sizeof(biped_msgs__msg__MITCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(biped_msgs__msg__MITCommand));
  bool success = biped_msgs__msg__MITCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
biped_msgs__msg__MITCommand__destroy(biped_msgs__msg__MITCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    biped_msgs__msg__MITCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
biped_msgs__msg__MITCommand__Sequence__init(biped_msgs__msg__MITCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biped_msgs__msg__MITCommand * data = NULL;

  if (size) {
    data = (biped_msgs__msg__MITCommand *)allocator.zero_allocate(size, sizeof(biped_msgs__msg__MITCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = biped_msgs__msg__MITCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        biped_msgs__msg__MITCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
biped_msgs__msg__MITCommand__Sequence__fini(biped_msgs__msg__MITCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      biped_msgs__msg__MITCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

biped_msgs__msg__MITCommand__Sequence *
biped_msgs__msg__MITCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biped_msgs__msg__MITCommand__Sequence * array = (biped_msgs__msg__MITCommand__Sequence *)allocator.allocate(sizeof(biped_msgs__msg__MITCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = biped_msgs__msg__MITCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
biped_msgs__msg__MITCommand__Sequence__destroy(biped_msgs__msg__MITCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    biped_msgs__msg__MITCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
biped_msgs__msg__MITCommand__Sequence__are_equal(const biped_msgs__msg__MITCommand__Sequence * lhs, const biped_msgs__msg__MITCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!biped_msgs__msg__MITCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
biped_msgs__msg__MITCommand__Sequence__copy(
  const biped_msgs__msg__MITCommand__Sequence * input,
  biped_msgs__msg__MITCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(biped_msgs__msg__MITCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    biped_msgs__msg__MITCommand * data =
      (biped_msgs__msg__MITCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!biped_msgs__msg__MITCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          biped_msgs__msg__MITCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!biped_msgs__msg__MITCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
