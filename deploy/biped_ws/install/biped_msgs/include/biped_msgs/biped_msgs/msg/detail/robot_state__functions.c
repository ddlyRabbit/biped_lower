// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from biped_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "biped_msgs/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `fsm_state`
// Member `active_faults`
#include "rosidl_runtime_c/string_functions.h"

bool
biped_msgs__msg__RobotState__init(biped_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    biped_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // fsm_state
  if (!rosidl_runtime_c__String__init(&msg->fsm_state)) {
    biped_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // safety_ok
  // all_motors_enabled
  // battery_voltage
  // active_faults
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_faults, 0)) {
    biped_msgs__msg__RobotState__fini(msg);
    return false;
  }
  return true;
}

void
biped_msgs__msg__RobotState__fini(biped_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fsm_state
  rosidl_runtime_c__String__fini(&msg->fsm_state);
  // safety_ok
  // all_motors_enabled
  // battery_voltage
  // active_faults
  rosidl_runtime_c__String__Sequence__fini(&msg->active_faults);
}

bool
biped_msgs__msg__RobotState__are_equal(const biped_msgs__msg__RobotState * lhs, const biped_msgs__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // fsm_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->fsm_state), &(rhs->fsm_state)))
  {
    return false;
  }
  // safety_ok
  if (lhs->safety_ok != rhs->safety_ok) {
    return false;
  }
  // all_motors_enabled
  if (lhs->all_motors_enabled != rhs->all_motors_enabled) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // active_faults
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_faults), &(rhs->active_faults)))
  {
    return false;
  }
  return true;
}

bool
biped_msgs__msg__RobotState__copy(
  const biped_msgs__msg__RobotState * input,
  biped_msgs__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // fsm_state
  if (!rosidl_runtime_c__String__copy(
      &(input->fsm_state), &(output->fsm_state)))
  {
    return false;
  }
  // safety_ok
  output->safety_ok = input->safety_ok;
  // all_motors_enabled
  output->all_motors_enabled = input->all_motors_enabled;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // active_faults
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_faults), &(output->active_faults)))
  {
    return false;
  }
  return true;
}

biped_msgs__msg__RobotState *
biped_msgs__msg__RobotState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biped_msgs__msg__RobotState * msg = (biped_msgs__msg__RobotState *)allocator.allocate(sizeof(biped_msgs__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(biped_msgs__msg__RobotState));
  bool success = biped_msgs__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
biped_msgs__msg__RobotState__destroy(biped_msgs__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    biped_msgs__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
biped_msgs__msg__RobotState__Sequence__init(biped_msgs__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biped_msgs__msg__RobotState * data = NULL;

  if (size) {
    data = (biped_msgs__msg__RobotState *)allocator.zero_allocate(size, sizeof(biped_msgs__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = biped_msgs__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        biped_msgs__msg__RobotState__fini(&data[i - 1]);
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
biped_msgs__msg__RobotState__Sequence__fini(biped_msgs__msg__RobotState__Sequence * array)
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
      biped_msgs__msg__RobotState__fini(&array->data[i]);
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

biped_msgs__msg__RobotState__Sequence *
biped_msgs__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biped_msgs__msg__RobotState__Sequence * array = (biped_msgs__msg__RobotState__Sequence *)allocator.allocate(sizeof(biped_msgs__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = biped_msgs__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
biped_msgs__msg__RobotState__Sequence__destroy(biped_msgs__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    biped_msgs__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
biped_msgs__msg__RobotState__Sequence__are_equal(const biped_msgs__msg__RobotState__Sequence * lhs, const biped_msgs__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!biped_msgs__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
biped_msgs__msg__RobotState__Sequence__copy(
  const biped_msgs__msg__RobotState__Sequence * input,
  biped_msgs__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(biped_msgs__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    biped_msgs__msg__RobotState * data =
      (biped_msgs__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!biped_msgs__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          biped_msgs__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!biped_msgs__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
