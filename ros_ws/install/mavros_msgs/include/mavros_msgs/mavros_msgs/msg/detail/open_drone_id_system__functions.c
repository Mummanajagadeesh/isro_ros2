// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/OpenDroneIDSystem.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/open_drone_id_system__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `id_or_mac`
#include "rosidl_runtime_c/string_functions.h"

bool
mavros_msgs__msg__OpenDroneIDSystem__init(mavros_msgs__msg__OpenDroneIDSystem * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__OpenDroneIDSystem__fini(msg);
    return false;
  }
  // id_or_mac
  if (!rosidl_runtime_c__String__init(&msg->id_or_mac)) {
    mavros_msgs__msg__OpenDroneIDSystem__fini(msg);
    return false;
  }
  // operator_location_type
  // classification_type
  // operator_latitude
  // operator_longitude
  // area_count
  // area_radius
  // area_ceiling
  // area_floor
  // category_eu
  // class_eu
  // operator_altitude_geo
  return true;
}

void
mavros_msgs__msg__OpenDroneIDSystem__fini(mavros_msgs__msg__OpenDroneIDSystem * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id_or_mac
  rosidl_runtime_c__String__fini(&msg->id_or_mac);
  // operator_location_type
  // classification_type
  // operator_latitude
  // operator_longitude
  // area_count
  // area_radius
  // area_ceiling
  // area_floor
  // category_eu
  // class_eu
  // operator_altitude_geo
}

bool
mavros_msgs__msg__OpenDroneIDSystem__are_equal(const mavros_msgs__msg__OpenDroneIDSystem * lhs, const mavros_msgs__msg__OpenDroneIDSystem * rhs)
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
  // id_or_mac
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->id_or_mac), &(rhs->id_or_mac)))
  {
    return false;
  }
  // operator_location_type
  if (lhs->operator_location_type != rhs->operator_location_type) {
    return false;
  }
  // classification_type
  if (lhs->classification_type != rhs->classification_type) {
    return false;
  }
  // operator_latitude
  if (lhs->operator_latitude != rhs->operator_latitude) {
    return false;
  }
  // operator_longitude
  if (lhs->operator_longitude != rhs->operator_longitude) {
    return false;
  }
  // area_count
  if (lhs->area_count != rhs->area_count) {
    return false;
  }
  // area_radius
  if (lhs->area_radius != rhs->area_radius) {
    return false;
  }
  // area_ceiling
  if (lhs->area_ceiling != rhs->area_ceiling) {
    return false;
  }
  // area_floor
  if (lhs->area_floor != rhs->area_floor) {
    return false;
  }
  // category_eu
  if (lhs->category_eu != rhs->category_eu) {
    return false;
  }
  // class_eu
  if (lhs->class_eu != rhs->class_eu) {
    return false;
  }
  // operator_altitude_geo
  if (lhs->operator_altitude_geo != rhs->operator_altitude_geo) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__OpenDroneIDSystem__copy(
  const mavros_msgs__msg__OpenDroneIDSystem * input,
  mavros_msgs__msg__OpenDroneIDSystem * output)
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
  // id_or_mac
  if (!rosidl_runtime_c__String__copy(
      &(input->id_or_mac), &(output->id_or_mac)))
  {
    return false;
  }
  // operator_location_type
  output->operator_location_type = input->operator_location_type;
  // classification_type
  output->classification_type = input->classification_type;
  // operator_latitude
  output->operator_latitude = input->operator_latitude;
  // operator_longitude
  output->operator_longitude = input->operator_longitude;
  // area_count
  output->area_count = input->area_count;
  // area_radius
  output->area_radius = input->area_radius;
  // area_ceiling
  output->area_ceiling = input->area_ceiling;
  // area_floor
  output->area_floor = input->area_floor;
  // category_eu
  output->category_eu = input->category_eu;
  // class_eu
  output->class_eu = input->class_eu;
  // operator_altitude_geo
  output->operator_altitude_geo = input->operator_altitude_geo;
  return true;
}

mavros_msgs__msg__OpenDroneIDSystem *
mavros_msgs__msg__OpenDroneIDSystem__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpenDroneIDSystem * msg = (mavros_msgs__msg__OpenDroneIDSystem *)allocator.allocate(sizeof(mavros_msgs__msg__OpenDroneIDSystem), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__OpenDroneIDSystem));
  bool success = mavros_msgs__msg__OpenDroneIDSystem__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__OpenDroneIDSystem__destroy(mavros_msgs__msg__OpenDroneIDSystem * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__OpenDroneIDSystem__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__OpenDroneIDSystem__Sequence__init(mavros_msgs__msg__OpenDroneIDSystem__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpenDroneIDSystem * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__OpenDroneIDSystem *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__OpenDroneIDSystem), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__OpenDroneIDSystem__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__OpenDroneIDSystem__fini(&data[i - 1]);
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
mavros_msgs__msg__OpenDroneIDSystem__Sequence__fini(mavros_msgs__msg__OpenDroneIDSystem__Sequence * array)
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
      mavros_msgs__msg__OpenDroneIDSystem__fini(&array->data[i]);
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

mavros_msgs__msg__OpenDroneIDSystem__Sequence *
mavros_msgs__msg__OpenDroneIDSystem__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpenDroneIDSystem__Sequence * array = (mavros_msgs__msg__OpenDroneIDSystem__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__OpenDroneIDSystem__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__OpenDroneIDSystem__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__OpenDroneIDSystem__Sequence__destroy(mavros_msgs__msg__OpenDroneIDSystem__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__OpenDroneIDSystem__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__OpenDroneIDSystem__Sequence__are_equal(const mavros_msgs__msg__OpenDroneIDSystem__Sequence * lhs, const mavros_msgs__msg__OpenDroneIDSystem__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__OpenDroneIDSystem__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__OpenDroneIDSystem__Sequence__copy(
  const mavros_msgs__msg__OpenDroneIDSystem__Sequence * input,
  mavros_msgs__msg__OpenDroneIDSystem__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__OpenDroneIDSystem);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__OpenDroneIDSystem * data =
      (mavros_msgs__msg__OpenDroneIDSystem *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__OpenDroneIDSystem__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__OpenDroneIDSystem__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__OpenDroneIDSystem__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
