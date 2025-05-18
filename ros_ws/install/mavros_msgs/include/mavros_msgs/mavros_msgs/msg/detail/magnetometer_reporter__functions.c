// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/MagnetometerReporter.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/magnetometer_reporter__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__MagnetometerReporter__init(mavros_msgs__msg__MagnetometerReporter * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__MagnetometerReporter__fini(msg);
    return false;
  }
  // report
  // confidence
  // compass_id
  // cal_mask
  // cal_status
  // autosaved
  // fitness
  // ofs_x
  // ofs_y
  // ofs_z
  // diag_x
  // diag_y
  // diag_z
  // offdiag_x
  // offdiag_y
  // offdiag_z
  // orientation_confidence
  // old_orientation
  // new_orientation
  // scale_factor
  return true;
}

void
mavros_msgs__msg__MagnetometerReporter__fini(mavros_msgs__msg__MagnetometerReporter * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // report
  // confidence
  // compass_id
  // cal_mask
  // cal_status
  // autosaved
  // fitness
  // ofs_x
  // ofs_y
  // ofs_z
  // diag_x
  // diag_y
  // diag_z
  // offdiag_x
  // offdiag_y
  // offdiag_z
  // orientation_confidence
  // old_orientation
  // new_orientation
  // scale_factor
}

bool
mavros_msgs__msg__MagnetometerReporter__are_equal(const mavros_msgs__msg__MagnetometerReporter * lhs, const mavros_msgs__msg__MagnetometerReporter * rhs)
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
  // report
  if (lhs->report != rhs->report) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // compass_id
  if (lhs->compass_id != rhs->compass_id) {
    return false;
  }
  // cal_mask
  if (lhs->cal_mask != rhs->cal_mask) {
    return false;
  }
  // cal_status
  if (lhs->cal_status != rhs->cal_status) {
    return false;
  }
  // autosaved
  if (lhs->autosaved != rhs->autosaved) {
    return false;
  }
  // fitness
  if (lhs->fitness != rhs->fitness) {
    return false;
  }
  // ofs_x
  if (lhs->ofs_x != rhs->ofs_x) {
    return false;
  }
  // ofs_y
  if (lhs->ofs_y != rhs->ofs_y) {
    return false;
  }
  // ofs_z
  if (lhs->ofs_z != rhs->ofs_z) {
    return false;
  }
  // diag_x
  if (lhs->diag_x != rhs->diag_x) {
    return false;
  }
  // diag_y
  if (lhs->diag_y != rhs->diag_y) {
    return false;
  }
  // diag_z
  if (lhs->diag_z != rhs->diag_z) {
    return false;
  }
  // offdiag_x
  if (lhs->offdiag_x != rhs->offdiag_x) {
    return false;
  }
  // offdiag_y
  if (lhs->offdiag_y != rhs->offdiag_y) {
    return false;
  }
  // offdiag_z
  if (lhs->offdiag_z != rhs->offdiag_z) {
    return false;
  }
  // orientation_confidence
  if (lhs->orientation_confidence != rhs->orientation_confidence) {
    return false;
  }
  // old_orientation
  if (lhs->old_orientation != rhs->old_orientation) {
    return false;
  }
  // new_orientation
  if (lhs->new_orientation != rhs->new_orientation) {
    return false;
  }
  // scale_factor
  if (lhs->scale_factor != rhs->scale_factor) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__MagnetometerReporter__copy(
  const mavros_msgs__msg__MagnetometerReporter * input,
  mavros_msgs__msg__MagnetometerReporter * output)
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
  // report
  output->report = input->report;
  // confidence
  output->confidence = input->confidence;
  // compass_id
  output->compass_id = input->compass_id;
  // cal_mask
  output->cal_mask = input->cal_mask;
  // cal_status
  output->cal_status = input->cal_status;
  // autosaved
  output->autosaved = input->autosaved;
  // fitness
  output->fitness = input->fitness;
  // ofs_x
  output->ofs_x = input->ofs_x;
  // ofs_y
  output->ofs_y = input->ofs_y;
  // ofs_z
  output->ofs_z = input->ofs_z;
  // diag_x
  output->diag_x = input->diag_x;
  // diag_y
  output->diag_y = input->diag_y;
  // diag_z
  output->diag_z = input->diag_z;
  // offdiag_x
  output->offdiag_x = input->offdiag_x;
  // offdiag_y
  output->offdiag_y = input->offdiag_y;
  // offdiag_z
  output->offdiag_z = input->offdiag_z;
  // orientation_confidence
  output->orientation_confidence = input->orientation_confidence;
  // old_orientation
  output->old_orientation = input->old_orientation;
  // new_orientation
  output->new_orientation = input->new_orientation;
  // scale_factor
  output->scale_factor = input->scale_factor;
  return true;
}

mavros_msgs__msg__MagnetometerReporter *
mavros_msgs__msg__MagnetometerReporter__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__MagnetometerReporter * msg = (mavros_msgs__msg__MagnetometerReporter *)allocator.allocate(sizeof(mavros_msgs__msg__MagnetometerReporter), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__MagnetometerReporter));
  bool success = mavros_msgs__msg__MagnetometerReporter__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__MagnetometerReporter__destroy(mavros_msgs__msg__MagnetometerReporter * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__MagnetometerReporter__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__MagnetometerReporter__Sequence__init(mavros_msgs__msg__MagnetometerReporter__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__MagnetometerReporter * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__MagnetometerReporter *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__MagnetometerReporter), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__MagnetometerReporter__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__MagnetometerReporter__fini(&data[i - 1]);
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
mavros_msgs__msg__MagnetometerReporter__Sequence__fini(mavros_msgs__msg__MagnetometerReporter__Sequence * array)
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
      mavros_msgs__msg__MagnetometerReporter__fini(&array->data[i]);
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

mavros_msgs__msg__MagnetometerReporter__Sequence *
mavros_msgs__msg__MagnetometerReporter__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__MagnetometerReporter__Sequence * array = (mavros_msgs__msg__MagnetometerReporter__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__MagnetometerReporter__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__MagnetometerReporter__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__MagnetometerReporter__Sequence__destroy(mavros_msgs__msg__MagnetometerReporter__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__MagnetometerReporter__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__MagnetometerReporter__Sequence__are_equal(const mavros_msgs__msg__MagnetometerReporter__Sequence * lhs, const mavros_msgs__msg__MagnetometerReporter__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__MagnetometerReporter__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__MagnetometerReporter__Sequence__copy(
  const mavros_msgs__msg__MagnetometerReporter__Sequence * input,
  mavros_msgs__msg__MagnetometerReporter__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__MagnetometerReporter);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__MagnetometerReporter * data =
      (mavros_msgs__msg__MagnetometerReporter *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__MagnetometerReporter__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__MagnetometerReporter__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__MagnetometerReporter__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
