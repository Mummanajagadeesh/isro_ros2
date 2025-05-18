// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mavros_msgs:msg/OpenDroneIDSystemUpdate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__FUNCTIONS_H_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mavros_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mavros_msgs/msg/detail/open_drone_id_system_update__struct.h"

/// Initialize msg/OpenDroneIDSystemUpdate message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mavros_msgs__msg__OpenDroneIDSystemUpdate
 * )) before or use
 * mavros_msgs__msg__OpenDroneIDSystemUpdate__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__OpenDroneIDSystemUpdate__init(mavros_msgs__msg__OpenDroneIDSystemUpdate * msg);

/// Finalize msg/OpenDroneIDSystemUpdate message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__OpenDroneIDSystemUpdate__fini(mavros_msgs__msg__OpenDroneIDSystemUpdate * msg);

/// Create msg/OpenDroneIDSystemUpdate message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mavros_msgs__msg__OpenDroneIDSystemUpdate__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
mavros_msgs__msg__OpenDroneIDSystemUpdate *
mavros_msgs__msg__OpenDroneIDSystemUpdate__create();

/// Destroy msg/OpenDroneIDSystemUpdate message.
/**
 * It calls
 * mavros_msgs__msg__OpenDroneIDSystemUpdate__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__OpenDroneIDSystemUpdate__destroy(mavros_msgs__msg__OpenDroneIDSystemUpdate * msg);

/// Check for msg/OpenDroneIDSystemUpdate message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__OpenDroneIDSystemUpdate__are_equal(const mavros_msgs__msg__OpenDroneIDSystemUpdate * lhs, const mavros_msgs__msg__OpenDroneIDSystemUpdate * rhs);

/// Copy a msg/OpenDroneIDSystemUpdate message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__OpenDroneIDSystemUpdate__copy(
  const mavros_msgs__msg__OpenDroneIDSystemUpdate * input,
  mavros_msgs__msg__OpenDroneIDSystemUpdate * output);

/// Initialize array of msg/OpenDroneIDSystemUpdate messages.
/**
 * It allocates the memory for the number of elements and calls
 * mavros_msgs__msg__OpenDroneIDSystemUpdate__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__init(mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence * array, size_t size);

/// Finalize array of msg/OpenDroneIDSystemUpdate messages.
/**
 * It calls
 * mavros_msgs__msg__OpenDroneIDSystemUpdate__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__fini(mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence * array);

/// Create array of msg/OpenDroneIDSystemUpdate messages.
/**
 * It allocates the memory for the array and calls
 * mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence *
mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__create(size_t size);

/// Destroy array of msg/OpenDroneIDSystemUpdate messages.
/**
 * It calls
 * mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__destroy(mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence * array);

/// Check for msg/OpenDroneIDSystemUpdate message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__are_equal(const mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence * lhs, const mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence * rhs);

/// Copy an array of msg/OpenDroneIDSystemUpdate messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence__copy(
  const mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence * input,
  mavros_msgs__msg__OpenDroneIDSystemUpdate__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__FUNCTIONS_H_
