// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OpenDroneIDOperatorID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ID_TYPE_CAA'.
/**
  * [[[cog:
  * import mavros_cog
  * mavros_cog.idl_decl_enum('MAV_ODID_OPERATOR_ID_TYPE', 'ID_TYPE_')
  * ]]]
  * MAV_ODID_OPERATOR_ID_TYPE
  * CAA (Civil Aviation Authority) registered operator ID.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDOperatorID__ID_TYPE_CAA = 0
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'id_or_mac'
// Member 'operator_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/OpenDroneIDOperatorID in the package mavros_msgs.
/**
  * Remote ID message - Operator ID
  * https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_OPERATOR_ID
 */
typedef struct mavros_msgs__msg__OpenDroneIDOperatorID
{
  /// ]] (checksum: be1240b63d8b101630595dd9ae4a42d4)
  std_msgs__msg__Header header;
  rosidl_runtime_c__String id_or_mac;
  uint8_t operator_id_type;
  rosidl_runtime_c__String operator_id;
} mavros_msgs__msg__OpenDroneIDOperatorID;

// Struct for a sequence of mavros_msgs__msg__OpenDroneIDOperatorID.
typedef struct mavros_msgs__msg__OpenDroneIDOperatorID__Sequence
{
  mavros_msgs__msg__OpenDroneIDOperatorID * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OpenDroneIDOperatorID__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__STRUCT_H_
