// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OpenDroneIDSelfID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DESC_TYPE_TEXT'.
/**
  * [[[cog:
  * import mavros_cog
  * mavros_cog.idl_decl_enum('MAV_ODID_DESC_TYPE', 'DESC_TYPE_')
  * ]]]
  * MAV_ODID_DESC_TYPE
  * Free-form text description of the purpose of the flight.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSelfID__DESC_TYPE_TEXT = 0
};

/// Constant 'DESC_TYPE_EMERGENCY'.
/**
  * Optional additional clarification when status == MAV_ODID_STATUS_EMERGENCY.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSelfID__DESC_TYPE_EMERGENCY = 1
};

/// Constant 'DESC_TYPE_EXTENDED_STATUS'.
/**
  * Optional additional clarification when status != MAV_ODID_STATUS_EMERGENCY.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSelfID__DESC_TYPE_EXTENDED_STATUS = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'id_or_mac'
// Member 'description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/OpenDroneIDSelfID in the package mavros_msgs.
/**
  * Remote ID message - Self ID
  * https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SELF_ID
 */
typedef struct mavros_msgs__msg__OpenDroneIDSelfID
{
  /// ]] (checksum: 08bf4b7e669d01d655893e5d543f86b1)
  std_msgs__msg__Header header;
  rosidl_runtime_c__String id_or_mac;
  uint8_t description_type;
  rosidl_runtime_c__String description;
} mavros_msgs__msg__OpenDroneIDSelfID;

// Struct for a sequence of mavros_msgs__msg__OpenDroneIDSelfID.
typedef struct mavros_msgs__msg__OpenDroneIDSelfID__Sequence
{
  mavros_msgs__msg__OpenDroneIDSelfID * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OpenDroneIDSelfID__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__STRUCT_H_
