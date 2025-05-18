// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OpenDroneIDSystem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LOCATION_TYPE_TAKEOFF'.
/**
  * [[[cog:
  * import mavros_cog
  * mavros_cog.idl_decl_enum('MAV_ODID_OPERATOR_LOCATION_TYPE', 'LOCATION_TYPE_')
  * mavros_cog.idl_decl_enum('MAV_ODID_CLASSIFICATION_TYPE', 'CLASSIFICATION_TYPE_')
  * ]]]
  * MAV_ODID_OPERATOR_LOCATION_TYPE
  * The location of the operator is the same as the take-off location.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSystem__LOCATION_TYPE_TAKEOFF = 0
};

/// Constant 'LOCATION_TYPE_LIVE_GNSS'.
/**
  * The location of the operator is based on live GNSS data.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSystem__LOCATION_TYPE_LIVE_GNSS = 1
};

/// Constant 'LOCATION_TYPE_FIXED'.
/**
  * The location of the operator is a fixed location.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSystem__LOCATION_TYPE_FIXED = 2
};

/// Constant 'CLASSIFICATION_TYPE_UNDECLARED'.
/**
  * MAV_ODID_CLASSIFICATION_TYPE
  * The classification type for the UA is undeclared.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSystem__CLASSIFICATION_TYPE_UNDECLARED = 0
};

/// Constant 'CLASSIFICATION_TYPE_EU'.
/**
  * The classification type for the UA follows EU (European Union) specifications.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDSystem__CLASSIFICATION_TYPE_EU = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'id_or_mac'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/OpenDroneIDSystem in the package mavros_msgs.
/**
  * Remote ID message - System
  * https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM
 */
typedef struct mavros_msgs__msg__OpenDroneIDSystem
{
  /// ]] (checksum: 518034079daae18afbc49769bc1a1031)
  std_msgs__msg__Header header;
  rosidl_runtime_c__String id_or_mac;
  uint8_t operator_location_type;
  uint8_t classification_type;
  int32_t operator_latitude;
  int32_t operator_longitude;
  uint16_t area_count;
  uint16_t area_radius;
  float area_ceiling;
  float area_floor;
  uint8_t category_eu;
  uint8_t class_eu;
  float operator_altitude_geo;
} mavros_msgs__msg__OpenDroneIDSystem;

// Struct for a sequence of mavros_msgs__msg__OpenDroneIDSystem.
typedef struct mavros_msgs__msg__OpenDroneIDSystem__Sequence
{
  mavros_msgs__msg__OpenDroneIDSystem * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OpenDroneIDSystem__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__STRUCT_H_
