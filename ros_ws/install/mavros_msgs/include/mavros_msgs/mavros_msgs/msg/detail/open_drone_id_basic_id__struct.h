// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OpenDroneIDBasicID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ID_TYPE_NONE'.
/**
  * [[[cog:
  * import mavros_cog
  * mavros_cog.idl_decl_enum('MAV_ODID_ID_TYPE', 'ID_TYPE_')
  * mavros_cog.idl_decl_enum('MAV_ODID_UA_TYPE', 'UA_TYPE_')
  * ]]]
  * MAV_ODID_ID_TYPE
  * No type defined.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__ID_TYPE_NONE = 0
};

/// Constant 'ID_TYPE_SERIAL_NUMBER'.
/**
  * Manufacturer Serial Number (ANSI/CTA-2063 format).
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__ID_TYPE_SERIAL_NUMBER = 1
};

/// Constant 'ID_TYPE_CAA_REGISTRATION_ID'.
/**
  * CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID].
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__ID_TYPE_CAA_REGISTRATION_ID = 2
};

/// Constant 'ID_TYPE_UTM_ASSIGNED_UUID'.
/**
  * UTM (Unmanned Traffic Management) assigned UUID (RFC4122).
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__ID_TYPE_UTM_ASSIGNED_UUID = 3
};

/// Constant 'ID_TYPE_SPECIFIC_SESSION_ID'.
/**
  * A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of uas_id and these type values are managed by ICAO.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__ID_TYPE_SPECIFIC_SESSION_ID = 4
};

/// Constant 'UA_TYPE_NONE'.
/**
  * MAV_ODID_UA_TYPE
  * No UA (Unmanned Aircraft) type defined.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_NONE = 0
};

/// Constant 'UA_TYPE_AEROPLANE'.
/**
  * Aeroplane/Airplane. Fixed wing.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_AEROPLANE = 1
};

/// Constant 'UA_TYPE_HELICOPTER_OR_MULTIROTOR'.
/**
  * Helicopter or multirotor.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_HELICOPTER_OR_MULTIROTOR = 2
};

/// Constant 'UA_TYPE_GYROPLANE'.
/**
  * Gyroplane.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_GYROPLANE = 3
};

/// Constant 'UA_TYPE_HYBRID_LIFT'.
/**
  * VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_HYBRID_LIFT = 4
};

/// Constant 'UA_TYPE_ORNITHOPTER'.
/**
  * Ornithopter.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_ORNITHOPTER = 5
};

/// Constant 'UA_TYPE_GLIDER'.
/**
  * Glider.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_GLIDER = 6
};

/// Constant 'UA_TYPE_KITE'.
/**
  * Kite.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_KITE = 7
};

/// Constant 'UA_TYPE_FREE_BALLOON'.
/**
  * Free Balloon.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_FREE_BALLOON = 8
};

/// Constant 'UA_TYPE_CAPTIVE_BALLOON'.
/**
  * Captive Balloon.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_CAPTIVE_BALLOON = 9
};

/// Constant 'UA_TYPE_AIRSHIP'.
/**
  * Airship. E.g. a blimp.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_AIRSHIP = 10
};

/// Constant 'UA_TYPE_FREE_FALL_PARACHUTE'.
/**
  * Free Fall/Parachute (unpowered).
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_FREE_FALL_PARACHUTE = 11
};

/// Constant 'UA_TYPE_ROCKET'.
/**
  * Rocket.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_ROCKET = 12
};

/// Constant 'UA_TYPE_TETHERED_POWERED_AIRCRAFT'.
/**
  * Tethered powered aircraft.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_TETHERED_POWERED_AIRCRAFT = 13
};

/// Constant 'UA_TYPE_GROUND_OBSTACLE'.
/**
  * Ground Obstacle.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_GROUND_OBSTACLE = 14
};

/// Constant 'UA_TYPE_OTHER'.
/**
  * Other type of aircraft not listed earlier.
 */
enum
{
  mavros_msgs__msg__OpenDroneIDBasicID__UA_TYPE_OTHER = 15
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'id_or_mac'
// Member 'uas_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/OpenDroneIDBasicID in the package mavros_msgs.
/**
  * Remote ID message - Basic ID
  * https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID
 */
typedef struct mavros_msgs__msg__OpenDroneIDBasicID
{
  /// ]] (checksum: 5c8d15e3412f1859012762424293d945)
  std_msgs__msg__Header header;
  rosidl_runtime_c__String id_or_mac;
  uint8_t id_type;
  uint8_t ua_type;
  rosidl_runtime_c__String uas_id;
} mavros_msgs__msg__OpenDroneIDBasicID;

// Struct for a sequence of mavros_msgs__msg__OpenDroneIDBasicID.
typedef struct mavros_msgs__msg__OpenDroneIDBasicID__Sequence
{
  mavros_msgs__msg__OpenDroneIDBasicID * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OpenDroneIDBasicID__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__STRUCT_H_
