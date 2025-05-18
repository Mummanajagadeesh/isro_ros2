// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDBasicID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__OpenDroneIDBasicID __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__OpenDroneIDBasicID __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpenDroneIDBasicID_
{
  using Type = OpenDroneIDBasicID_<ContainerAllocator>;

  explicit OpenDroneIDBasicID_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_or_mac = "";
      this->id_type = 0;
      this->ua_type = 0;
      this->uas_id = "";
    }
  }

  explicit OpenDroneIDBasicID_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    id_or_mac(_alloc),
    uas_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_or_mac = "";
      this->id_type = 0;
      this->ua_type = 0;
      this->uas_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_or_mac_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_or_mac_type id_or_mac;
  using _id_type_type =
    uint8_t;
  _id_type_type id_type;
  using _ua_type_type =
    uint8_t;
  _ua_type_type ua_type;
  using _uas_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uas_id_type uas_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id_or_mac(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->id_or_mac = _arg;
    return *this;
  }
  Type & set__id_type(
    const uint8_t & _arg)
  {
    this->id_type = _arg;
    return *this;
  }
  Type & set__ua_type(
    const uint8_t & _arg)
  {
    this->ua_type = _arg;
    return *this;
  }
  Type & set__uas_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uas_id = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ID_TYPE_NONE =
    0u;
  static constexpr uint8_t ID_TYPE_SERIAL_NUMBER =
    1u;
  static constexpr uint8_t ID_TYPE_CAA_REGISTRATION_ID =
    2u;
  static constexpr uint8_t ID_TYPE_UTM_ASSIGNED_UUID =
    3u;
  static constexpr uint8_t ID_TYPE_SPECIFIC_SESSION_ID =
    4u;
  static constexpr uint8_t UA_TYPE_NONE =
    0u;
  static constexpr uint8_t UA_TYPE_AEROPLANE =
    1u;
  static constexpr uint8_t UA_TYPE_HELICOPTER_OR_MULTIROTOR =
    2u;
  static constexpr uint8_t UA_TYPE_GYROPLANE =
    3u;
  static constexpr uint8_t UA_TYPE_HYBRID_LIFT =
    4u;
  static constexpr uint8_t UA_TYPE_ORNITHOPTER =
    5u;
  static constexpr uint8_t UA_TYPE_GLIDER =
    6u;
  static constexpr uint8_t UA_TYPE_KITE =
    7u;
  static constexpr uint8_t UA_TYPE_FREE_BALLOON =
    8u;
  static constexpr uint8_t UA_TYPE_CAPTIVE_BALLOON =
    9u;
  static constexpr uint8_t UA_TYPE_AIRSHIP =
    10u;
  static constexpr uint8_t UA_TYPE_FREE_FALL_PARACHUTE =
    11u;
  static constexpr uint8_t UA_TYPE_ROCKET =
    12u;
  static constexpr uint8_t UA_TYPE_TETHERED_POWERED_AIRCRAFT =
    13u;
  static constexpr uint8_t UA_TYPE_GROUND_OBSTACLE =
    14u;
  static constexpr uint8_t UA_TYPE_OTHER =
    15u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__OpenDroneIDBasicID
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__OpenDroneIDBasicID
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDBasicID_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpenDroneIDBasicID_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id_or_mac != other.id_or_mac) {
      return false;
    }
    if (this->id_type != other.id_type) {
      return false;
    }
    if (this->ua_type != other.ua_type) {
      return false;
    }
    if (this->uas_id != other.uas_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpenDroneIDBasicID_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpenDroneIDBasicID_

// alias to use template instance with default allocator
using OpenDroneIDBasicID =
  mavros_msgs::msg::OpenDroneIDBasicID_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::ID_TYPE_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::ID_TYPE_SERIAL_NUMBER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::ID_TYPE_CAA_REGISTRATION_ID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::ID_TYPE_UTM_ASSIGNED_UUID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::ID_TYPE_SPECIFIC_SESSION_ID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_AEROPLANE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_HELICOPTER_OR_MULTIROTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_GYROPLANE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_HYBRID_LIFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_ORNITHOPTER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_GLIDER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_KITE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_FREE_BALLOON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_CAPTIVE_BALLOON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_AIRSHIP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_FREE_FALL_PARACHUTE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_ROCKET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_TETHERED_POWERED_AIRCRAFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_GROUND_OBSTACLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDBasicID_<ContainerAllocator>::UA_TYPE_OTHER;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__STRUCT_HPP_
