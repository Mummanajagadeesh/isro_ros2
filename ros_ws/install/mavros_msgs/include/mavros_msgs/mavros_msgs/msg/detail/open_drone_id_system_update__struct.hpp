// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDSystemUpdate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__OpenDroneIDSystemUpdate __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__OpenDroneIDSystemUpdate __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpenDroneIDSystemUpdate_
{
  using Type = OpenDroneIDSystemUpdate_<ContainerAllocator>;

  explicit OpenDroneIDSystemUpdate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operator_latitude = 0l;
      this->operator_longitude = 0l;
      this->operator_altitude_geo = 0.0f;
    }
  }

  explicit OpenDroneIDSystemUpdate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operator_latitude = 0l;
      this->operator_longitude = 0l;
      this->operator_altitude_geo = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _operator_latitude_type =
    int32_t;
  _operator_latitude_type operator_latitude;
  using _operator_longitude_type =
    int32_t;
  _operator_longitude_type operator_longitude;
  using _operator_altitude_geo_type =
    float;
  _operator_altitude_geo_type operator_altitude_geo;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__operator_latitude(
    const int32_t & _arg)
  {
    this->operator_latitude = _arg;
    return *this;
  }
  Type & set__operator_longitude(
    const int32_t & _arg)
  {
    this->operator_longitude = _arg;
    return *this;
  }
  Type & set__operator_altitude_geo(
    const float & _arg)
  {
    this->operator_altitude_geo = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__OpenDroneIDSystemUpdate
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__OpenDroneIDSystemUpdate
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSystemUpdate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpenDroneIDSystemUpdate_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->operator_latitude != other.operator_latitude) {
      return false;
    }
    if (this->operator_longitude != other.operator_longitude) {
      return false;
    }
    if (this->operator_altitude_geo != other.operator_altitude_geo) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpenDroneIDSystemUpdate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpenDroneIDSystemUpdate_

// alias to use template instance with default allocator
using OpenDroneIDSystemUpdate =
  mavros_msgs::msg::OpenDroneIDSystemUpdate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__STRUCT_HPP_
