// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDSelfID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__OpenDroneIDSelfID __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__OpenDroneIDSelfID __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpenDroneIDSelfID_
{
  using Type = OpenDroneIDSelfID_<ContainerAllocator>;

  explicit OpenDroneIDSelfID_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_or_mac = "";
      this->description_type = 0;
      this->description = "";
    }
  }

  explicit OpenDroneIDSelfID_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    id_or_mac(_alloc),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_or_mac = "";
      this->description_type = 0;
      this->description = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_or_mac_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_or_mac_type id_or_mac;
  using _description_type_type =
    uint8_t;
  _description_type_type description_type;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

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
  Type & set__description_type(
    const uint8_t & _arg)
  {
    this->description_type = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t DESC_TYPE_TEXT =
    0u;
  static constexpr uint8_t DESC_TYPE_EMERGENCY =
    1u;
  static constexpr uint8_t DESC_TYPE_EXTENDED_STATUS =
    2u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__OpenDroneIDSelfID
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__OpenDroneIDSelfID
    std::shared_ptr<mavros_msgs::msg::OpenDroneIDSelfID_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpenDroneIDSelfID_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id_or_mac != other.id_or_mac) {
      return false;
    }
    if (this->description_type != other.description_type) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpenDroneIDSelfID_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpenDroneIDSelfID_

// alias to use template instance with default allocator
using OpenDroneIDSelfID =
  mavros_msgs::msg::OpenDroneIDSelfID_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDSelfID_<ContainerAllocator>::DESC_TYPE_TEXT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDSelfID_<ContainerAllocator>::DESC_TYPE_EMERGENCY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OpenDroneIDSelfID_<ContainerAllocator>::DESC_TYPE_EXTENDED_STATUS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__STRUCT_HPP_
