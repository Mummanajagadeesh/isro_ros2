// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/ManualControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__ManualControl __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__ManualControl __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ManualControl_
{
  using Type = ManualControl_<ContainerAllocator>;

  explicit ManualControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->r = 0.0f;
      this->buttons = 0;
      this->buttons2 = 0;
      this->enabled_extensions = 0;
      this->s = 0.0f;
      this->t = 0.0f;
      this->aux1 = 0.0f;
      this->aux2 = 0.0f;
      this->aux3 = 0.0f;
      this->aux4 = 0.0f;
      this->aux5 = 0.0f;
      this->aux6 = 0.0f;
    }
  }

  explicit ManualControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->r = 0.0f;
      this->buttons = 0;
      this->buttons2 = 0;
      this->enabled_extensions = 0;
      this->s = 0.0f;
      this->t = 0.0f;
      this->aux1 = 0.0f;
      this->aux2 = 0.0f;
      this->aux3 = 0.0f;
      this->aux4 = 0.0f;
      this->aux5 = 0.0f;
      this->aux6 = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _r_type =
    float;
  _r_type r;
  using _buttons_type =
    uint16_t;
  _buttons_type buttons;
  using _buttons2_type =
    uint16_t;
  _buttons2_type buttons2;
  using _enabled_extensions_type =
    uint8_t;
  _enabled_extensions_type enabled_extensions;
  using _s_type =
    float;
  _s_type s;
  using _t_type =
    float;
  _t_type t;
  using _aux1_type =
    float;
  _aux1_type aux1;
  using _aux2_type =
    float;
  _aux2_type aux2;
  using _aux3_type =
    float;
  _aux3_type aux3;
  using _aux4_type =
    float;
  _aux4_type aux4;
  using _aux5_type =
    float;
  _aux5_type aux5;
  using _aux6_type =
    float;
  _aux6_type aux6;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__r(
    const float & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__buttons(
    const uint16_t & _arg)
  {
    this->buttons = _arg;
    return *this;
  }
  Type & set__buttons2(
    const uint16_t & _arg)
  {
    this->buttons2 = _arg;
    return *this;
  }
  Type & set__enabled_extensions(
    const uint8_t & _arg)
  {
    this->enabled_extensions = _arg;
    return *this;
  }
  Type & set__s(
    const float & _arg)
  {
    this->s = _arg;
    return *this;
  }
  Type & set__t(
    const float & _arg)
  {
    this->t = _arg;
    return *this;
  }
  Type & set__aux1(
    const float & _arg)
  {
    this->aux1 = _arg;
    return *this;
  }
  Type & set__aux2(
    const float & _arg)
  {
    this->aux2 = _arg;
    return *this;
  }
  Type & set__aux3(
    const float & _arg)
  {
    this->aux3 = _arg;
    return *this;
  }
  Type & set__aux4(
    const float & _arg)
  {
    this->aux4 = _arg;
    return *this;
  }
  Type & set__aux5(
    const float & _arg)
  {
    this->aux5 = _arg;
    return *this;
  }
  Type & set__aux6(
    const float & _arg)
  {
    this->aux6 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::ManualControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::ManualControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ManualControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ManualControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__ManualControl
    std::shared_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__ManualControl
    std::shared_ptr<mavros_msgs::msg::ManualControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManualControl_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->buttons != other.buttons) {
      return false;
    }
    if (this->buttons2 != other.buttons2) {
      return false;
    }
    if (this->enabled_extensions != other.enabled_extensions) {
      return false;
    }
    if (this->s != other.s) {
      return false;
    }
    if (this->t != other.t) {
      return false;
    }
    if (this->aux1 != other.aux1) {
      return false;
    }
    if (this->aux2 != other.aux2) {
      return false;
    }
    if (this->aux3 != other.aux3) {
      return false;
    }
    if (this->aux4 != other.aux4) {
      return false;
    }
    if (this->aux5 != other.aux5) {
      return false;
    }
    if (this->aux6 != other.aux6) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManualControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManualControl_

// alias to use template instance with default allocator
using ManualControl =
  mavros_msgs::msg::ManualControl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__STRUCT_HPP_
