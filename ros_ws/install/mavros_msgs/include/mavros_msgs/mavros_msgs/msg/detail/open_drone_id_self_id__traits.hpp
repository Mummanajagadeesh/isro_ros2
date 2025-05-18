// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDSelfID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/open_drone_id_self_id__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OpenDroneIDSelfID & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id_or_mac
  {
    out << "id_or_mac: ";
    rosidl_generator_traits::value_to_yaml(msg.id_or_mac, out);
    out << ", ";
  }

  // member: description_type
  {
    out << "description_type: ";
    rosidl_generator_traits::value_to_yaml(msg.description_type, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OpenDroneIDSelfID & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: id_or_mac
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id_or_mac: ";
    rosidl_generator_traits::value_to_yaml(msg.id_or_mac, out);
    out << "\n";
  }

  // member: description_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description_type: ";
    rosidl_generator_traits::value_to_yaml(msg.description_type, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OpenDroneIDSelfID & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::msg::OpenDroneIDSelfID & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::OpenDroneIDSelfID & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::OpenDroneIDSelfID>()
{
  return "mavros_msgs::msg::OpenDroneIDSelfID";
}

template<>
inline const char * name<mavros_msgs::msg::OpenDroneIDSelfID>()
{
  return "mavros_msgs/msg/OpenDroneIDSelfID";
}

template<>
struct has_fixed_size<mavros_msgs::msg::OpenDroneIDSelfID>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::OpenDroneIDSelfID>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::OpenDroneIDSelfID>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__TRAITS_HPP_
