// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDOperatorID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/open_drone_id_operator_id__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OpenDroneIDOperatorID & msg,
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

  // member: operator_id_type
  {
    out << "operator_id_type: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id_type, out);
    out << ", ";
  }

  // member: operator_id
  {
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OpenDroneIDOperatorID & msg,
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

  // member: operator_id_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_id_type: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id_type, out);
    out << "\n";
  }

  // member: operator_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OpenDroneIDOperatorID & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::OpenDroneIDOperatorID & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::OpenDroneIDOperatorID & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::OpenDroneIDOperatorID>()
{
  return "mavros_msgs::msg::OpenDroneIDOperatorID";
}

template<>
inline const char * name<mavros_msgs::msg::OpenDroneIDOperatorID>()
{
  return "mavros_msgs/msg/OpenDroneIDOperatorID";
}

template<>
struct has_fixed_size<mavros_msgs::msg::OpenDroneIDOperatorID>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::OpenDroneIDOperatorID>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::OpenDroneIDOperatorID>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__TRAITS_HPP_
