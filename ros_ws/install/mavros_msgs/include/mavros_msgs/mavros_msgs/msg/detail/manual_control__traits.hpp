// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/ManualControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/manual_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ManualControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: buttons
  {
    out << "buttons: ";
    rosidl_generator_traits::value_to_yaml(msg.buttons, out);
    out << ", ";
  }

  // member: buttons2
  {
    out << "buttons2: ";
    rosidl_generator_traits::value_to_yaml(msg.buttons2, out);
    out << ", ";
  }

  // member: enabled_extensions
  {
    out << "enabled_extensions: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled_extensions, out);
    out << ", ";
  }

  // member: s
  {
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
    out << ", ";
  }

  // member: t
  {
    out << "t: ";
    rosidl_generator_traits::value_to_yaml(msg.t, out);
    out << ", ";
  }

  // member: aux1
  {
    out << "aux1: ";
    rosidl_generator_traits::value_to_yaml(msg.aux1, out);
    out << ", ";
  }

  // member: aux2
  {
    out << "aux2: ";
    rosidl_generator_traits::value_to_yaml(msg.aux2, out);
    out << ", ";
  }

  // member: aux3
  {
    out << "aux3: ";
    rosidl_generator_traits::value_to_yaml(msg.aux3, out);
    out << ", ";
  }

  // member: aux4
  {
    out << "aux4: ";
    rosidl_generator_traits::value_to_yaml(msg.aux4, out);
    out << ", ";
  }

  // member: aux5
  {
    out << "aux5: ";
    rosidl_generator_traits::value_to_yaml(msg.aux5, out);
    out << ", ";
  }

  // member: aux6
  {
    out << "aux6: ";
    rosidl_generator_traits::value_to_yaml(msg.aux6, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManualControl & msg,
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

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: buttons
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "buttons: ";
    rosidl_generator_traits::value_to_yaml(msg.buttons, out);
    out << "\n";
  }

  // member: buttons2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "buttons2: ";
    rosidl_generator_traits::value_to_yaml(msg.buttons2, out);
    out << "\n";
  }

  // member: enabled_extensions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enabled_extensions: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled_extensions, out);
    out << "\n";
  }

  // member: s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
    out << "\n";
  }

  // member: t
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t: ";
    rosidl_generator_traits::value_to_yaml(msg.t, out);
    out << "\n";
  }

  // member: aux1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux1: ";
    rosidl_generator_traits::value_to_yaml(msg.aux1, out);
    out << "\n";
  }

  // member: aux2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux2: ";
    rosidl_generator_traits::value_to_yaml(msg.aux2, out);
    out << "\n";
  }

  // member: aux3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux3: ";
    rosidl_generator_traits::value_to_yaml(msg.aux3, out);
    out << "\n";
  }

  // member: aux4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux4: ";
    rosidl_generator_traits::value_to_yaml(msg.aux4, out);
    out << "\n";
  }

  // member: aux5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux5: ";
    rosidl_generator_traits::value_to_yaml(msg.aux5, out);
    out << "\n";
  }

  // member: aux6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux6: ";
    rosidl_generator_traits::value_to_yaml(msg.aux6, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManualControl & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::ManualControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::ManualControl & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::ManualControl>()
{
  return "mavros_msgs::msg::ManualControl";
}

template<>
inline const char * name<mavros_msgs::msg::ManualControl>()
{
  return "mavros_msgs/msg/ManualControl";
}

template<>
struct has_fixed_size<mavros_msgs::msg::ManualControl>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::ManualControl>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::ManualControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__TRAITS_HPP_
