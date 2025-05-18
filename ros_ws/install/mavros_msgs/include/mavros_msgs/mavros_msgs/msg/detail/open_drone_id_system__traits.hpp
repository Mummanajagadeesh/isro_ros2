// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDSystem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/open_drone_id_system__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OpenDroneIDSystem & msg,
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

  // member: operator_location_type
  {
    out << "operator_location_type: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_location_type, out);
    out << ", ";
  }

  // member: classification_type
  {
    out << "classification_type: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_type, out);
    out << ", ";
  }

  // member: operator_latitude
  {
    out << "operator_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_latitude, out);
    out << ", ";
  }

  // member: operator_longitude
  {
    out << "operator_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_longitude, out);
    out << ", ";
  }

  // member: area_count
  {
    out << "area_count: ";
    rosidl_generator_traits::value_to_yaml(msg.area_count, out);
    out << ", ";
  }

  // member: area_radius
  {
    out << "area_radius: ";
    rosidl_generator_traits::value_to_yaml(msg.area_radius, out);
    out << ", ";
  }

  // member: area_ceiling
  {
    out << "area_ceiling: ";
    rosidl_generator_traits::value_to_yaml(msg.area_ceiling, out);
    out << ", ";
  }

  // member: area_floor
  {
    out << "area_floor: ";
    rosidl_generator_traits::value_to_yaml(msg.area_floor, out);
    out << ", ";
  }

  // member: category_eu
  {
    out << "category_eu: ";
    rosidl_generator_traits::value_to_yaml(msg.category_eu, out);
    out << ", ";
  }

  // member: class_eu
  {
    out << "class_eu: ";
    rosidl_generator_traits::value_to_yaml(msg.class_eu, out);
    out << ", ";
  }

  // member: operator_altitude_geo
  {
    out << "operator_altitude_geo: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_altitude_geo, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OpenDroneIDSystem & msg,
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

  // member: operator_location_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_location_type: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_location_type, out);
    out << "\n";
  }

  // member: classification_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "classification_type: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_type, out);
    out << "\n";
  }

  // member: operator_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_latitude, out);
    out << "\n";
  }

  // member: operator_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_longitude, out);
    out << "\n";
  }

  // member: area_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area_count: ";
    rosidl_generator_traits::value_to_yaml(msg.area_count, out);
    out << "\n";
  }

  // member: area_radius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area_radius: ";
    rosidl_generator_traits::value_to_yaml(msg.area_radius, out);
    out << "\n";
  }

  // member: area_ceiling
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area_ceiling: ";
    rosidl_generator_traits::value_to_yaml(msg.area_ceiling, out);
    out << "\n";
  }

  // member: area_floor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area_floor: ";
    rosidl_generator_traits::value_to_yaml(msg.area_floor, out);
    out << "\n";
  }

  // member: category_eu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "category_eu: ";
    rosidl_generator_traits::value_to_yaml(msg.category_eu, out);
    out << "\n";
  }

  // member: class_eu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_eu: ";
    rosidl_generator_traits::value_to_yaml(msg.class_eu, out);
    out << "\n";
  }

  // member: operator_altitude_geo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_altitude_geo: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_altitude_geo, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OpenDroneIDSystem & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::OpenDroneIDSystem & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::OpenDroneIDSystem & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::OpenDroneIDSystem>()
{
  return "mavros_msgs::msg::OpenDroneIDSystem";
}

template<>
inline const char * name<mavros_msgs::msg::OpenDroneIDSystem>()
{
  return "mavros_msgs/msg/OpenDroneIDSystem";
}

template<>
struct has_fixed_size<mavros_msgs::msg::OpenDroneIDSystem>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::OpenDroneIDSystem>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::OpenDroneIDSystem>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__TRAITS_HPP_
