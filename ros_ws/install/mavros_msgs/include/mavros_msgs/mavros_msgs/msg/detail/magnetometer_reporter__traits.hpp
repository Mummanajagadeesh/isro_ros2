// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/MagnetometerReporter.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/magnetometer_reporter__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MagnetometerReporter & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: report
  {
    out << "report: ";
    rosidl_generator_traits::value_to_yaml(msg.report, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: compass_id
  {
    out << "compass_id: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_id, out);
    out << ", ";
  }

  // member: cal_mask
  {
    out << "cal_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_mask, out);
    out << ", ";
  }

  // member: cal_status
  {
    out << "cal_status: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_status, out);
    out << ", ";
  }

  // member: autosaved
  {
    out << "autosaved: ";
    rosidl_generator_traits::value_to_yaml(msg.autosaved, out);
    out << ", ";
  }

  // member: fitness
  {
    out << "fitness: ";
    rosidl_generator_traits::value_to_yaml(msg.fitness, out);
    out << ", ";
  }

  // member: ofs_x
  {
    out << "ofs_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ofs_x, out);
    out << ", ";
  }

  // member: ofs_y
  {
    out << "ofs_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ofs_y, out);
    out << ", ";
  }

  // member: ofs_z
  {
    out << "ofs_z: ";
    rosidl_generator_traits::value_to_yaml(msg.ofs_z, out);
    out << ", ";
  }

  // member: diag_x
  {
    out << "diag_x: ";
    rosidl_generator_traits::value_to_yaml(msg.diag_x, out);
    out << ", ";
  }

  // member: diag_y
  {
    out << "diag_y: ";
    rosidl_generator_traits::value_to_yaml(msg.diag_y, out);
    out << ", ";
  }

  // member: diag_z
  {
    out << "diag_z: ";
    rosidl_generator_traits::value_to_yaml(msg.diag_z, out);
    out << ", ";
  }

  // member: offdiag_x
  {
    out << "offdiag_x: ";
    rosidl_generator_traits::value_to_yaml(msg.offdiag_x, out);
    out << ", ";
  }

  // member: offdiag_y
  {
    out << "offdiag_y: ";
    rosidl_generator_traits::value_to_yaml(msg.offdiag_y, out);
    out << ", ";
  }

  // member: offdiag_z
  {
    out << "offdiag_z: ";
    rosidl_generator_traits::value_to_yaml(msg.offdiag_z, out);
    out << ", ";
  }

  // member: orientation_confidence
  {
    out << "orientation_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation_confidence, out);
    out << ", ";
  }

  // member: old_orientation
  {
    out << "old_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.old_orientation, out);
    out << ", ";
  }

  // member: new_orientation
  {
    out << "new_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.new_orientation, out);
    out << ", ";
  }

  // member: scale_factor
  {
    out << "scale_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.scale_factor, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MagnetometerReporter & msg,
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

  // member: report
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "report: ";
    rosidl_generator_traits::value_to_yaml(msg.report, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: compass_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compass_id: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_id, out);
    out << "\n";
  }

  // member: cal_mask
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cal_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_mask, out);
    out << "\n";
  }

  // member: cal_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cal_status: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_status, out);
    out << "\n";
  }

  // member: autosaved
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "autosaved: ";
    rosidl_generator_traits::value_to_yaml(msg.autosaved, out);
    out << "\n";
  }

  // member: fitness
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fitness: ";
    rosidl_generator_traits::value_to_yaml(msg.fitness, out);
    out << "\n";
  }

  // member: ofs_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ofs_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ofs_x, out);
    out << "\n";
  }

  // member: ofs_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ofs_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ofs_y, out);
    out << "\n";
  }

  // member: ofs_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ofs_z: ";
    rosidl_generator_traits::value_to_yaml(msg.ofs_z, out);
    out << "\n";
  }

  // member: diag_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diag_x: ";
    rosidl_generator_traits::value_to_yaml(msg.diag_x, out);
    out << "\n";
  }

  // member: diag_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diag_y: ";
    rosidl_generator_traits::value_to_yaml(msg.diag_y, out);
    out << "\n";
  }

  // member: diag_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diag_z: ";
    rosidl_generator_traits::value_to_yaml(msg.diag_z, out);
    out << "\n";
  }

  // member: offdiag_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offdiag_x: ";
    rosidl_generator_traits::value_to_yaml(msg.offdiag_x, out);
    out << "\n";
  }

  // member: offdiag_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offdiag_y: ";
    rosidl_generator_traits::value_to_yaml(msg.offdiag_y, out);
    out << "\n";
  }

  // member: offdiag_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offdiag_z: ";
    rosidl_generator_traits::value_to_yaml(msg.offdiag_z, out);
    out << "\n";
  }

  // member: orientation_confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation_confidence, out);
    out << "\n";
  }

  // member: old_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "old_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.old_orientation, out);
    out << "\n";
  }

  // member: new_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.new_orientation, out);
    out << "\n";
  }

  // member: scale_factor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scale_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.scale_factor, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MagnetometerReporter & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::MagnetometerReporter & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::MagnetometerReporter & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::MagnetometerReporter>()
{
  return "mavros_msgs::msg::MagnetometerReporter";
}

template<>
inline const char * name<mavros_msgs::msg::MagnetometerReporter>()
{
  return "mavros_msgs/msg/MagnetometerReporter";
}

template<>
struct has_fixed_size<mavros_msgs::msg::MagnetometerReporter>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::MagnetometerReporter>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::MagnetometerReporter>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__TRAITS_HPP_
