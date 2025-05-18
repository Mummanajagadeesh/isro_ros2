// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/MagnetometerReporter.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/magnetometer_reporter__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_MagnetometerReporter_scale_factor
{
public:
  explicit Init_MagnetometerReporter_scale_factor(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::MagnetometerReporter scale_factor(::mavros_msgs::msg::MagnetometerReporter::_scale_factor_type arg)
  {
    msg_.scale_factor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_new_orientation
{
public:
  explicit Init_MagnetometerReporter_new_orientation(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_scale_factor new_orientation(::mavros_msgs::msg::MagnetometerReporter::_new_orientation_type arg)
  {
    msg_.new_orientation = std::move(arg);
    return Init_MagnetometerReporter_scale_factor(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_old_orientation
{
public:
  explicit Init_MagnetometerReporter_old_orientation(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_new_orientation old_orientation(::mavros_msgs::msg::MagnetometerReporter::_old_orientation_type arg)
  {
    msg_.old_orientation = std::move(arg);
    return Init_MagnetometerReporter_new_orientation(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_orientation_confidence
{
public:
  explicit Init_MagnetometerReporter_orientation_confidence(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_old_orientation orientation_confidence(::mavros_msgs::msg::MagnetometerReporter::_orientation_confidence_type arg)
  {
    msg_.orientation_confidence = std::move(arg);
    return Init_MagnetometerReporter_old_orientation(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_offdiag_z
{
public:
  explicit Init_MagnetometerReporter_offdiag_z(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_orientation_confidence offdiag_z(::mavros_msgs::msg::MagnetometerReporter::_offdiag_z_type arg)
  {
    msg_.offdiag_z = std::move(arg);
    return Init_MagnetometerReporter_orientation_confidence(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_offdiag_y
{
public:
  explicit Init_MagnetometerReporter_offdiag_y(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_offdiag_z offdiag_y(::mavros_msgs::msg::MagnetometerReporter::_offdiag_y_type arg)
  {
    msg_.offdiag_y = std::move(arg);
    return Init_MagnetometerReporter_offdiag_z(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_offdiag_x
{
public:
  explicit Init_MagnetometerReporter_offdiag_x(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_offdiag_y offdiag_x(::mavros_msgs::msg::MagnetometerReporter::_offdiag_x_type arg)
  {
    msg_.offdiag_x = std::move(arg);
    return Init_MagnetometerReporter_offdiag_y(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_diag_z
{
public:
  explicit Init_MagnetometerReporter_diag_z(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_offdiag_x diag_z(::mavros_msgs::msg::MagnetometerReporter::_diag_z_type arg)
  {
    msg_.diag_z = std::move(arg);
    return Init_MagnetometerReporter_offdiag_x(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_diag_y
{
public:
  explicit Init_MagnetometerReporter_diag_y(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_diag_z diag_y(::mavros_msgs::msg::MagnetometerReporter::_diag_y_type arg)
  {
    msg_.diag_y = std::move(arg);
    return Init_MagnetometerReporter_diag_z(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_diag_x
{
public:
  explicit Init_MagnetometerReporter_diag_x(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_diag_y diag_x(::mavros_msgs::msg::MagnetometerReporter::_diag_x_type arg)
  {
    msg_.diag_x = std::move(arg);
    return Init_MagnetometerReporter_diag_y(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_ofs_z
{
public:
  explicit Init_MagnetometerReporter_ofs_z(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_diag_x ofs_z(::mavros_msgs::msg::MagnetometerReporter::_ofs_z_type arg)
  {
    msg_.ofs_z = std::move(arg);
    return Init_MagnetometerReporter_diag_x(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_ofs_y
{
public:
  explicit Init_MagnetometerReporter_ofs_y(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_ofs_z ofs_y(::mavros_msgs::msg::MagnetometerReporter::_ofs_y_type arg)
  {
    msg_.ofs_y = std::move(arg);
    return Init_MagnetometerReporter_ofs_z(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_ofs_x
{
public:
  explicit Init_MagnetometerReporter_ofs_x(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_ofs_y ofs_x(::mavros_msgs::msg::MagnetometerReporter::_ofs_x_type arg)
  {
    msg_.ofs_x = std::move(arg);
    return Init_MagnetometerReporter_ofs_y(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_fitness
{
public:
  explicit Init_MagnetometerReporter_fitness(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_ofs_x fitness(::mavros_msgs::msg::MagnetometerReporter::_fitness_type arg)
  {
    msg_.fitness = std::move(arg);
    return Init_MagnetometerReporter_ofs_x(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_autosaved
{
public:
  explicit Init_MagnetometerReporter_autosaved(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_fitness autosaved(::mavros_msgs::msg::MagnetometerReporter::_autosaved_type arg)
  {
    msg_.autosaved = std::move(arg);
    return Init_MagnetometerReporter_fitness(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_cal_status
{
public:
  explicit Init_MagnetometerReporter_cal_status(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_autosaved cal_status(::mavros_msgs::msg::MagnetometerReporter::_cal_status_type arg)
  {
    msg_.cal_status = std::move(arg);
    return Init_MagnetometerReporter_autosaved(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_cal_mask
{
public:
  explicit Init_MagnetometerReporter_cal_mask(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_cal_status cal_mask(::mavros_msgs::msg::MagnetometerReporter::_cal_mask_type arg)
  {
    msg_.cal_mask = std::move(arg);
    return Init_MagnetometerReporter_cal_status(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_compass_id
{
public:
  explicit Init_MagnetometerReporter_compass_id(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_cal_mask compass_id(::mavros_msgs::msg::MagnetometerReporter::_compass_id_type arg)
  {
    msg_.compass_id = std::move(arg);
    return Init_MagnetometerReporter_cal_mask(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_confidence
{
public:
  explicit Init_MagnetometerReporter_confidence(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_compass_id confidence(::mavros_msgs::msg::MagnetometerReporter::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_MagnetometerReporter_compass_id(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_report
{
public:
  explicit Init_MagnetometerReporter_report(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_confidence report(::mavros_msgs::msg::MagnetometerReporter::_report_type arg)
  {
    msg_.report = std::move(arg);
    return Init_MagnetometerReporter_confidence(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_header
{
public:
  Init_MagnetometerReporter_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MagnetometerReporter_report header(::mavros_msgs::msg::MagnetometerReporter::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MagnetometerReporter_report(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::MagnetometerReporter>()
{
  return mavros_msgs::msg::builder::Init_MagnetometerReporter_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__BUILDER_HPP_
