// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/MagnetometerReporter.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__MagnetometerReporter __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__MagnetometerReporter __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MagnetometerReporter_
{
  using Type = MagnetometerReporter_<ContainerAllocator>;

  explicit MagnetometerReporter_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->report = 0;
      this->confidence = 0.0f;
      this->compass_id = 0;
      this->cal_mask = 0;
      this->cal_status = 0;
      this->autosaved = 0;
      this->fitness = 0.0f;
      this->ofs_x = 0.0f;
      this->ofs_y = 0.0f;
      this->ofs_z = 0.0f;
      this->diag_x = 0.0f;
      this->diag_y = 0.0f;
      this->diag_z = 0.0f;
      this->offdiag_x = 0.0f;
      this->offdiag_y = 0.0f;
      this->offdiag_z = 0.0f;
      this->orientation_confidence = 0.0f;
      this->old_orientation = 0;
      this->new_orientation = 0;
      this->scale_factor = 0.0f;
    }
  }

  explicit MagnetometerReporter_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->report = 0;
      this->confidence = 0.0f;
      this->compass_id = 0;
      this->cal_mask = 0;
      this->cal_status = 0;
      this->autosaved = 0;
      this->fitness = 0.0f;
      this->ofs_x = 0.0f;
      this->ofs_y = 0.0f;
      this->ofs_z = 0.0f;
      this->diag_x = 0.0f;
      this->diag_y = 0.0f;
      this->diag_z = 0.0f;
      this->offdiag_x = 0.0f;
      this->offdiag_y = 0.0f;
      this->offdiag_z = 0.0f;
      this->orientation_confidence = 0.0f;
      this->old_orientation = 0;
      this->new_orientation = 0;
      this->scale_factor = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _report_type =
    uint8_t;
  _report_type report;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _compass_id_type =
    uint8_t;
  _compass_id_type compass_id;
  using _cal_mask_type =
    uint8_t;
  _cal_mask_type cal_mask;
  using _cal_status_type =
    uint8_t;
  _cal_status_type cal_status;
  using _autosaved_type =
    uint8_t;
  _autosaved_type autosaved;
  using _fitness_type =
    float;
  _fitness_type fitness;
  using _ofs_x_type =
    float;
  _ofs_x_type ofs_x;
  using _ofs_y_type =
    float;
  _ofs_y_type ofs_y;
  using _ofs_z_type =
    float;
  _ofs_z_type ofs_z;
  using _diag_x_type =
    float;
  _diag_x_type diag_x;
  using _diag_y_type =
    float;
  _diag_y_type diag_y;
  using _diag_z_type =
    float;
  _diag_z_type diag_z;
  using _offdiag_x_type =
    float;
  _offdiag_x_type offdiag_x;
  using _offdiag_y_type =
    float;
  _offdiag_y_type offdiag_y;
  using _offdiag_z_type =
    float;
  _offdiag_z_type offdiag_z;
  using _orientation_confidence_type =
    float;
  _orientation_confidence_type orientation_confidence;
  using _old_orientation_type =
    uint8_t;
  _old_orientation_type old_orientation;
  using _new_orientation_type =
    uint8_t;
  _new_orientation_type new_orientation;
  using _scale_factor_type =
    float;
  _scale_factor_type scale_factor;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__report(
    const uint8_t & _arg)
  {
    this->report = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__compass_id(
    const uint8_t & _arg)
  {
    this->compass_id = _arg;
    return *this;
  }
  Type & set__cal_mask(
    const uint8_t & _arg)
  {
    this->cal_mask = _arg;
    return *this;
  }
  Type & set__cal_status(
    const uint8_t & _arg)
  {
    this->cal_status = _arg;
    return *this;
  }
  Type & set__autosaved(
    const uint8_t & _arg)
  {
    this->autosaved = _arg;
    return *this;
  }
  Type & set__fitness(
    const float & _arg)
  {
    this->fitness = _arg;
    return *this;
  }
  Type & set__ofs_x(
    const float & _arg)
  {
    this->ofs_x = _arg;
    return *this;
  }
  Type & set__ofs_y(
    const float & _arg)
  {
    this->ofs_y = _arg;
    return *this;
  }
  Type & set__ofs_z(
    const float & _arg)
  {
    this->ofs_z = _arg;
    return *this;
  }
  Type & set__diag_x(
    const float & _arg)
  {
    this->diag_x = _arg;
    return *this;
  }
  Type & set__diag_y(
    const float & _arg)
  {
    this->diag_y = _arg;
    return *this;
  }
  Type & set__diag_z(
    const float & _arg)
  {
    this->diag_z = _arg;
    return *this;
  }
  Type & set__offdiag_x(
    const float & _arg)
  {
    this->offdiag_x = _arg;
    return *this;
  }
  Type & set__offdiag_y(
    const float & _arg)
  {
    this->offdiag_y = _arg;
    return *this;
  }
  Type & set__offdiag_z(
    const float & _arg)
  {
    this->offdiag_z = _arg;
    return *this;
  }
  Type & set__orientation_confidence(
    const float & _arg)
  {
    this->orientation_confidence = _arg;
    return *this;
  }
  Type & set__old_orientation(
    const uint8_t & _arg)
  {
    this->old_orientation = _arg;
    return *this;
  }
  Type & set__new_orientation(
    const uint8_t & _arg)
  {
    this->new_orientation = _arg;
    return *this;
  }
  Type & set__scale_factor(
    const float & _arg)
  {
    this->scale_factor = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__MagnetometerReporter
    std::shared_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__MagnetometerReporter
    std::shared_ptr<mavros_msgs::msg::MagnetometerReporter_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MagnetometerReporter_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->report != other.report) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->compass_id != other.compass_id) {
      return false;
    }
    if (this->cal_mask != other.cal_mask) {
      return false;
    }
    if (this->cal_status != other.cal_status) {
      return false;
    }
    if (this->autosaved != other.autosaved) {
      return false;
    }
    if (this->fitness != other.fitness) {
      return false;
    }
    if (this->ofs_x != other.ofs_x) {
      return false;
    }
    if (this->ofs_y != other.ofs_y) {
      return false;
    }
    if (this->ofs_z != other.ofs_z) {
      return false;
    }
    if (this->diag_x != other.diag_x) {
      return false;
    }
    if (this->diag_y != other.diag_y) {
      return false;
    }
    if (this->diag_z != other.diag_z) {
      return false;
    }
    if (this->offdiag_x != other.offdiag_x) {
      return false;
    }
    if (this->offdiag_y != other.offdiag_y) {
      return false;
    }
    if (this->offdiag_z != other.offdiag_z) {
      return false;
    }
    if (this->orientation_confidence != other.orientation_confidence) {
      return false;
    }
    if (this->old_orientation != other.old_orientation) {
      return false;
    }
    if (this->new_orientation != other.new_orientation) {
      return false;
    }
    if (this->scale_factor != other.scale_factor) {
      return false;
    }
    return true;
  }
  bool operator!=(const MagnetometerReporter_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MagnetometerReporter_

// alias to use template instance with default allocator
using MagnetometerReporter =
  mavros_msgs::msg::MagnetometerReporter_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__STRUCT_HPP_
