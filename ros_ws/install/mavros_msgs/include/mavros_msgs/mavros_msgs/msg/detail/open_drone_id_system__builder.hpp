// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDSystem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/open_drone_id_system__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIDSystem_operator_altitude_geo
{
public:
  explicit Init_OpenDroneIDSystem_operator_altitude_geo(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OpenDroneIDSystem operator_altitude_geo(::mavros_msgs::msg::OpenDroneIDSystem::_operator_altitude_geo_type arg)
  {
    msg_.operator_altitude_geo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_class_eu
{
public:
  explicit Init_OpenDroneIDSystem_class_eu(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_operator_altitude_geo class_eu(::mavros_msgs::msg::OpenDroneIDSystem::_class_eu_type arg)
  {
    msg_.class_eu = std::move(arg);
    return Init_OpenDroneIDSystem_operator_altitude_geo(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_category_eu
{
public:
  explicit Init_OpenDroneIDSystem_category_eu(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_class_eu category_eu(::mavros_msgs::msg::OpenDroneIDSystem::_category_eu_type arg)
  {
    msg_.category_eu = std::move(arg);
    return Init_OpenDroneIDSystem_class_eu(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_area_floor
{
public:
  explicit Init_OpenDroneIDSystem_area_floor(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_category_eu area_floor(::mavros_msgs::msg::OpenDroneIDSystem::_area_floor_type arg)
  {
    msg_.area_floor = std::move(arg);
    return Init_OpenDroneIDSystem_category_eu(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_area_ceiling
{
public:
  explicit Init_OpenDroneIDSystem_area_ceiling(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_area_floor area_ceiling(::mavros_msgs::msg::OpenDroneIDSystem::_area_ceiling_type arg)
  {
    msg_.area_ceiling = std::move(arg);
    return Init_OpenDroneIDSystem_area_floor(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_area_radius
{
public:
  explicit Init_OpenDroneIDSystem_area_radius(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_area_ceiling area_radius(::mavros_msgs::msg::OpenDroneIDSystem::_area_radius_type arg)
  {
    msg_.area_radius = std::move(arg);
    return Init_OpenDroneIDSystem_area_ceiling(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_area_count
{
public:
  explicit Init_OpenDroneIDSystem_area_count(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_area_radius area_count(::mavros_msgs::msg::OpenDroneIDSystem::_area_count_type arg)
  {
    msg_.area_count = std::move(arg);
    return Init_OpenDroneIDSystem_area_radius(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_operator_longitude
{
public:
  explicit Init_OpenDroneIDSystem_operator_longitude(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_area_count operator_longitude(::mavros_msgs::msg::OpenDroneIDSystem::_operator_longitude_type arg)
  {
    msg_.operator_longitude = std::move(arg);
    return Init_OpenDroneIDSystem_area_count(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_operator_latitude
{
public:
  explicit Init_OpenDroneIDSystem_operator_latitude(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_operator_longitude operator_latitude(::mavros_msgs::msg::OpenDroneIDSystem::_operator_latitude_type arg)
  {
    msg_.operator_latitude = std::move(arg);
    return Init_OpenDroneIDSystem_operator_longitude(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_classification_type
{
public:
  explicit Init_OpenDroneIDSystem_classification_type(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_operator_latitude classification_type(::mavros_msgs::msg::OpenDroneIDSystem::_classification_type_type arg)
  {
    msg_.classification_type = std::move(arg);
    return Init_OpenDroneIDSystem_operator_latitude(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_operator_location_type
{
public:
  explicit Init_OpenDroneIDSystem_operator_location_type(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_classification_type operator_location_type(::mavros_msgs::msg::OpenDroneIDSystem::_operator_location_type_type arg)
  {
    msg_.operator_location_type = std::move(arg);
    return Init_OpenDroneIDSystem_classification_type(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_id_or_mac
{
public:
  explicit Init_OpenDroneIDSystem_id_or_mac(::mavros_msgs::msg::OpenDroneIDSystem & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystem_operator_location_type id_or_mac(::mavros_msgs::msg::OpenDroneIDSystem::_id_or_mac_type arg)
  {
    msg_.id_or_mac = std::move(arg);
    return Init_OpenDroneIDSystem_operator_location_type(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

class Init_OpenDroneIDSystem_header
{
public:
  Init_OpenDroneIDSystem_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIDSystem_id_or_mac header(::mavros_msgs::msg::OpenDroneIDSystem::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpenDroneIDSystem_id_or_mac(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystem msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OpenDroneIDSystem>()
{
  return mavros_msgs::msg::builder::Init_OpenDroneIDSystem_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM__BUILDER_HPP_
