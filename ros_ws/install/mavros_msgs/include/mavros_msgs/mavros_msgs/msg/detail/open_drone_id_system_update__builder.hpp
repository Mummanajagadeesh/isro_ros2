// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDSystemUpdate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/open_drone_id_system_update__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIDSystemUpdate_operator_altitude_geo
{
public:
  explicit Init_OpenDroneIDSystemUpdate_operator_altitude_geo(::mavros_msgs::msg::OpenDroneIDSystemUpdate & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OpenDroneIDSystemUpdate operator_altitude_geo(::mavros_msgs::msg::OpenDroneIDSystemUpdate::_operator_altitude_geo_type arg)
  {
    msg_.operator_altitude_geo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystemUpdate msg_;
};

class Init_OpenDroneIDSystemUpdate_operator_longitude
{
public:
  explicit Init_OpenDroneIDSystemUpdate_operator_longitude(::mavros_msgs::msg::OpenDroneIDSystemUpdate & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystemUpdate_operator_altitude_geo operator_longitude(::mavros_msgs::msg::OpenDroneIDSystemUpdate::_operator_longitude_type arg)
  {
    msg_.operator_longitude = std::move(arg);
    return Init_OpenDroneIDSystemUpdate_operator_altitude_geo(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystemUpdate msg_;
};

class Init_OpenDroneIDSystemUpdate_operator_latitude
{
public:
  explicit Init_OpenDroneIDSystemUpdate_operator_latitude(::mavros_msgs::msg::OpenDroneIDSystemUpdate & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSystemUpdate_operator_longitude operator_latitude(::mavros_msgs::msg::OpenDroneIDSystemUpdate::_operator_latitude_type arg)
  {
    msg_.operator_latitude = std::move(arg);
    return Init_OpenDroneIDSystemUpdate_operator_longitude(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystemUpdate msg_;
};

class Init_OpenDroneIDSystemUpdate_header
{
public:
  Init_OpenDroneIDSystemUpdate_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIDSystemUpdate_operator_latitude header(::mavros_msgs::msg::OpenDroneIDSystemUpdate::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpenDroneIDSystemUpdate_operator_latitude(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSystemUpdate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OpenDroneIDSystemUpdate>()
{
  return mavros_msgs::msg::builder::Init_OpenDroneIDSystemUpdate_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SYSTEM_UPDATE__BUILDER_HPP_
