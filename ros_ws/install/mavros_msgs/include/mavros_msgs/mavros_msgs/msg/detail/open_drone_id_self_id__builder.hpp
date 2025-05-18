// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDSelfID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/open_drone_id_self_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIDSelfID_description
{
public:
  explicit Init_OpenDroneIDSelfID_description(::mavros_msgs::msg::OpenDroneIDSelfID & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OpenDroneIDSelfID description(::mavros_msgs::msg::OpenDroneIDSelfID::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSelfID msg_;
};

class Init_OpenDroneIDSelfID_description_type
{
public:
  explicit Init_OpenDroneIDSelfID_description_type(::mavros_msgs::msg::OpenDroneIDSelfID & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSelfID_description description_type(::mavros_msgs::msg::OpenDroneIDSelfID::_description_type_type arg)
  {
    msg_.description_type = std::move(arg);
    return Init_OpenDroneIDSelfID_description(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSelfID msg_;
};

class Init_OpenDroneIDSelfID_id_or_mac
{
public:
  explicit Init_OpenDroneIDSelfID_id_or_mac(::mavros_msgs::msg::OpenDroneIDSelfID & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDSelfID_description_type id_or_mac(::mavros_msgs::msg::OpenDroneIDSelfID::_id_or_mac_type arg)
  {
    msg_.id_or_mac = std::move(arg);
    return Init_OpenDroneIDSelfID_description_type(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSelfID msg_;
};

class Init_OpenDroneIDSelfID_header
{
public:
  Init_OpenDroneIDSelfID_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIDSelfID_id_or_mac header(::mavros_msgs::msg::OpenDroneIDSelfID::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpenDroneIDSelfID_id_or_mac(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDSelfID msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OpenDroneIDSelfID>()
{
  return mavros_msgs::msg::builder::Init_OpenDroneIDSelfID_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__BUILDER_HPP_
