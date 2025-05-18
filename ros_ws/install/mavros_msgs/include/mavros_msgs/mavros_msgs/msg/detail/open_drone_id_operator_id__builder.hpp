// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDOperatorID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/open_drone_id_operator_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIDOperatorID_operator_id
{
public:
  explicit Init_OpenDroneIDOperatorID_operator_id(::mavros_msgs::msg::OpenDroneIDOperatorID & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OpenDroneIDOperatorID operator_id(::mavros_msgs::msg::OpenDroneIDOperatorID::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDOperatorID msg_;
};

class Init_OpenDroneIDOperatorID_operator_id_type
{
public:
  explicit Init_OpenDroneIDOperatorID_operator_id_type(::mavros_msgs::msg::OpenDroneIDOperatorID & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDOperatorID_operator_id operator_id_type(::mavros_msgs::msg::OpenDroneIDOperatorID::_operator_id_type_type arg)
  {
    msg_.operator_id_type = std::move(arg);
    return Init_OpenDroneIDOperatorID_operator_id(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDOperatorID msg_;
};

class Init_OpenDroneIDOperatorID_id_or_mac
{
public:
  explicit Init_OpenDroneIDOperatorID_id_or_mac(::mavros_msgs::msg::OpenDroneIDOperatorID & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDOperatorID_operator_id_type id_or_mac(::mavros_msgs::msg::OpenDroneIDOperatorID::_id_or_mac_type arg)
  {
    msg_.id_or_mac = std::move(arg);
    return Init_OpenDroneIDOperatorID_operator_id_type(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDOperatorID msg_;
};

class Init_OpenDroneIDOperatorID_header
{
public:
  Init_OpenDroneIDOperatorID_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIDOperatorID_id_or_mac header(::mavros_msgs::msg::OpenDroneIDOperatorID::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpenDroneIDOperatorID_id_or_mac(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDOperatorID msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OpenDroneIDOperatorID>()
{
  return mavros_msgs::msg::builder::Init_OpenDroneIDOperatorID_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__BUILDER_HPP_
