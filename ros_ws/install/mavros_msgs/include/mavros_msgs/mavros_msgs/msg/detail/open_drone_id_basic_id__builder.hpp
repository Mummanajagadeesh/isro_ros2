// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OpenDroneIDBasicID.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/open_drone_id_basic_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIDBasicID_uas_id
{
public:
  explicit Init_OpenDroneIDBasicID_uas_id(::mavros_msgs::msg::OpenDroneIDBasicID & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OpenDroneIDBasicID uas_id(::mavros_msgs::msg::OpenDroneIDBasicID::_uas_id_type arg)
  {
    msg_.uas_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDBasicID msg_;
};

class Init_OpenDroneIDBasicID_ua_type
{
public:
  explicit Init_OpenDroneIDBasicID_ua_type(::mavros_msgs::msg::OpenDroneIDBasicID & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDBasicID_uas_id ua_type(::mavros_msgs::msg::OpenDroneIDBasicID::_ua_type_type arg)
  {
    msg_.ua_type = std::move(arg);
    return Init_OpenDroneIDBasicID_uas_id(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDBasicID msg_;
};

class Init_OpenDroneIDBasicID_id_type
{
public:
  explicit Init_OpenDroneIDBasicID_id_type(::mavros_msgs::msg::OpenDroneIDBasicID & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDBasicID_ua_type id_type(::mavros_msgs::msg::OpenDroneIDBasicID::_id_type_type arg)
  {
    msg_.id_type = std::move(arg);
    return Init_OpenDroneIDBasicID_ua_type(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDBasicID msg_;
};

class Init_OpenDroneIDBasicID_id_or_mac
{
public:
  explicit Init_OpenDroneIDBasicID_id_or_mac(::mavros_msgs::msg::OpenDroneIDBasicID & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIDBasicID_id_type id_or_mac(::mavros_msgs::msg::OpenDroneIDBasicID::_id_or_mac_type arg)
  {
    msg_.id_or_mac = std::move(arg);
    return Init_OpenDroneIDBasicID_id_type(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDBasicID msg_;
};

class Init_OpenDroneIDBasicID_header
{
public:
  Init_OpenDroneIDBasicID_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIDBasicID_id_or_mac header(::mavros_msgs::msg::OpenDroneIDBasicID::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpenDroneIDBasicID_id_or_mac(msg_);
  }

private:
  ::mavros_msgs::msg::OpenDroneIDBasicID msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OpenDroneIDBasicID>()
{
  return mavros_msgs::msg::builder::Init_OpenDroneIDBasicID_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPEN_DRONE_ID_BASIC_ID__BUILDER_HPP_
