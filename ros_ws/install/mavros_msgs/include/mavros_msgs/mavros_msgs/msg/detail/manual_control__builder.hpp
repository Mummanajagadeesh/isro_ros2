// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ManualControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/manual_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ManualControl_aux6
{
public:
  explicit Init_ManualControl_aux6(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ManualControl aux6(::mavros_msgs::msg::ManualControl::_aux6_type arg)
  {
    msg_.aux6 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_aux5
{
public:
  explicit Init_ManualControl_aux5(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_aux6 aux5(::mavros_msgs::msg::ManualControl::_aux5_type arg)
  {
    msg_.aux5 = std::move(arg);
    return Init_ManualControl_aux6(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_aux4
{
public:
  explicit Init_ManualControl_aux4(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_aux5 aux4(::mavros_msgs::msg::ManualControl::_aux4_type arg)
  {
    msg_.aux4 = std::move(arg);
    return Init_ManualControl_aux5(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_aux3
{
public:
  explicit Init_ManualControl_aux3(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_aux4 aux3(::mavros_msgs::msg::ManualControl::_aux3_type arg)
  {
    msg_.aux3 = std::move(arg);
    return Init_ManualControl_aux4(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_aux2
{
public:
  explicit Init_ManualControl_aux2(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_aux3 aux2(::mavros_msgs::msg::ManualControl::_aux2_type arg)
  {
    msg_.aux2 = std::move(arg);
    return Init_ManualControl_aux3(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_aux1
{
public:
  explicit Init_ManualControl_aux1(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_aux2 aux1(::mavros_msgs::msg::ManualControl::_aux1_type arg)
  {
    msg_.aux1 = std::move(arg);
    return Init_ManualControl_aux2(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_t
{
public:
  explicit Init_ManualControl_t(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_aux1 t(::mavros_msgs::msg::ManualControl::_t_type arg)
  {
    msg_.t = std::move(arg);
    return Init_ManualControl_aux1(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_s
{
public:
  explicit Init_ManualControl_s(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_t s(::mavros_msgs::msg::ManualControl::_s_type arg)
  {
    msg_.s = std::move(arg);
    return Init_ManualControl_t(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_enabled_extensions
{
public:
  explicit Init_ManualControl_enabled_extensions(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_s enabled_extensions(::mavros_msgs::msg::ManualControl::_enabled_extensions_type arg)
  {
    msg_.enabled_extensions = std::move(arg);
    return Init_ManualControl_s(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_buttons2
{
public:
  explicit Init_ManualControl_buttons2(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_enabled_extensions buttons2(::mavros_msgs::msg::ManualControl::_buttons2_type arg)
  {
    msg_.buttons2 = std::move(arg);
    return Init_ManualControl_enabled_extensions(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_buttons
{
public:
  explicit Init_ManualControl_buttons(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_buttons2 buttons(::mavros_msgs::msg::ManualControl::_buttons_type arg)
  {
    msg_.buttons = std::move(arg);
    return Init_ManualControl_buttons2(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_r
{
public:
  explicit Init_ManualControl_r(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_buttons r(::mavros_msgs::msg::ManualControl::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_ManualControl_buttons(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_z
{
public:
  explicit Init_ManualControl_z(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_r z(::mavros_msgs::msg::ManualControl::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ManualControl_r(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_y
{
public:
  explicit Init_ManualControl_y(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_z y(::mavros_msgs::msg::ManualControl::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ManualControl_z(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_x
{
public:
  explicit Init_ManualControl_x(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_y x(::mavros_msgs::msg::ManualControl::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ManualControl_y(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_header
{
public:
  Init_ManualControl_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManualControl_x header(::mavros_msgs::msg::ManualControl::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ManualControl_x(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ManualControl>()
{
  return mavros_msgs::msg::builder::Init_ManualControl_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__BUILDER_HPP_
