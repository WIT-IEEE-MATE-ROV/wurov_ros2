// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auv:msg/JoystickChaos.idl
// generated code does not contain a copyright notice

#ifndef AUV__MSG__DETAIL__JOYSTICK_CHAOS__TRAITS_HPP_
#define AUV__MSG__DETAIL__JOYSTICK_CHAOS__TRAITS_HPP_

#include "auv/msg/detail/joystick_chaos__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<auv::msg::JoystickChaos>()
{
  return "auv::msg::JoystickChaos";
}

template<>
struct has_fixed_size<auv::msg::JoystickChaos>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auv::msg::JoystickChaos>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auv::msg::JoystickChaos>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUV__MSG__DETAIL__JOYSTICK_CHAOS__TRAITS_HPP_