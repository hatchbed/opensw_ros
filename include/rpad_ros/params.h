#include <string>

#include <rclcpp/rclcpp.hpp>

namespace rpad_ros {

template <class T>
T param(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_val, const std::string& description) {
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
  descriptor.description = description;
  descriptor.read_only = true;
  node->declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
  return node->get_parameter(name).get_value<T>();
}

}  // namespace rpad_ros;
