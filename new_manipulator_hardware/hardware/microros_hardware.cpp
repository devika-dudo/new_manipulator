#include "new_manipulator_hardware/microros_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace new_manipulator_hardware
{

hardware_interface::CallbackReturn MicroROSHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize vectors to store joint data
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Create ROS2 node for communication
  node_ = rclcpp::Node::make_shared("microros_hardware_interface");

  // Subscribe to joint states from ESP32
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states_teensy", 10,
    std::bind(&MicroROSHardware::joint_state_callback, this, std::placeholders::_1));

  // Publish joint commands to ESP32
  joint_command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_commands_to_teensy", 10);

  RCLCPP_INFO(node_->get_logger(), "MicroROSHardware initialized with %zu joints", info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MicroROSHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MicroROSHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MicroROSHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Activating MicroROSHardware...");

  // Initialize commands to current positions
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
    }
    hw_commands_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(node_->get_logger(), "MicroROSHardware activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MicroROSHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating MicroROSHardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MicroROSHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spin the node to process incoming joint state messages
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MicroROSHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // IMPORTANT: Only send commands for the first 6 joints (joint_1 through joint_6)
  // ESP32 only handles 6 motors
  auto msg = std_msgs::msg::Float64MultiArray();
  
  // Resize to exactly 6 elements
  msg.data.resize(6);
  
  // Copy only the first 6 joint commands
  for (size_t i = 0; i < 6 && i < hw_commands_.size(); i++)
  {
    msg.data[i] = hw_commands_[i];
  }
  
  joint_command_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

void MicroROSHardware::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update hardware positions and velocities from ESP32
  // ESP32 sends joint_1 through joint_6 in order
  
  for (size_t i = 0; i < msg->name.size() && i < 6; i++)
  {
    // Find the matching joint in our hw_positions_ array
    for (size_t j = 0; j < info_.joints.size(); j++)
    {
      if (info_.joints[j].name == msg->name[i])
      {
        hw_positions_[j] = msg->position[i];
        if (msg->velocity.size() > i)
        {
          hw_velocities_[j] = msg->velocity[i];
        }
        break;
      }
    }
  }
}

}  // namespace new_manipulator_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  new_manipulator_hardware::MicroROSHardware, hardware_interface::SystemInterface)
