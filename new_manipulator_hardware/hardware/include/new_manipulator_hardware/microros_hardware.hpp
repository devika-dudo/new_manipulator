#ifndef NEW_MANIPULATOR_HARDWARE__MICROROS_HARDWARE_HPP_
#define NEW_MANIPULATOR_HARDWARE__MICROROS_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace new_manipulator_hardware
{
class MicroROSHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> last_teensy_commands_;
  
  bool servo_mode_;
  std::atomic<bool> executor_running_;
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr teensy_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_switch_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teensy_mode_sub_;
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_status_pub_;
  
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void teensy_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void mode_switch_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void teensy_mode_callback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace new_manipulator_hardware

#endif  // NEW_MANIPULATOR_HARDWARE__MICROROS_HARDWARE_HPP_
