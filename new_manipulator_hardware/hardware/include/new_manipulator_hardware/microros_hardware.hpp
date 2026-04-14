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
//hardware_interface::SystemINterface is the base class provided by ros2_control, so this class is inheriting from it
{
public:
  hardware_interface::CallbackReturn on_init(
    // ros2_control passes your URDF joint info here
// you read joint names, set up your vectors
// e.g. "ok i have 6 joints, initialize arrays for positions/velocities"
    const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_activate(
    // called when controller starts
// good place to establish micro-ROS connection to ESP32
// set joints to initial state

    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(
    // called when controller stops
// clean up micro-ROS connection
// safe stop
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(
    // you tell ros2_control "here's where to READ joint states FROM"
// returns pointers to your position/velocity variables
// ros2_control reads these every loop
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(
    // you tell ros2_control "here's where to WRITE commands TO"
// returns pointers to your command variables
// JointTrajectoryController writes into these every loop

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
    // Sync handshake
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sync_complete_sub_;
  bool syncing_to_position_;
  bool teensy_in_pwm_mode_ = true;

  std::vector<double> sync_positions_;  // position snapshot taken at mode switch

  // Callback
  void sync_complete_callback(const std_msgs::msg::Bool::SharedPtr msg);


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
