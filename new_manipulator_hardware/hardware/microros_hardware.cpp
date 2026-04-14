// microros_hardware.cpp - Fixed: Removed dual mode switching conflict

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
//return type:hardware_interface::CallbackReturn 
hardware_interface::CallbackReturn MicroROSHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    //runs the parent class' on_init, in that on_init it stores urdf parsed data into info, so it will have all joint info names robot name etc
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
//sets size of vector -- fills all with NaN values -- not 0 because NaN means no data recieved yet
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  last_teensy_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  servo_mode_ = false;
//a node named microros_hardware_interface is created 
  node_ = rclcpp::Node::make_shared("microros_hardware_interface");
  //call back group controls all callbacks -- its set to mutually exclusive so that only one callback runs at a time, command and state shouldnt get updated simultaneously
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Subscribe to ACTUAL joint states from Teensy (for visualization)
  auto joint_state_options = rclcpp::SubscriptionOptions();
  joint_state_options.callback_group = callback_group_;
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states_teensy", 10,
    std::bind(&MicroROSHardware::joint_state_callback, this, std::placeholders::_1),
    joint_state_options);

  // Subscribe to commands being sent to Teensy (for tracking)
  auto teensy_cmd_options = rclcpp::SubscriptionOptions();
  teensy_cmd_options.callback_group = callback_group_;
  teensy_command_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/joint_commands_to_teensy", 10,
    std::bind(&MicroROSHardware::teensy_command_callback, this, std::placeholders::_1),
    teensy_cmd_options);

  // Subscribe to servo mode switching
  //ros2 topic pub /servo_mode std_msgs/msg/Bool "data: true" --once ou can do something like this
  auto mode_switch_options = rclcpp::SubscriptionOptions();
  mode_switch_options.callback_group = callback_group_;
  mode_switch_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/servo_mode", rclcpp::QoS(10).reliable().transient_local(),
    std::bind(&MicroROSHardware::mode_switch_callback, this, std::placeholders::_1),
    mode_switch_options);

  // ========== CHANGED: Only LISTEN to Teensy control mode, don't publish commands ==========
  //ros2 topic pub /control_mode std_msgs/msg/String "data: 'PID_CONTROL'" --once you can do this to change 
  auto teensy_mode_options = rclcpp::SubscriptionOptions();
  teensy_mode_options.callback_group = callback_group_;
  teensy_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/control_mode", 10,
    std::bind(&MicroROSHardware::teensy_mode_callback, this, std::placeholders::_1),
    teensy_mode_options);

  joint_command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_commands_to_teensy", 10);

  mode_status_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/servo_mode_status", 10);
  
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  RCLCPP_INFO(node_->get_logger(), "========================================");
  RCLCPP_INFO(node_->get_logger(), "MicroROSHardware initialized");
  RCLCPP_INFO(node_->get_logger(), "Mode control: /servo_mode (true/false)");
  RCLCPP_INFO(node_->get_logger(), "Teensy control: /control_mode (monitored only)");
  RCLCPP_INFO(node_->get_logger(), "PID sync: handled by mode_switch_handler node");
  RCLCPP_INFO(node_->get_logger(), "Starting in TRAJECTORY mode");
  RCLCPP_INFO(node_->get_logger(), "========================================");

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
  RCLCPP_INFO(node_->get_logger(), "Activating...");

  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
    }
    hw_commands_[i] = hw_positions_[i];
    last_teensy_commands_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(node_->get_logger(), "Activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MicroROSHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MicroROSHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  executor_->spin_some(std::chrono::milliseconds(0));
  
  auto status_msg = std_msgs::msg::Bool();
  status_msg.data = servo_mode_;
  mode_status_pub_->publish(status_msg);
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MicroROSHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  if (servo_mode_ || teensy_in_pwm_mode_ ) {
    // Don't publish - servo is in control
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      hw_commands_[i] = hw_positions_[i];  // ← controller never forgets current pose
    }
    return hardware_interface::return_type::OK;  // don't publish
  }
    
  
  // Trajectory mode: publish commands
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data.resize(6);
  
  for (size_t i = 0; i < 6 && i < hw_commands_.size(); i++)
  {
    msg.data[i] = hw_commands_[i];
  }
  
  joint_command_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

void MicroROSHardware::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // ALWAYS update from actual feedback--- this updates the hw positions and hw velocities used in read 
  for (size_t i = 0; i < msg->name.size() && i < 6; i++)
  {
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

void MicroROSHardware::teensy_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // Track commands sent to Teensy, just listens to joint commands to teensy 
  for (size_t i = 0; i < msg->data.size() && i < last_teensy_commands_.size(); i++)
  {
    last_teensy_commands_[i] = msg->data[i];
  }
}
//this is servo_mode callback
void MicroROSHardware::mode_switch_callback(const std_msgs::msg::Bool::SharedPtr msg)
{//the msg will be either true or false , so initially its false 
  bool new_mode = msg->data;
  //if new data , servo mode is the new data 
  if (new_mode != servo_mode_)
  {
    servo_mode_ = new_mode;
    
    if (servo_mode_) //if true 
    {
      RCLCPP_INFO(node_->get_logger(), "🎮 SWITCHING TO SERVO MODE");
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "🤖 SWITCHING TO TRAJECTORY MODE");
      
      // ========== CRITICAL: Sync commands to current position ==========
      // Publish current joint states as new commands to prevent jumping!, so we need to make sure joint states are recieved no matter in what mode 
      auto sync_msg = std_msgs::msg::Float64MultiArray();
      sync_msg.data.resize(6);
      
      for (size_t i = 0; i < hw_commands_.size(); i++)
      {
        // Use current actual position from encoders
        hw_commands_[i] = hw_positions_[i];
        sync_msg.data[i] = hw_positions_[i];
        
        RCLCPP_INFO(node_->get_logger(), "  Joint %zu synced to: %.4f rad", i, hw_positions_[i]);
      }
      
      // Publish current position as command
      joint_command_pub_->publish(sync_msg);
      RCLCPP_INFO(node_->get_logger(), "✓ Published current position as starting command");
    }
  }
}

// ========== FIXED: Only monitor mode, don't publish commands ==========
// mode_switch_handler.py will handle publishing encoder positions to Teensy
//subscriber to control mode 
void MicroROSHardware::teensy_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string mode = msg->data;
  
  RCLCPP_INFO(node_->get_logger(), "========================================");
  RCLCPP_INFO(node_->get_logger(), "Teensy control mode: %s", mode.c_str());
  
  if (mode == "PID_CONTROL")
  {
    teensy_in_pwm_mode_ = true;  
    RCLCPP_INFO(node_->get_logger(), "PID mode activated");
    RCLCPP_INFO(node_->get_logger(), "Position sync handled by mode_switch_handler node");
    // ✓ REMOVED: Publishing commands here - let mode_switch_handler do it!
    // That node has access to ACTUAL encoder positions from /joint_states_teensy
  }
  else if (mode == "PWM_CONTROL")
  {
    teensy_in_pwm_mode_ = false; 
    RCLCPP_INFO(node_->get_logger(), "PWM mode activated - direct motor control");
  }
  
  RCLCPP_INFO(node_->get_logger(), "========================================");
}

}  // namespace new_manipulator_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  new_manipulator_hardware::MicroROSHardware, hardware_interface::SystemInterface)
