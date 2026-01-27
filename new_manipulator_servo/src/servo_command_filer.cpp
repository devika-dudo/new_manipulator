// servo_command_filter.cpp - Filters servo commands based on pause state
// This node sits between MoveIt Servo and /joint_commands_to_teensy

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/msg/joint_jog.hpp>

class ServoCommandFilter : public rclcpp::Node
{
public:
  ServoCommandFilter() : Node("servo_command_filter"), paused_(true)  // Start paused
  {
    // Subscribe to pause commands
    pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/servo_node/pause_servo", 10,
      std::bind(&ServoCommandFilter::pause_callback, this, std::placeholders::_1));
    
    // Subscribe to MoveIt Servo's output (delta joint commands)
    servo_cmd_sub_ = this->create_subscription<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10,
      std::bind(&ServoCommandFilter::servo_command_callback, this, std::placeholders::_1));
    
    // Publish to Teensy (only when NOT paused)
    teensy_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/joint_commands_to_teensy", 10);
    
    // Subscribe to joint states to track current position
    joint_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joint_states_teensy", 10,
      std::bind(&ServoCommandFilter::joint_state_callback, this, std::placeholders::_1));
    
    current_positions_.resize(6, 0.0);
    
    RCLCPP_INFO(this->get_logger(), "Servo Command Filter initialized");
    RCLCPP_INFO(this->get_logger(), "  Listening to: /servo_node/delta_joint_cmds");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: /joint_commands_to_teensy");
    RCLCPP_INFO(this->get_logger(), "  Pause control: /servo_node/pause_servo");
    RCLCPP_INFO(this->get_logger(), "  Status: PAUSED (waiting for servo mode)");
  }

private:
  void pause_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool new_pause = msg->data;
    if (new_pause != paused_)
    {
      paused_ = new_pause;
      if (paused_)
      {
        RCLCPP_INFO(this->get_logger(), "🛑 Servo output PAUSED");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "▶️  Servo output ACTIVE");
      }
    }
  }
  
  void joint_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Update current positions (needed for delta→absolute conversion)
    if (msg->data.size() >= 6)
    {
      for (size_t i = 0; i < 6; i++)
      {
        current_positions_[i] = msg->data[i];
      }
    }
  }
  
  void servo_command_callback(const control_msgs::msg::JointJog::SharedPtr msg)
  {
    if (paused_)
    {
      // Don't publish when paused
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Servo paused - ignoring commands");
      return;
    }
    
    // Convert delta commands to absolute positions
    auto cmd = std_msgs::msg::Float64MultiArray();
    cmd.data.resize(6);
    
    for (size_t i = 0; i < 6 && i < msg->velocities.size(); i++)
    {
      // Integrate delta command: new_pos = current_pos + delta * dt
      // Assuming servo publishes at ~100Hz, dt ≈ 0.01s
      cmd.data[i] = current_positions_[i] + msg->velocities[i] * 0.01;
    }
    
    teensy_cmd_pub_->publish(cmd);
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Publishing servo command");
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr servo_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr teensy_cmd_pub_;
  
  bool paused_;
  std::vector<double> current_positions_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoCommandFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
