// Joystick Servo with SIMPLE Control + Gripper (Clock Fixed)
// - Cartesian mode: smooth continuous motion
// - Joint mode: Button pressed = move, released = stop
// - Gripper: Button 2 toggle open/close

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <thread>
#include <cmath>
#include <chrono>
#include <map>

// Configuration
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string GRIPPER_TOPIC = "/hand_controller/joint_trajectory";
const std::string EEF_FRAME_ID = "fake_link";
const std::string BASE_FRAME_ID = "base_link";

// Deadband configuration
const double STICK_DEADBAND = 0.15;
const double TWIST_DEADBAND = 0.20;
const double HAT_DEADBAND = 0.05;
const double THROTTLE_DEADBAND = 0.05;

// Ramping configuration
const double ACCELERATION_RATE = 2.0;
const double UPDATE_RATE = 50.0;
const double MAX_DELTA = ACCELERATION_RATE / UPDATE_RATE;

// Command timeout for Cartesian mode
const double COMMAND_TIMEOUT_MS = 200.0;

// Gripper positions
const double GRIPPER_OPEN_POSITION = 1.0;
const double GRIPPER_CLOSED_POSITION = 0.0;

// Optional axis filtering
const bool USE_DOMINANT_AXIS_FILTER = false;
const double DOMINANT_AXIS_RATIO = 1.5;

// Joystick mapping
enum Axis { STICK_X = 0, STICK_Y = 1, STICK_TWIST = 2, THROTTLE = 3, HAT_X = 4, HAT_Y = 5 };
enum Button { TRIGGER = 0, BUTTON_2 = 1, BUTTON_3 = 2, BUTTON_4 = 3, BUTTON_5 = 4, BUTTON_6 = 5,
              BUTTON_7 = 6, BUTTON_8 = 7, BUTTON_9 = 8, BUTTON_10 = 9, BUTTON_11 = 10, BUTTON_12 = 11 };

std::map<Axis, double> AXIS_DEFAULTS = { { THROTTLE, -1.0 } };

/** Apply deadband with smooth scaling */
double applyDeadband(double value, double deadband)
{
  double abs_value = std::abs(value);
  if (abs_value < deadband) return 0.0;
  
  double sign = (value > 0) ? 1.0 : -1.0;
  double scaled_value = (abs_value - deadband) / (1.0 - deadband);
  return sign * scaled_value;
}

/** Suppress weaker axis (optional) */
void applyDominantAxisFiltering(double& axis1, double& axis2, double ratio = DOMINANT_AXIS_RATIO)
{
  double abs1 = std::abs(axis1);
  double abs2 = std::abs(axis2);
  
  if (abs1 > abs2 * ratio) {
    axis2 = 0.0;
  } else if (abs2 > abs1 * ratio) {
    axis1 = 0.0;
  }
}

/** Smooth velocity ramping */
double applyRamping(double current, double target, double max_delta)
{
  double error = target - current;
  
  if (std::abs(error) <= max_delta) {
    return target;
  }
  
  return current + (error > 0 ? max_delta : -max_delta);
}

/** Check if any joint button is pressed */
bool isJointButtonPressed(const std::vector<int>& buttons)
{
  if (buttons.size() < 12) return false;
  
  return (buttons[BUTTON_3] || buttons[BUTTON_4] || buttons[BUTTON_5] || buttons[BUTTON_6] ||
          buttons[BUTTON_7] || buttons[BUTTON_8] || buttons[BUTTON_9] || buttons[BUTTON_10] ||
          buttons[BUTTON_11]);
}

/** Convert joystick to Cartesian commands */
void convertJoyToTwist(const std::vector<float>& axes,
                       std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist)
{
  double stick_x = applyDeadband(axes[STICK_X], STICK_DEADBAND);
  double stick_y = applyDeadband(axes[STICK_Y], STICK_DEADBAND);
  double stick_twist = applyDeadband(axes[STICK_TWIST], TWIST_DEADBAND);
  double hat_x = applyDeadband(axes[HAT_X], HAT_DEADBAND);
  double hat_y = applyDeadband(axes[HAT_Y], HAT_DEADBAND);
  
  if (USE_DOMINANT_AXIS_FILTER) {
    applyDominantAxisFiltering(stick_x, stick_y);
    applyDominantAxisFiltering(hat_x, hat_y);
  }
  
  double throttle_raw = axes[THROTTLE];
  double throttle_normalized = (throttle_raw - AXIS_DEFAULTS.at(THROTTLE)) / 2.0;
  double throttle_deadbanded = applyDeadband(throttle_normalized - 0.5, THROTTLE_DEADBAND);

  twist->twist.linear.x = stick_y;
  twist->twist.linear.y = -stick_x;
  twist->twist.linear.z = throttle_deadbanded;
  twist->twist.angular.x = hat_y;
  twist->twist.angular.y = -hat_x;
  twist->twist.angular.z = stick_twist;
}

/** Update command frame with feedback */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons, rclcpp::Logger logger)
{
  static bool trigger_pressed_last = false;
  bool trigger_pressed_now = buttons[TRIGGER];
  
  if (trigger_pressed_now && !trigger_pressed_last) {
    if (frame_name == EEF_FRAME_ID) {
      frame_name = BASE_FRAME_ID;
      RCLCPP_INFO(logger, "🎯 [BASE FRAME] - World coordinates");
    } else {
      frame_name = EEF_FRAME_ID;
      RCLCPP_INFO(logger, "🎯 [END-EFFECTOR FRAME] - Gripper coordinates");
    }
  }
  
  trigger_pressed_last = trigger_pressed_now;
}

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), 
      frame_to_publish_(BASE_FRAME_ID),
      gripper_is_open_(false),
      button_2_was_pressed_(false),
      received_first_joy_msg_(false)
  {
    // Initialize velocity state to zero
    current_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Setup publishers and subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(GRIPPER_TOPIC, rclcpp::SystemDefaultsQoS());
    collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // Timer for cartesian timeout (don't start until we get first joy message)
    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JoyToServoPub::timeoutCheck, this));

    RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║   Logitech Extreme 3D Pro - Simple Control              ║");
    RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "🎮 Controls:");
    RCLCPP_INFO(this->get_logger(), "   Stick/Twist/Throttle/Hat → Cartesian motion");
    RCLCPP_INFO(this->get_logger(), "   Buttons 3-11 → Joint control (hold to move)");
    RCLCPP_INFO(this->get_logger(), "      • Joints 1-4: Bidirectional (+ and -)");
    RCLCPP_INFO(this->get_logger(), "      • Joint 5: Button 11 only (forward)");
    RCLCPP_INFO(this->get_logger(), "   Button 2 → Toggle gripper open/close");
    RCLCPP_INFO(this->get_logger(), "   Trigger → Toggle frame");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "📡 Waiting for joystick...");
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Initialize timestamps on first message
    if (!received_first_joy_msg_) {
      last_command_time_ = this->now();
      last_joint_command_time_ = this->now();
      received_first_joy_msg_ = true;
      RCLCPP_INFO(this->get_logger(), "✅ Joystick connected!");
    }
    
    // Update frame toggle
    updateCmdFrame(frame_to_publish_, msg->buttons, this->get_logger());
    
    // Handle gripper toggle (Button 2)
    handleGripperToggle(msg->buttons);

    // Check if we're in joint mode
    bool joint_mode = isJointButtonPressed(msg->buttons);
    
    if (!joint_mode)
    {
      // CARTESIAN MODE
      last_command_time_ = this->now();
      
      auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      convertJoyToTwist(msg->axes, twist_msg);
      
      // Apply smooth ramping
      current_velocity_.linear_x = applyRamping(current_velocity_.linear_x, twist_msg->twist.linear.x, MAX_DELTA);
      current_velocity_.linear_y = applyRamping(current_velocity_.linear_y, twist_msg->twist.linear.y, MAX_DELTA);
      current_velocity_.linear_z = applyRamping(current_velocity_.linear_z, twist_msg->twist.linear.z, MAX_DELTA);
      current_velocity_.angular_x = applyRamping(current_velocity_.angular_x, twist_msg->twist.angular.x, MAX_DELTA);
      current_velocity_.angular_y = applyRamping(current_velocity_.angular_y, twist_msg->twist.angular.y, MAX_DELTA);
      current_velocity_.angular_z = applyRamping(current_velocity_.angular_z, twist_msg->twist.angular.z, MAX_DELTA);
      
      twist_msg->twist.linear.x = current_velocity_.linear_x;
      twist_msg->twist.linear.y = current_velocity_.linear_y;
      twist_msg->twist.linear.z = current_velocity_.linear_z;
      twist_msg->twist.angular.x = current_velocity_.angular_x;
      twist_msg->twist.angular.y = current_velocity_.angular_y;
      twist_msg->twist.angular.z = current_velocity_.angular_z;
      
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // JOINT MODE - Send unitless commands (0.0 to 1.0 range)
      current_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      
      auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
      
      // Check if we have enough buttons (need at least 12 for 4 joint pairs)
      if (msg->buttons.size() < 12) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "⚠️  Not enough buttons! Have %zu, need 12", msg->buttons.size());
        return;
      }
      
      // Joint 1: Buttons 3 and 4
      double j1_vel = static_cast<double>(msg->buttons[BUTTON_3] - msg->buttons[BUTTON_4]);
      joint_msg->joint_names.push_back("joint_1");
      joint_msg->velocities.push_back(j1_vel);
      
      // Joint 2: Buttons 5 and 6
      double j2_vel = static_cast<double>(msg->buttons[BUTTON_5] - msg->buttons[BUTTON_6]);
      joint_msg->joint_names.push_back("joint_2");
      joint_msg->velocities.push_back(j2_vel);
      
      // Joint 3: Buttons 7 and 8
      double j3_vel = static_cast<double>(msg->buttons[BUTTON_7] - msg->buttons[BUTTON_8]);
      joint_msg->joint_names.push_back("joint_3");
      joint_msg->velocities.push_back(j3_vel);
      
      // Joint 4: Buttons 9 and 10
      double j4_vel = static_cast<double>(msg->buttons[BUTTON_9] - msg->buttons[BUTTON_10]);
      joint_msg->joint_names.push_back("joint_4");
      joint_msg->velocities.push_back(j4_vel);
      
      // Joint 5: Buttons 11 only (no button 12 available)
      // Hold button 11 to move forward in positive direction
      double j5_vel = static_cast<double>(msg->buttons[BUTTON_11]);
      joint_msg->joint_names.push_back("joint_5");
      joint_msg->velocities.push_back(j5_vel);
      
      // CRITICAL: Add fake_joint (required by arm_group_controller)
      joint_msg->joint_names.push_back("fake_joint");
      joint_msg->velocities.push_back(0.0);
      
      // ALWAYS publish - even zeros (important for servo to know we're active)
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = "base_link";
      
      // Set duration as double (seconds) - critical for servo
      joint_msg->duration = 0.1;  // 100ms
      
      joint_pub_->publish(std::move(joint_msg));
      
      // DEBUG: Print what we're sending
      if (std::abs(j1_vel) > 0.01 || std::abs(j2_vel) > 0.01 || std::abs(j3_vel) > 0.01 || 
          std::abs(j4_vel) > 0.01 || std::abs(j5_vel) > 0.01) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "📤 Joint cmd: [%.1f, %.1f, %.1f, %.1f, %.1f] + fake_joint",
                            j1_vel, j2_vel, j3_vel, j4_vel, j5_vel);
      }
      
      // Update last joint command time
      last_joint_command_time_ = this->now();
    }
  }
  
  void handleGripperToggle(const std::vector<int>& buttons)
  {
    bool button_2_pressed = buttons[BUTTON_2];
    
    // Rising edge detection
    if (button_2_pressed && !button_2_was_pressed_) {
      // Toggle gripper state
      gripper_is_open_ = !gripper_is_open_;
      
      // Send gripper command
      auto gripper_msg = trajectory_msgs::msg::JointTrajectory();
      gripper_msg.joint_names = {"joint_6"};
      
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {gripper_is_open_ ? GRIPPER_OPEN_POSITION : GRIPPER_CLOSED_POSITION};
      point.time_from_start = rclcpp::Duration::from_seconds(1.0);
      
      gripper_msg.points = {point};
      gripper_pub_->publish(gripper_msg);
      
      RCLCPP_INFO(this->get_logger(), "🤏 Gripper → %s", gripper_is_open_ ? "OPEN" : "CLOSED");
    }
    
    button_2_was_pressed_ = button_2_pressed;
  }

  void timeoutCheck()
  {
    // Don't run timeout check until we've received first joy message
    if (!received_first_joy_msg_) {
      return;
    }
    
    auto now = this->now();
    
    // Check cartesian timeout
    auto cart_duration = (now - last_command_time_).seconds() * 1000.0;
    
    if (cart_duration > COMMAND_TIMEOUT_MS) {
      bool any_non_zero = false;
      
      current_velocity_.linear_x = applyRamping(current_velocity_.linear_x, 0.0, MAX_DELTA);
      current_velocity_.linear_y = applyRamping(current_velocity_.linear_y, 0.0, MAX_DELTA);
      current_velocity_.linear_z = applyRamping(current_velocity_.linear_z, 0.0, MAX_DELTA);
      current_velocity_.angular_x = applyRamping(current_velocity_.angular_x, 0.0, MAX_DELTA);
      current_velocity_.angular_y = applyRamping(current_velocity_.angular_y, 0.0, MAX_DELTA);
      current_velocity_.angular_z = applyRamping(current_velocity_.angular_z, 0.0, MAX_DELTA);
      
      if (std::abs(current_velocity_.linear_x) > 0.001 ||
          std::abs(current_velocity_.linear_y) > 0.001 ||
          std::abs(current_velocity_.linear_z) > 0.001 ||
          std::abs(current_velocity_.angular_x) > 0.001 ||
          std::abs(current_velocity_.angular_y) > 0.001 ||
          std::abs(current_velocity_.angular_z) > 0.001) {
        any_non_zero = true;
      }
      
      if (any_non_zero) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->twist.linear.x = current_velocity_.linear_x;
        twist_msg->twist.linear.y = current_velocity_.linear_y;
        twist_msg->twist.linear.z = current_velocity_.linear_z;
        twist_msg->twist.angular.x = current_velocity_.angular_x;
        twist_msg->twist.angular.y = current_velocity_.angular_y;
        twist_msg->twist.angular.z = current_velocity_.angular_z;
        twist_msg->header.frame_id = frame_to_publish_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
      }
    }
    
    // Check joint timeout - send explicit stop if no recent messages
    auto joint_duration = (now - last_joint_command_time_).seconds() * 1000.0;
    if (joint_duration > 200.0 && joint_duration < 300.0) {
      RCLCPP_WARN(this->get_logger(), "⚠️  No joint cmds for 200ms - STOP");
      
      auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
      joint_msg->joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "fake_joint"};
      joint_msg->velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = "base_link";
      joint_msg->duration = 0.1;
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  struct VelocityState {
    double linear_x, linear_y, linear_z;
    double angular_x, angular_y, angular_z;
  };
  
  VelocityState current_velocity_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_joint_command_time_;
  
  // Gripper state
  bool gripper_is_open_;
  bool button_2_was_pressed_;
  
  // Flag to track if we've received first joy message
  bool received_first_joy_msg_;
  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::string frame_to_publish_;
};

}  // namespace moveit_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)
