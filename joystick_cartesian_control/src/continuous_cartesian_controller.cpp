#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class ContinuousCartesianController : public rclcpp::Node
{
public:
  ContinuousCartesianController() : Node("continuous_cartesian_controller")
  {
    // Initialize MoveIt interface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm_group");
    
    // Configure for smooth continuous motion
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    
    // Subscribe to joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ContinuousCartesianController::joyCallback, this, std::placeholders::_1));
    
    // Timer for continuous updates
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ContinuousCartesianController::controlLoop, this));
    
    velocity_scale_ = 0.1;  // meters per second
    active_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Continuous Cartesian Controller ready");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    
    // Enable/disable control with button
    if (msg->buttons.size() > 0 && msg->buttons[0]) {
      active_ = true;
    } else if (msg->buttons.size() > 1 && msg->buttons[1]) {
      active_ = false;
      target_vel_ = {0, 0, 0};
      return;
    }
    
    if (!active_) return;
    
    // Get velocities from joystick
    const double deadzone = 0.15;
    
    if (msg->axes.size() >= 5) {
      double x = msg->axes[1];
      double y = msg->axes[0];
      double z = msg->axes[4];
      
      target_vel_[0] = (std::abs(x) > deadzone) ? x * velocity_scale_ : 0.0;
      target_vel_[1] = (std::abs(y) > deadzone) ? y * velocity_scale_ : 0.0;
      target_vel_[2] = (std::abs(z) > deadzone) ? z * velocity_scale_ : 0.0;
    }
  }
  
  void controlLoop()
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    
    if (!active_ || (target_vel_[0] == 0 && target_vel_[1] == 0 && target_vel_[2] == 0)) {
      return;
    }
    
    // Get current pose
    auto current = move_group_->getCurrentPose().pose;
    
    // Calculate new target (small increment)
    double dt = 0.1;  // 100ms timer
    geometry_msgs::msg::Pose target = current;
    target.position.x += target_vel_[0] * dt;
    target.position.y += target_vel_[1] * dt;
    target.position.z += target_vel_[2] * dt;
    
    // Plan cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints = {target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    double fraction = move_group_->computeCartesianPath(
      waypoints, 0.01, 0.0, trajectory);
    
    if (fraction > 0.9) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      move_group_->asyncExecute(plan);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Cartesian path blocked, only %.0f%% reachable", fraction * 100);
    }
  }
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  std::mutex vel_mutex_;
  std::array<double, 3> target_vel_ = {0, 0, 0};
  double velocity_scale_;
  bool active_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContinuousCartesianController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
