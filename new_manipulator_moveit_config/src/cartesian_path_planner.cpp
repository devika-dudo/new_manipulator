/**
 * @file cartesian_path_interactive.cpp
 * @brief Interactive Cartesian path planner with "down mode" for Z-axis movement
 * 
 * Features:
 * - Full pose control: move <x> <y> <z> <roll> <pitch> <yaw>
 * - Down mode: Toggle with 'down' command, then just type Z values
 * - Fixed downward orientation (roll=-0.235, pitch=1.428, yaw=1.529)
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <thread>
#include <string>
#include <sstream>

class CartesianPathPlanner : public rclcpp::Node
{
public:
  CartesianPathPlanner(const rclcpp::NodeOptions& options)
    : Node("cartesian_path_interactive", options),
      down_mode_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("planning_group", "arm_group");
    this->declare_parameter<double>("down_roll", -0.235);
    this->declare_parameter<double>("down_pitch", 1.428);
    this->declare_parameter<double>("down_yaw", 1.529);
    
    std::string planning_group = this->get_parameter("planning_group").as_string();
    down_roll_ = this->get_parameter("down_roll").as_double();
    down_pitch_ = this->get_parameter("down_pitch").as_double();
    down_yaw_ = this->get_parameter("down_yaw").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interface for group: %s", planning_group.c_str());
    RCLCPP_INFO(this->get_logger(), "Down orientation: roll=%.3f, pitch=%.3f, yaw=%.3f", 
                down_roll_, down_pitch_, down_yaw_);
    
    // Create a separate node for MoveGroupInterface
    moveit_node_ = std::make_shared<rclcpp::Node>(
      "moveit_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    
    // Create a separate executor for the MoveIt node
    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(moveit_node_);
    
    // Start the executor in a separate thread
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(this->get_logger(), "Starting MoveIt executor thread...");
      moveit_executor_->spin();
    });
    
    // Wait for executor to be running
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt to initialize...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Initialize MoveGroupInterface with the separate node
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node_, planning_group);
    
    // Wait a bit more for state monitor to connect
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Configure planning parameters
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setGoalPositionTolerance(0.001);
    move_group_->setGoalOrientationTolerance(0.01);
    
    // Set planner
    move_group_->setPlannerId("RRTConnect");
    move_group_->setPlanningPipelineId("ompl");
    
    RCLCPP_INFO(this->get_logger(), "✅ MoveIt initialized successfully!");
    RCLCPP_INFO(this->get_logger(), "Planning group: %s", move_group_->getName().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
  }
  
  ~CartesianPathPlanner()
  {
    // Shutdown the executor
    if (moveit_executor_) {
      moveit_executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }
  
  void printCurrentPose()
  {
    try {
      geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
      
      // Convert quaternion to RPY
      tf2::Quaternion q(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
      );
      
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      
      RCLCPP_INFO(this->get_logger(), "📍 Current Position: x=%.3f, y=%.3f, z=%.3f",
                  current_pose.pose.position.x,
                  current_pose.pose.position.y,
                  current_pose.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "📐 Current Orientation: roll=%.3f, pitch=%.3f, yaw=%.3f (%.1f°, %.1f°, %.1f°)",
                  roll, pitch, yaw,
                  roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
      
      // Store for down mode
      current_x_ = current_pose.pose.position.x;
      current_y_ = current_pose.pose.position.y;
      current_z_ = current_pose.pose.position.z;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current pose: %s", e.what());
    }
  }
  
  void toggleDownMode()
  {
    down_mode_ = !down_mode_;
    
    if (down_mode_) {
      // Entering down mode - get current position
      printCurrentPose();
      
      std::cout << "\n🔽 DOWN MODE ENABLED!\n";
      std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
      std::cout << "Current XY position locked: x=" << current_x_ << ", y=" << current_y_ << "\n";
      std::cout << "Fixed orientation: roll=" << down_roll_ << ", pitch=" << down_pitch_ 
                << ", yaw=" << down_yaw_ << "\n";
      std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
      std::cout << "Now just type a Z value to move straight down!\n";
      std::cout << "Example: 0.3  (moves to Z=0.3)\n";
      std::cout << "Type 'down' again to exit down mode.\n";
      std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    } else {
      std::cout << "\n⬆️  DOWN MODE DISABLED!\n";
      std::cout << "Back to normal mode. Use 'move' commands.\n";
    }
  }
  
  bool moveDown(double target_z)
  {
    if (!down_mode_) {
      RCLCPP_ERROR(this->get_logger(), "Not in down mode!");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "🔽 Moving DOWN to Z=%.3f (keeping XY at %.3f, %.3f)", 
                target_z, current_x_, current_y_);
    
    return moveToCartesianPose(current_x_, current_y_, target_z, 
                               down_roll_, down_pitch_, down_yaw_, true);
  }
  
  bool moveToCartesianPose(double x, double y, double z, 
                           double roll, double pitch, double yaw,
                           bool use_cartesian_path = true)
  {
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Planning motion to target pose:");
    RCLCPP_INFO(this->get_logger(), "  Position: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    RCLCPP_INFO(this->get_logger(), "  Orientation: roll=%.3f, pitch=%.3f, yaw=%.3f", 
                roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "              (%.1f°, %.1f°, %.1f°)",
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    
    // Create target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    
    // Convert RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    target_pose.orientation = tf2::toMsg(q);
    
    bool success = false;
    
    if (use_cartesian_path) {
      // Use Cartesian path (straight line motion)
      success = executeCartesianPath(target_pose);
    } else {
      // Use regular planning (may not be straight line)
      success = executeRegularPath(target_pose);
    }
    
    if (success && down_mode_) {
      // Update current Z position in down mode
      current_z_ = z;
    }
    
    RCLCPP_INFO(this->get_logger(), "============================================================");
    return success;
  }
  
  bool executeCartesianPath(const geometry_msgs::msg::Pose& target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "Using Cartesian path planning (straight line)...");
    
    // Create waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    
    // Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 1cm steps
    const double jump_threshold = 0.0;  // Disable jump threshold
    
    double fraction = move_group_->computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory
    );
    
    RCLCPP_INFO(this->get_logger(), "Cartesian path: %.2f%% achieved", fraction * 100.0);
    
    if (fraction >= 0.95) {  // At least 95% of path planned
      RCLCPP_INFO(this->get_logger(), "✅ Cartesian planning successful! Executing...");
      
      // Execute the trajectory
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      
      auto result = move_group_->execute(plan);
      
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ Motion executed successfully!");
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Execution failed!");
        return false;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️  Cartesian path only %.2f%% complete", fraction * 100.0);
      RCLCPP_INFO(this->get_logger(), "Trying regular planning instead...");
      return executeRegularPath(target_pose);
    }
  }
  
  bool executeRegularPath(const geometry_msgs::msg::Pose& target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "Using regular motion planning...");
    
    // Set target pose
    move_group_->setPoseTarget(target_pose);
    
    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "✅ Planning successful! Executing...");
      
      auto result = move_group_->execute(plan);
      
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ Motion executed successfully!");
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Execution failed!");
        return false;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Planning failed!");
      RCLCPP_ERROR(this->get_logger(), "Possible reasons:");
      RCLCPP_ERROR(this->get_logger(), "  - Target pose is unreachable");
      RCLCPP_ERROR(this->get_logger(), "  - Joint limits exceeded");
      RCLCPP_ERROR(this->get_logger(), "  - Collision detected");
      return false;
    }
  }
  
  bool isDownMode() const { return down_mode_; }

private:
  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
  std::thread executor_thread_;
  
  // Down mode state
  bool down_mode_;
  double current_x_, current_y_, current_z_;
  double down_roll_, down_pitch_, down_yaw_;
};

void printHelp()
{
  std::cout << "\n============================================================\n";
  std::cout << "🤖 Interactive Cartesian Path Planner\n";
  std::cout << "============================================================\n";
  std::cout << "\nCommands:\n";
  std::cout << "  move <x> <y> <z> <roll> <pitch> <yaw>  - Move to XYZ with RPY\n";
  std::cout << "  down                                    - Toggle DOWN MODE\n";
  std::cout << "  current                                 - Show current pose\n";
  std::cout << "  help                                    - Show this help\n";
  std::cout << "  quit / exit                             - Exit program\n";
  std::cout << "\n🔽 DOWN MODE:\n";
  std::cout << "  When down mode is active, just type a Z value:\n";
  std::cout << "  Example: 0.35  (moves to Z=0.35 with fixed orientation)\n";
  std::cout << "  The robot keeps current XY position and fixed down orientation\n";
  std::cout << "\nNote: XYZ in meters, RPY in radians\n";
  std::cout << "      Degrees to radians: deg * 0.01745 (or deg * π/180)\n";
  std::cout << "\nExamples:\n";
  std::cout << "  move 0.3 0.0 0.4 0 0 0              - Move to position\n";
  std::cout << "  move 0.3 0.1 0.5 -0.235 1.428 1.529 - Move with down orientation\n";
  std::cout << "  down                                 - Enable down mode\n";
  std::cout << "  0.25                                 - (in down mode) Move to Z=0.25\n";
  std::cout << "============================================================\n";
}

void interactiveControl(std::shared_ptr<CartesianPathPlanner> planner)
{
  printHelp();
  
  std::string line;
  while (rclcpp::ok()) {
    if (planner->isDownMode()) {
      std::cout << "\n🔽 > ";
    } else {
      std::cout << "\n> ";
    }
    
    if (!std::getline(std::cin, line)) {
      break;
    }
    
    std::istringstream iss(line);
    std::string command;
    iss >> command;
    
    if (command.empty()) {
      continue;
    }
    
    if (command == "quit" || command == "exit" || command == "q") {
      std::cout << "Exiting...\n";
      break;
    }
    else if (command == "help" || command == "h") {
      printHelp();
    }
    else if (command == "current" || command == "c") {
      planner->printCurrentPose();
    }
    else if (command == "down" || command == "d") {
      planner->toggleDownMode();
    }
    else if (command == "move" || command == "m") {
      if (planner->isDownMode()) {
        std::cout << "⚠️  Exit down mode first! Type 'down' to toggle.\n";
        continue;
      }
      
      double x, y, z, roll, pitch, yaw;
      
      if (iss >> x >> y >> z >> roll >> pitch >> yaw) {
        planner->moveToCartesianPose(x, y, z, roll, pitch, yaw);
      } else {
        std::cout << "❌ Invalid format!\n";
        std::cout << "Usage: move <x> <y> <z> <roll> <pitch> <yaw>\n";
      }
    }
    else {
      // In down mode, try to parse as a single Z value
      if (planner->isDownMode()) {
        try {
          double z = std::stod(command);
          planner->moveDown(z);
        } catch (...) {
          std::cout << "❌ In down mode, enter a single Z value or use 'down' to exit\n";
        }
      } else {
        std::cout << "❌ Unknown command: '" << command << "'\n";
        std::cout << "Type 'help' for available commands\n";
      }
    }
  }
  
  std::cout << "\nShutting down...\n";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create node options
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // Create the main node
  auto planner = std::make_shared<CartesianPathPlanner>(node_options);
  
  // Spin the main node in a separate thread
  std::thread spinner([planner]() {
    rclcpp::spin(planner);
  });
  
  try {
    // Wait for full initialization
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    RCLCPP_INFO(planner->get_logger(), "✅ Ready!");
    
    // Show current pose
    std::cout << "\n📍 Current robot pose:\n";
    planner->printCurrentPose();
    
    // Start interactive control
    interactiveControl(planner);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(planner->get_logger(), "Error: %s", e.what());
  }
  
  rclcpp::shutdown();
  spinner.join();
  
  return 0;
}
