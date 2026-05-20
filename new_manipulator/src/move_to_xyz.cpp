/**
 * @file move_to_xyz.cpp
 * @brief MoveIt 2 + ROS 2 topic-based controller for a 5-DOF robot arm.
 *
 * Subscribes to pose commands and executes them using MoveIt planning.
 * Topics:
 *   - /arm_pose_command     (geometry_msgs/PoseStamped)       : Full pose command
 *   - /arm_xyzrpy_command   (std_msgs/Float64MultiArray)      : [x, y, z, roll, pitch, yaw]
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MoveToXYZ : public rclcpp::Node
{
public:
  MoveToXYZ() : Node("move_to_xyz")
  {
    this->declare_parameter("planning_pipeline",        "ompl");
    this->declare_parameter("planner_id",               "RRTConnectkConfigDefault");
    this->declare_parameter("planning_time",            10.0);
    this->declare_parameter("position_tolerance",       0.001);
    this->declare_parameter("orientation_tolerance",    0.01);
    this->declare_parameter("max_velocity_scaling",     1.0);
    this->declare_parameter("max_acceleration_scaling", 1.0);

    RCLCPP_INFO(this->get_logger(), "MoveToXYZ node starting...");
  }

  void initialize()
  {
    arm_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm_group");

    setupPlanningParameters();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_pose_command", 10,
      std::bind(&MoveToXYZ::poseCallback, this, std::placeholders::_1));

    xyzrpy_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/arm_xyzrpy_command", 10,
      std::bind(&MoveToXYZ::xyzrpyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Ready. Listening on:");
    RCLCPP_INFO(this->get_logger(), "  /arm_pose_command   (geometry_msgs/PoseStamped)");
    RCLCPP_INFO(this->get_logger(), "  /arm_xyzrpy_command (std_msgs/Float64MultiArray [x,y,z,r,p,y])");
  }

private:
  // ── Planning configuration ───────────────────────────────────────────────
  void setupPlanningParameters()
  {
    arm_group_->setPlanningPipelineId(this->get_parameter("planning_pipeline").as_string());
    arm_group_->setPlannerId(this->get_parameter("planner_id").as_string());
    arm_group_->setPlanningTime(this->get_parameter("planning_time").as_double());
    arm_group_->setGoalPositionTolerance(this->get_parameter("position_tolerance").as_double());
    arm_group_->setGoalOrientationTolerance(this->get_parameter("orientation_tolerance").as_double());
    arm_group_->setMaxVelocityScalingFactor(this->get_parameter("max_velocity_scaling").as_double());
    arm_group_->setMaxAccelerationScalingFactor(this->get_parameter("max_acceleration_scaling").as_double());

    RCLCPP_INFO(this->get_logger(), "Planner : %s / %s",
      arm_group_->getPlanningPipelineId().c_str(),
      arm_group_->getPlannerId().c_str());
    RCLCPP_INFO(this->get_logger(), "Tolerances — pos: %.4f m  ori: %.4f rad",
      this->get_parameter("position_tolerance").as_double(),
      this->get_parameter("orientation_tolerance").as_double());
  }

  // ── Subscribers ──────────────────────────────────────────────────────────
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(), "PoseStamped → pos [%.3f, %.3f, %.3f]  rpy [%.2f°, %.2f°, %.2f°]",
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

    executePose(*msg);
  }

  void xyzrpyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (msg->data.size() != 6) {
      RCLCPP_ERROR(this->get_logger(),
        "Expected 6 values [x,y,z,roll,pitch,yaw], got %zu", msg->data.size());
      return;
    }

    double x = msg->data[0], y = msg->data[1], z = msg->data[2];
    double roll = msg->data[3], pitch = msg->data[4], yaw = msg->data[5];

    RCLCPP_INFO(this->get_logger(), "XYZ-RPY → pos [%.3f, %.3f, %.3f]  rpy [%.2f°, %.2f°, %.2f°]",
      x, y, z,
      roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "base_link";
    pose_msg.header.stamp    = this->now();
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    pose_msg.pose.orientation = tf2::toMsg(q);

    executePose(pose_msg);
  }

  // ── Core execution ───────────────────────────────────────────────────────
  void executePose(const geometry_msgs::msg::PoseStamped& target)
  {
    if (!arm_group_->setPoseTarget(target)) {
      RCLCPP_ERROR(this->get_logger(), "setPoseTarget failed — pose may be unreachable.");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = static_cast<bool>(arm_group_->plan(plan));

    if (!planned) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed. Possible causes:");
      RCLCPP_ERROR(this->get_logger(), "  - Pose out of workspace");
      RCLCPP_ERROR(this->get_logger(), "  - Orientation infeasible for 5-DOF");
      RCLCPP_ERROR(this->get_logger(), "  - Collision or joint limit violation");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Plan found. Executing...");
    auto result = arm_group_->execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "✅ Motion complete.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Execution failed (error code: %d).", result.val);
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for next command...");
  }

  // ── Members ──────────────────────────────────────────────────────────────
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr xyzrpy_sub_;
  std::mutex mutex_;
};

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveToXYZ>();
  node->initialize();

  RCLCPP_INFO(node->get_logger(), "🤖 move_to_xyz ready!");
  RCLCPP_INFO(node->get_logger(), "Example commands:");
  RCLCPP_INFO(node->get_logger(),
    "  ros2 topic pub --once /arm_xyzrpy_command std_msgs/Float64MultiArray "
    "\"data: [0.3, 0.0, 0.3, 0.0, -1.57, 0.0]\"");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}