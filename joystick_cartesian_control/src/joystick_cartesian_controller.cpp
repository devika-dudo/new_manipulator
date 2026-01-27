#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <cmath>

class CartesianJoystickControl : public rclcpp::Node
{
public:
    CartesianJoystickControl() : Node("joystick_cartesian_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // Create a separate node for MoveGroup
        move_group_node_ = std::make_shared<rclcpp::Node>("move_group_node",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
        
        // Create executor for MoveGroup node
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(move_group_node_);
        
        // Spin executor in separate thread
        executor_thread_ = std::thread([this]() { 
            this->executor_->spin(); 
        });
        
        // Subscribe to joystick
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&CartesianJoystickControl::joyCallback, this, std::placeholders::_1));
        
        // Use a one-shot timer to initialize MoveGroup
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CartesianJoystickControl::initializeMoveGroup, this));
        
        RCLCPP_INFO(this->get_logger(), "Node created - Logitech Extreme 3D Pro");
    }
    
    ~CartesianJoystickControl()
    {
        if (executor_) {
            executor_->cancel();
        }
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

private:
    void initializeMoveGroup()
    {
        init_timer_->cancel();
        
        try {
            // Initialize MoveGroup
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                move_group_node_, "arm_group");
            
            // ⚡ MAXIMUM SPEED - Changed from 0.5 to 1.0
            move_group_->setMaxVelocityScalingFactor(1.0);  // 100% velocity
            move_group_->setMaxAccelerationScalingFactor(1.0);  // 100% acceleration
            
            RCLCPP_INFO(this->get_logger(), "MoveGroup initialized with MAXIMUM speed");
            
            rclcpp::sleep_for(std::chrono::seconds(1));
        
            // SET TO WORLD FRAME
            move_group_->setPoseReferenceFrame("world");
            
            rclcpp::sleep_for(std::chrono::seconds(1));
            
            // =============== CRITICAL FRAME INFORMATION ===============
            std::string planning_frame = move_group_->getPlanningFrame();
            std::string ee_frame = move_group_->getEndEffectorLink();
            std::string pose_frame = move_group_->getPoseReferenceFrame();
        
            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
            RCLCPP_INFO(this->get_logger(), "║           FRAME CONFIGURATION DEBUG                ║");
            RCLCPP_INFO(this->get_logger(), "╠════════════════════════════════════════════════════╣");
            RCLCPP_INFO(this->get_logger(), "║ Planning frame:      %-30s║", planning_frame.c_str());
            RCLCPP_INFO(this->get_logger(), "║ End effector frame:  %-30s║", ee_frame.c_str());
            RCLCPP_INFO(this->get_logger(), "║ Pose reference frame: %-29s║", pose_frame.c_str());
            RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
            RCLCPP_INFO(this->get_logger(), "");
            
            RCLCPP_INFO(this->get_logger(), "📍 What these frames mean:");
            RCLCPP_INFO(this->get_logger(), "   - Planning frame: The coordinate system MoveIt uses for planning");
            RCLCPP_INFO(this->get_logger(), "   - EE frame: The actual end-effector link being controlled");
            RCLCPP_INFO(this->get_logger(), "   - Pose reference: The frame that target poses are specified in");
            RCLCPP_INFO(this->get_logger(), "");

            auto current_pose = move_group_->getCurrentPose();
            
            RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
            RCLCPP_INFO(this->get_logger(), "║           CURRENT END-EFFECTOR POSE                ║");
            RCLCPP_INFO(this->get_logger(), "╠════════════════════════════════════════════════════╣");
            RCLCPP_INFO(this->get_logger(), "║ Frame: %-44s║", current_pose.header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "║ Position:                                          ║");
            RCLCPP_INFO(this->get_logger(), "║   X = %+.4f m                                    ║", current_pose.pose.position.x);
            RCLCPP_INFO(this->get_logger(), "║   Y = %+.4f m                                    ║", current_pose.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "║   Z = %+.4f m                                    ║", current_pose.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "║ Orientation (quaternion):                          ║");
            RCLCPP_INFO(this->get_logger(), "║   x = %+.4f                                      ║", current_pose.pose.orientation.x);
            RCLCPP_INFO(this->get_logger(), "║   y = %+.4f                                      ║", current_pose.pose.orientation.y);
            RCLCPP_INFO(this->get_logger(), "║   z = %+.4f                                      ║", current_pose.pose.orientation.z);
            RCLCPP_INFO(this->get_logger(), "║   w = %+.4f                                      ║", current_pose.pose.orientation.w);
            RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
            RCLCPP_INFO(this->get_logger(), "");
            
            // 🔒 CAPTURE INITIAL ORIENTATION AND POSITION
            locked_orientation_ = current_pose.pose.orientation;
            target_position_ = current_pose.pose.position;
            target_position_initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "🔒 Orientation LOCKED: [%.3f, %.3f, %.3f, %.3f]",
                locked_orientation_.x, locked_orientation_.y, 
                locked_orientation_.z, locked_orientation_.w);
            RCLCPP_INFO(this->get_logger(), "🎯 Target position INITIALIZED: [%.3f, %.3f, %.3f]",
                target_position_.x, target_position_.y, target_position_.z);
            RCLCPP_INFO(this->get_logger(), "");
            
            // ⚡ FASTER CONTROL LOOP - Changed from 100ms (10Hz) to 50ms (20Hz)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),  // 20Hz for faster response
                std::bind(&CartesianJoystickControl::controlLoop, this));
            
            RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
            RCLCPP_INFO(this->get_logger(), "║              AXIS CONTROL MAPPING                  ║");
            RCLCPP_INFO(this->get_logger(), "╠════════════════════════════════════════════════════╣");
            RCLCPP_INFO(this->get_logger(), "║ TRIGGER (0) + Stick Y = WORLD X axis (±)           ║");
            RCLCPP_INFO(this->get_logger(), "║ Button 2    + Stick X = WORLD Y axis (±)           ║");
            RCLCPP_INFO(this->get_logger(), "║ Button 3    + Stick Y = WORLD Z axis (±)           ║");
            RCLCPP_INFO(this->get_logger(), "║ Button 7              = Reset orientation lock     ║");
            RCLCPP_INFO(this->get_logger(), "╠════════════════════════════════════════════════════╣");
            RCLCPP_INFO(this->get_logger(), "║ Velocity Control:                                  ║");
            RCLCPP_INFO(this->get_logger(), "║   Max velocity: %.2f m/s (%.0f cm/s) ⚡            ║", max_velocity_, max_velocity_ * 100);
            RCLCPP_INFO(this->get_logger(), "║   Control frequency: %.0f Hz                        ║", control_frequency_);
            RCLCPP_INFO(this->get_logger(), "║   Stick magnitude controls speed (0%% to 100%%)     ║");
            RCLCPP_INFO(this->get_logger(), "╠════════════════════════════════════════════════════╣");
            RCLCPP_INFO(this->get_logger(), "║ World frame axes (in RViz):                        ║");
            RCLCPP_INFO(this->get_logger(), "║   X = RED arrow   (forward/backward)               ║");
            RCLCPP_INFO(this->get_logger(), "║   Y = GREEN arrow (left/right)                     ║");
            RCLCPP_INFO(this->get_logger(), "║   Z = BLUE arrow  (up/down)                        ║");
            RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "✅ READY TO CONTROL! ⚡ HIGH SPEED MODE ENABLED");
            RCLCPP_INFO(this->get_logger(), "");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        
        if (msg->axes.size() < 3 || msg->buttons.size() < 3) {
            return;
        }
        
        // Get stick inputs - ⚡ REDUCED DEADBAND for more sensitivity
        auto apply_deadband = [](double val, double threshold = 0.05) {  // Reduced from 0.1 to 0.05
            return (std::abs(val) < threshold) ? 0.0 : val;
        };
        
        double stick_x = apply_deadband(msg->axes[0]);   // Left/Right
        double stick_y = apply_deadband(-msg->axes[1]);  // Forward/Back (INVERTED!)
        
        // Reset all motion
        linear_x_ = 0.0;
        linear_y_ = 0.0;
        linear_z_ = 0.0;
        
        // TRIGGER (button 0) = X axis control
        if (msg->buttons[0]) {
            linear_x_ = stick_y;
            current_mode_ = "WORLD X (RED)";
            current_axis_ = "+Y stick → +X world, -Y stick → -X world";
        }
        // Button 2 (side button) = Y axis control
        else if (msg->buttons.size() > 2 && msg->buttons[2]) {
            linear_y_ = stick_x;
            current_mode_ = "WORLD Y (GREEN)";
            current_axis_ = "+X stick → +Y world, -X stick → -Y world";
        }
        // Button 3 = Z axis control
        else if (msg->buttons.size() > 3 && msg->buttons[3]) {
            linear_z_ = stick_y;
            current_mode_ = "WORLD Z (BLUE)";
            current_axis_ = "+Y stick → +Z world, -Y stick → -Z world";
        }
        else {
            current_mode_ = "IDLE";
            current_axis_ = "No button pressed";
        }
        
        // Button 7 = RESET orientation lock
        if (msg->buttons.size() > 7 && msg->buttons[7]) {
            reset_orientation_lock_ = true;
        }
    }
    
    void controlLoop()
    {
        if (!move_group_) return;
        
        // Check if we need to reset the orientation lock
        if (reset_orientation_lock_) {
            auto current_pose = move_group_->getCurrentPose();
            locked_orientation_ = current_pose.pose.orientation;
            target_position_ = current_pose.pose.position;  // Also reset position target
            target_position_initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "🔒 ═══════════════════════════════════");
            RCLCPP_INFO(this->get_logger(), "🔒  ORIENTATION & POSITION LOCK RESET!");
            RCLCPP_INFO(this->get_logger(), "🔒  New locked orientation:");
            RCLCPP_INFO(this->get_logger(), "🔒    [%.3f, %.3f, %.3f, %.3f]",
                locked_orientation_.x,
                locked_orientation_.y,
                locked_orientation_.z,
                locked_orientation_.w);
            RCLCPP_INFO(this->get_logger(), "🎯  Position target reset to: [%.3f, %.3f, %.3f]",
                target_position_.x, target_position_.y, target_position_.z);
            RCLCPP_INFO(this->get_logger(), "🔒 ═══════════════════════════════════");
            RCLCPP_INFO(this->get_logger(), "");
            reset_orientation_lock_ = false;
            return;
        }
        
        // Get joystick values
        double lx, ly, lz;
        std::string mode, axis;
        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            lx = linear_x_;
            ly = linear_y_;
            lz = linear_z_;
            mode = current_mode_;
            axis = current_axis_;
        }
        
        // Skip if no input
        if (lx == 0.0 && ly == 0.0 && lz == 0.0) {
            if (motion_active_) {
                RCLCPP_INFO(this->get_logger(), "⏹  Motion stopped");
                motion_active_ = false;
            }
            return;
        }
        
        try {
            move_group_->setStartStateToCurrentState();
            
            // Get current pose
            auto current_pose = move_group_->getCurrentPose().pose;
            
            // 🎯 SAFETY CHECK: Initialize or resync target position if too far from current
            if (!target_position_initialized_) {
                target_position_ = current_pose.position;
                target_position_initialized_ = true;
                RCLCPP_WARN(this->get_logger(), "⚠️  Target position initialized to current: [%.3f, %.3f, %.3f]",
                    target_position_.x, target_position_.y, target_position_.z);
            }
            
            // 🛡️ SAFETY: Check if target drifted too far from current position
            double distance_to_target = std::sqrt(
                std::pow(target_position_.x - current_pose.position.x, 2) +
                std::pow(target_position_.y - current_pose.position.y, 2) +
                std::pow(target_position_.z - current_pose.position.z, 2)
            );
            
            if (distance_to_target > 0.10) {  // If target is >10cm away, resync
                RCLCPP_WARN(this->get_logger(), "⚠️  Target position drifted %.3fm away! Resyncing to current position.", distance_to_target);
                target_position_ = current_pose.position;
            }
            
            // 🎯 UPDATE ONLY THE CONTROLLED AXIS IN TARGET POSITION
            // Calculate step size based on joystick magnitude and desired velocity
            double step_x = (lx != 0.0) ? (max_velocity_ / control_frequency_) * lx : 0.0;
            double step_y = (ly != 0.0) ? (max_velocity_ / control_frequency_) * ly : 0.0;
            double step_z = (lz != 0.0) ? (max_velocity_ / control_frequency_) * lz : 0.0;
            
            // Update target position
            target_position_.x += step_x;
            target_position_.y += step_y;
            target_position_.z += step_z;
            
            // Create target pose using our tracked target position
            geometry_msgs::msg::Pose target;
            target.position = target_position_;  // Use tracked target
            target.orientation = locked_orientation_;  // Keep orientation locked
            
            // Calculate deltas for logging
            double delta_x = target.position.x - current_pose.position.x;
            double delta_y = target.position.y - current_pose.position.y;
            double delta_z = target.position.z - current_pose.position.z;
            
            // Detailed logging
            if (!motion_active_) {
                RCLCPP_INFO(this->get_logger(), "");
                RCLCPP_INFO(this->get_logger(), "▶ ═══════════════════════════════════════════════");
                RCLCPP_INFO(this->get_logger(), "▶  MOTION STARTED ⚡");
                RCLCPP_INFO(this->get_logger(), "▶  Mode: %s", mode.c_str());
                RCLCPP_INFO(this->get_logger(), "▶  Mapping: %s", axis.c_str());
                RCLCPP_INFO(this->get_logger(), "▶ ═══════════════════════════════════════════════");
            }
            
            RCLCPP_INFO(this->get_logger(), 
                "📍 [%s] Current: [%.3f, %.3f, %.3f] → Target: [%.3f, %.3f, %.3f] (Δ: [%+.3f, %+.3f, %+.3f])",
                mode.c_str(),
                current_pose.position.x, current_pose.position.y, current_pose.position.z,
                target.position.x, target.position.y, target.position.z,
                delta_x, delta_y, delta_z);
            
            motion_active_ = true;
            
            // Create waypoints
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target);
            
            // Compute Cartesian path - ⚡ COARSER INTERPOLATION for speed
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.01;  // Changed from 0.005 to 0.01 (10mm - faster)
            const double jump_threshold = 0.0;
            
            double fraction = move_group_->computeCartesianPath(
                waypoints,
                eef_step,
                jump_threshold,
                trajectory,
                true
            );
            
            // ⚡ MORE LENIENT PATH ACCEPTANCE - Changed from 0.90 to 0.80
            if (fraction > 0.80) {  // Accept 80% instead of 90%
                auto result = move_group_->execute(trajectory);
                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    // Reduce logging spam for speed
                    // RCLCPP_INFO(this->get_logger(), "   ✓ Executed (%.1f%%)", fraction * 100.0);
                } else {
                    RCLCPP_WARN(this->get_logger(), "   ✗ Execution failed!");
                }
            } else if (fraction > 0.0) {
                RCLCPP_WARN(this->get_logger(), 
                    "   ⚠ Path quality: %.1f%% (need 80%%) - skipping", 
                    fraction * 100.0);
            } else {
                RCLCPP_ERROR(this->get_logger(), "   ✗ Cannot compute Cartesian path!");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error: %s", e.what());
        }
    }
    
    // MoveGroup components
    rclcpp::Node::SharedPtr move_group_node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    
    // ROS components
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    // State
    std::mutex joy_mutex_;
    double linear_x_ = 0.0, linear_y_ = 0.0, linear_z_ = 0.0;
    std::string current_mode_ = "IDLE";
    std::string current_axis_ = "No button pressed";
    bool motion_active_ = false;
    
    // 🔒 LOCKED ORIENTATION
    geometry_msgs::msg::Quaternion locked_orientation_;
    bool reset_orientation_lock_ = false;
    
    // 🎯 TARGET POSITION TRACKING (prevents drift)
    geometry_msgs::msg::Point target_position_;
    bool target_position_initialized_ = false;
    
    // ⚡ VELOCITY CONTROL - INCREASED SPEED
    const double max_velocity_ = 0.30;  // Changed from 0.10 to 0.30 m/s (30 cm/s - 3x faster!)
    const double control_frequency_ = 20.0;  // Changed from 10.0 to 20.0 Hz (faster updates)
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianJoystickControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
