#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

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
            
            // MAXIMUM SPEED settings! 🚀
            move_group_->setMaxVelocityScalingFactor(1.0);      // MAX!
            move_group_->setMaxAccelerationScalingFactor(1.0);  // MAX!
            
            RCLCPP_INFO(this->get_logger(), "MoveGroup initialized");
            
            rclcpp::sleep_for(std::chrono::seconds(1));
        
        
        // SET TO WORLD FRAME - THIS WAS THE FIX!
        move_group_->setPoseReferenceFrame("world");
        
            rclcpp::sleep_for(std::chrono::seconds(1));
            std::string planning_frame = move_group_->getPlanningFrame();
            std::string ee_frame = move_group_->getEndEffectorLink();
            std::string pose_frame = move_group_->getPoseReferenceFrame();
        
        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", planning_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector frame: %s", ee_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "Pose reference frame: %s", pose_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "==============================================");

            auto current_pose = move_group_->getCurrentPose();
            RCLCPP_INFO(this->get_logger(), "Current pose: [%.3f, %.3f, %.3f]",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
            
            // ULTRA FAST control loop - 20 Hz! 🚀💨
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),  // Was 100ms, now 50ms for 20Hz!
                std::bind(&CartesianJoystickControl::controlLoop, this));
            
            RCLCPP_INFO(this->get_logger(), "==============================================");
            RCLCPP_INFO(this->get_logger(), "   LOGITECH EXTREME 3D PRO - ULTRA FAST 🚀💨");
            RCLCPP_INFO(this->get_logger(), "==============================================");
            RCLCPP_INFO(this->get_logger(), "TRIGGER (Button 0) + Stick = X axis");
            RCLCPP_INFO(this->get_logger(), "Button 2           + Stick = Y axis");
            RCLCPP_INFO(this->get_logger(), "Button 3           + Stick = Z axis");
            RCLCPP_INFO(this->get_logger(), "Button 4           + Twist = ROLL");
            RCLCPP_INFO(this->get_logger(), "Button 5           + Stick = PITCH");
            RCLCPP_INFO(this->get_logger(), "Button 6           + Twist = YAW");
            RCLCPP_INFO(this->get_logger(), "==============================================");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joy_mutex_);
    
    if (msg->axes.size() < 3 || msg->buttons.size() < 7) {
        return;
    }
    
    // Get stick inputs with smaller deadband for more responsiveness
    auto apply_deadband = [](double val, double threshold = 0.05) {  // Smaller deadband
        return (std::abs(val) < threshold) ? 0.0 : val;
    };
    
    double stick_x = apply_deadband(msg->axes[0]);   // Left/Right
    double stick_y = apply_deadband(-msg->axes[1]);  // Forward/Back (INVERTED!)
    double twist = apply_deadband(msg->axes[2]);     // Twist
    
    // Reset all motion
    linear_x_ = 0.0;
    linear_y_ = 0.0;
    linear_z_ = 0.0;
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
    
    // TRIGGER (button 0) = X axis control
    if (msg->buttons[0]) {
        linear_x_ = stick_y * linear_scale_;  // Forward/back controls X
        current_mode_ = "X AXIS";
    }
    // Button 2 (side button) = Y axis control
    else if (msg->buttons[2]) {
        linear_y_ = stick_x * linear_scale_;  // Left/right controls Y
        current_mode_ = "Y AXIS";
    }
    // Button 3 = Z axis control
    else if (msg->buttons[3]) {
        linear_z_ = stick_y * linear_scale_;  // Forward/back controls Z (up/down)
        current_mode_ = "Z AXIS";
    }
    // Button 4 = ROLL control
    else if (msg->buttons[4]) {
        roll_ = twist * angular_scale_;  // Twist controls roll
        current_mode_ = "ROLL";
    }
    // Button 5 = PITCH control
    else if (msg->buttons[5]) {
        pitch_ = stick_y * angular_scale_;  // Forward/back controls pitch
        current_mode_ = "PITCH";
    }
    // Button 6 = YAW control
    else if (msg->buttons[6]) {
        yaw_ = twist * angular_scale_;  // Twist controls yaw
        current_mode_ = "YAW";
    }
    else {
        current_mode_ = "IDLE";
    }
}
    
    void controlLoop()
    {
        if (!move_group_) return;
        
        // Get joystick values
        double lx, ly, lz, r, p, y;
        std::string mode;
        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            lx = linear_x_;
            ly = linear_y_;
            lz = linear_z_;
            r = roll_;
            p = pitch_;
            y = yaw_;
            mode = current_mode_;
        }
        
        // Skip if no input
        if (lx == 0.0 && ly == 0.0 && lz == 0.0 && r == 0.0 && p == 0.0 && y == 0.0) {
            return;
        }
        
        try {
            move_group_->setStartStateToCurrentState();
            
            // Get current pose
            auto current_pose = move_group_->getCurrentPose().pose;
            
            // Create target pose
            geometry_msgs::msg::Pose target = current_pose;
            
            // Update position
            target.position.x += lx * step_size_;
            target.position.y += ly * step_size_;
            target.position.z += lz * step_size_;
            
            // Update orientation
            if (r != 0.0 || p != 0.0 || y != 0.0) {
                tf2::Quaternion q_current;
                tf2::fromMsg(current_pose.orientation, q_current);
                
                tf2::Quaternion q_delta;
                q_delta.setRPY(r * rotation_step_, p * rotation_step_, y * rotation_step_);
                
                tf2::Quaternion q_new = q_current * q_delta;
                q_new.normalize();
                
                target.orientation = tf2::toMsg(q_new);
            }
            
            RCLCPP_INFO(this->get_logger(), 
                "[%s] Moving: pos[%.3f, %.3f, %.3f] rot[%.2f, %.2f, %.2f]",
                mode.c_str(),
                lx * step_size_, ly * step_size_, lz * step_size_,
                r * rotation_step_, p * rotation_step_, y * rotation_step_);
            
            // Create waypoints vector
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target);
            
            // Compute Cartesian path - BIGGER steps for maximum speed
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.03;  // Was 0.02, now 3cm interpolation for speed!
            const double jump_threshold = 0.0;
            
            double fraction = move_group_->computeCartesianPath(
                waypoints,
                eef_step,
                jump_threshold,
                trajectory,
                true
            );
            
            // EVEN LOWER threshold for maximum acceptance
            if (fraction > 0.70) {  // Was 0.80, now 0.70 for more aggressive movement
                auto exec_result = move_group_->execute(trajectory);
                
                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "✓");
                }
            } else if (fraction > 0.0) {
                RCLCPP_WARN(this->get_logger(), "⚠ Only %.1f%%", fraction * 100.0);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
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
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    std::string current_mode_ = "IDLE";
    
    // ULTRA TURBO MODE! 🚀💨
    const double linear_scale_ = 1.0;        // Was 0.5, now MAX INPUT!
    const double angular_scale_ = 1.5;       // Was 1.0, now 1.5x!
    const double step_size_ = 0.15;          // Was 0.10m, now 15cm steps!
    const double rotation_step_ = 0.3;       // Was 0.2, now ~17 degrees!
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianJoystickControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
