#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>  // Add this for joint state subscription
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <thread>
#include <map>
#include <chrono>

class KeyboardServoNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr gripper_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; // Add this
    rclcpp::TimerBase::SharedPtr timer_;
    
    struct termios old_termios_;
    bool is_cartesian_mode_;
    
    // Key state tracking with timestamps
    std::map<char, std::chrono::steady_clock::time_point> key_timestamps_;
    static constexpr double KEY_TIMEOUT_MS = 200.0; // Stop after 200ms of no input for a specific key
    
    // Motion parameters
    static constexpr double LINEAR_SPEED = 0.1;   // m/s
    static constexpr double ANGULAR_SPEED = 0.5;  // rad/s
    static constexpr double JOINT_SPEED = 0.5;    // rad/s
    
    // Gripper parameters
    static constexpr double GRIPPER_OPEN_POSITION = 1.0;    // Absolute positions
    static constexpr double GRIPPER_CLOSED_POSITION = 0.0;  
    static constexpr double GRIPPER_SERVO_STEP = 0.05;      // Small incremental step
    
    // Track current gripper position
    double current_gripper_position_;

public:
    KeyboardServoNode() : Node("keyboard_servo_node"), is_cartesian_mode_(true), current_gripper_position_(0.0)
    {
        // Initialize publishers
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
        joint_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_node/delta_joint_cmds", 10);
        gripper_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_node/delta_gripper_cmds", 10);
        
        // Add direct gripper trajectory publisher
        gripper_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/hand_controller/joint_trajectory", 10);
            
        mode_publisher_ = this->create_publisher<std_msgs::msg::Int8>(
            "/switch_command", 10);
        
        // Add joint state subscription to track gripper position
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&KeyboardServoNode::jointStateCallback, this, std::placeholders::_1));
        
        // Set up timer for continuous publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&KeyboardServoNode::timerCallback, this));
        
        // Setup terminal for raw input
        setupTerminal();
        
        // Print instructions
        printInstructions();
        
        // Start input thread
        std::thread input_thread(&KeyboardServoNode::inputLoop, this);
        input_thread.detach();
        
        RCLCPP_INFO(this->get_logger(), "Keyboard Servo Node initialized with gripper control");
    }
    
    ~KeyboardServoNode()
    {
        restoreTerminal();
    }

private:
    // Add joint state callback to track gripper position
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Find joint_00 in the joint states
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "joint_00") {
                current_gripper_position_ = msg->position[i];
                break;
            }
        }
    }

    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &old_termios_);
        struct termios new_termios = old_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    
    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
    
    void printInstructions()
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Keyboard Servo Control for Robotic Arm");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Cartesian Control (End-Effector):");
        RCLCPP_INFO(this->get_logger(), "  w/s: Move forward/backward (X-axis)");
        RCLCPP_INFO(this->get_logger(), "  a/d: Move left/right (Y-axis)");
        RCLCPP_INFO(this->get_logger(), "  r/f: Move up/down (Z-axis)");
        RCLCPP_INFO(this->get_logger(), "  q/e: Rotate around X-axis");
        RCLCPP_INFO(this->get_logger(), "  t/g: Rotate around Y-axis");
        RCLCPP_INFO(this->get_logger(), "  y/h: Rotate around Z-axis");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Joint Control:");
        RCLCPP_INFO(this->get_logger(), "  1/2: Joint 1 +/-");
        RCLCPP_INFO(this->get_logger(), "  3/4: Joint 2 +/-");
        RCLCPP_INFO(this->get_logger(), "  5/6: Joint 3 +/-");
        RCLCPP_INFO(this->get_logger(), "  7/8: Joint 4 +/-");
        RCLCPP_INFO(this->get_logger(), "  9/0: Joint 5 +/-");
        RCLCPP_INFO(this->get_logger(), "  -/=: Gripper servo +/- (continuous)");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Gripper Control (Servo Style):");
        RCLCPP_INFO(this->get_logger(), "  o: Open gripper gradually (hold key)");
        RCLCPP_INFO(this->get_logger(), "  p: Close gripper gradually (hold key)");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Control:");
        RCLCPP_INFO(this->get_logger(), "  SPACE: Stop all motion");
        RCLCPP_INFO(this->get_logger(), "  c: Switch between Cartesian/Joint mode");
        RCLCPP_INFO(this->get_logger(), "  ESC: Exit");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Current mode: %s", is_cartesian_mode_ ? "Cartesian" : "Joint");
        RCLCPP_INFO(this->get_logger(), "Keyboard input ready. Press keys to control the robot.");
    }
    
    void inputLoop()
    {
        char key;
        while (rclcpp::ok())
        {
            if (read(STDIN_FILENO, &key, 1) == 1)
            {
                processKeyPress(key);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void processKeyPress(char key)
    {
        // Handle special keys
        if (key == 27) // ESC
        {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();
            return;
        }
        else if (key == ' ') // SPACE - stop all motion
        {
            clearAllKeyStates();
            RCLCPP_INFO(this->get_logger(), "Stopping all motion");
            return;
        }
        else if (key == 'c') // Switch mode
        {
            switchMode();
            return;
        }
        
        // Update timestamp for valid keys (including 'o' and 'p' for servo control)
        if (isValidKey(key))
        {
            key_timestamps_[key] = std::chrono::steady_clock::now();
        }
    }
    
    bool isValidKey(char key)
    {
        // Cartesian keys
        if (key == 'w' || key == 's' || key == 'a' || key == 'd' || 
            key == 'r' || key == 'f' || key == 'q' || key == 'e' || 
            key == 't' || key == 'g' || key == 'y' || key == 'h') {
            return true;
        }
        
        // Joint keys (including gripper servo)
        if (key == '1' || key == '2' || key == '3' || key == '4' || 
            key == '5' || key == '6' || key == '7' || key == '8' || 
            key == '9' || key == '0' || key == '-' || key == '=') {
            return true;
        }
        
        // Gripper servo keys
        if (key == 'o' || key == 'p') {
            return true;
        }
        
        return false;
    }
    
    bool isKeyActive(char key)
    {
        auto it = key_timestamps_.find(key);
        if (it == key_timestamps_.end()) {
            return false;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second);
        
        return duration.count() <= KEY_TIMEOUT_MS;
    }
    
    void timerCallback()
    {
        if (key_timestamps_.empty()) {
            return;  // Skip publishing entirely
        }
        
        // Generate and publish appropriate message based on mode
        if (is_cartesian_mode_)
        {
            auto twist_msg = generateTwistMessage();
            twist_publisher_->publish(twist_msg);
        }
        else
        {
            auto joint_msg = generateJointMessage();
            joint_publisher_->publish(joint_msg);
        }
        
        // Gripper servo control (like cartesian/joint servo)
        if (isKeyActive('o') || isKeyActive('p')) {
            auto gripper_servo_msg = generateGripperServoMessage();
            gripper_trajectory_pub_->publish(gripper_servo_msg);
        }
        
        // Regular gripper servo (-/= keys)
        if (isKeyActive('-') || isKeyActive('=')) {
            auto gripper_msg = generateGripperMessage();
            gripper_publisher_->publish(gripper_msg);
        }
    }
    
    // New function: Generate incremental gripper movements (like cartesian servo)
    trajectory_msgs::msg::JointTrajectory generateGripperServoMessage()
    {
        trajectory_msgs::msg::JointTrajectory gripper_msg;
        gripper_msg.joint_names = {"joint_00"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Calculate incremental movement based on active keys
        double increment = 0.0;
        if (isKeyActive('o')) increment = +GRIPPER_SERVO_STEP;   // Open incrementally
        if (isKeyActive('p')) increment = -GRIPPER_SERVO_STEP;  // Close incrementally
        
        // Calculate new position
        double new_position = current_gripper_position_ + increment;
        
        // Clamp to limits
        new_position = std::max(GRIPPER_CLOSED_POSITION, std::min(GRIPPER_OPEN_POSITION, new_position));
        
        point.positions = {new_position};
        point.time_from_start = rclcpp::Duration::from_seconds(0.1); // Fast execution for servo
        
        gripper_msg.points = {point};
        
        // Optional debug output
        if (increment != 0.0) {
            RCLCPP_DEBUG(this->get_logger(), "Gripper servo: %.3f -> %.3f (increment: %.3f)", 
                        current_gripper_position_, new_position, increment);
        }
        
        return gripper_msg;
    }
    
    void clearAllKeyStates()
    {
        key_timestamps_.clear();
    }
    
    void switchMode()
    {
        is_cartesian_mode_ = !is_cartesian_mode_;
        
        auto mode_msg = std_msgs::msg::Int8();
        mode_msg.data = is_cartesian_mode_ ? 1 : 0;
        mode_publisher_->publish(mode_msg);
        
        RCLCPP_INFO(this->get_logger(), "Switched to %s mode", 
                   is_cartesian_mode_ ? "Cartesian" : "Joint");
    }
    
    geometry_msgs::msg::TwistStamped generateTwistMessage()
    {
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.header.frame_id = "";
        
        // Initialize all values to zero
        twist_msg.twist.linear.x = 0.0;
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;
        
        // Cartesian control - check if keys are currently active
        if (isKeyActive('w')) twist_msg.twist.linear.x = LINEAR_SPEED;
        if (isKeyActive('s')) twist_msg.twist.linear.x = -LINEAR_SPEED;
        if (isKeyActive('a')) twist_msg.twist.linear.y = LINEAR_SPEED;
        if (isKeyActive('d')) twist_msg.twist.linear.y = -LINEAR_SPEED;
        if (isKeyActive('r')) twist_msg.twist.linear.z = LINEAR_SPEED;
        if (isKeyActive('f')) twist_msg.twist.linear.z = -LINEAR_SPEED;
        if (isKeyActive('q')) twist_msg.twist.angular.x = ANGULAR_SPEED;
        if (isKeyActive('e')) twist_msg.twist.angular.x = -ANGULAR_SPEED;
        if (isKeyActive('t')) twist_msg.twist.angular.y = ANGULAR_SPEED;
        if (isKeyActive('g')) twist_msg.twist.angular.y = -ANGULAR_SPEED;
        if (isKeyActive('y')) twist_msg.twist.angular.z = ANGULAR_SPEED;
        if (isKeyActive('h')) twist_msg.twist.angular.z = -ANGULAR_SPEED;
        
        return twist_msg;
    }
    
    control_msgs::msg::JointJog generateJointMessage()
    {
        control_msgs::msg::JointJog joint_msg;
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.header.frame_id = "";
        
        // Initialize joint names and velocities for 5 joints (arm only, no gripper)
        joint_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};
        joint_msg.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Joint control - check if keys are currently active (joints 1-5 only)
        if (isKeyActive('1')) joint_msg.velocities[0] = JOINT_SPEED;
        if (isKeyActive('2')) joint_msg.velocities[0] = -JOINT_SPEED;
        if (isKeyActive('3')) joint_msg.velocities[1] = JOINT_SPEED;
        if (isKeyActive('4')) joint_msg.velocities[1] = -JOINT_SPEED;
        if (isKeyActive('5')) joint_msg.velocities[2] = JOINT_SPEED;
        if (isKeyActive('6')) joint_msg.velocities[2] = -JOINT_SPEED;
        if (isKeyActive('7')) joint_msg.velocities[3] = JOINT_SPEED;
        if (isKeyActive('8')) joint_msg.velocities[3] = -JOINT_SPEED;
        if (isKeyActive('9')) joint_msg.velocities[4] = JOINT_SPEED;
        if (isKeyActive('0')) joint_msg.velocities[4] = -JOINT_SPEED;

        return joint_msg;
    }
    
    control_msgs::msg::JointJog generateGripperMessage()
    {
        control_msgs::msg::JointJog gripper_msg;
        gripper_msg.header.stamp = this->get_clock()->now();
        gripper_msg.header.frame_id = "";
        
        // Initialize gripper joint for servo control
        gripper_msg.joint_names = {"joint_00"}; // Your gripper joint name
        gripper_msg.velocities = {0.0};
        
        // Gripper servo control - check if keys are currently active
        if (isKeyActive('-')) gripper_msg.velocities[0] = JOINT_SPEED;   // Open gripper (servo)
        if (isKeyActive('=')) gripper_msg.velocities[0] = -JOINT_SPEED;  // Close gripper (servo)
        
        return gripper_msg;
    }
};

// Signal handler for clean shutdown
KeyboardServoNode* g_node = nullptr;
void signalHandler(int sig)
{
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", sig);
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    auto node = std::make_shared<KeyboardServoNode>();
    g_node = node.get();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

