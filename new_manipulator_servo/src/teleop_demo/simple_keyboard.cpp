#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class SimpleKeyboardNode : public rclcpp::Node
{
public:
    SimpleKeyboardNode() : Node("simple_keyboard_node")
    {
        // Create publisher
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/servo_node/twist", 10);
        
        // Setup terminal
        setupTerminal();
        
        // Create timer to check for keyboard input
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SimpleKeyboardNode::checkKeyboard, this));
        
        // Initialize message
        twist_msg_.linear.x = 0.0;
        twist_msg_.linear.y = 0.0;
        twist_msg_.linear.z = 0.0;
        twist_msg_.angular.x = 0.0;
        twist_msg_.angular.y = 0.0;
        twist_msg_.angular.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Simple Keyboard Node Started!");
        RCLCPP_INFO(this->get_logger(), "Press w/s/a/d/q/e keys, space to stop, ESC to exit");
    }
    
    ~SimpleKeyboardNode()
    {
        restoreTerminal();
    }

private:
    void setupTerminal()
    {
        // Save original terminal settings
        tcgetattr(STDIN_FILENO, &original_termios_);
        
        // Set new terminal settings
        struct termios raw = original_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        raw.c_cc[VMIN] = 0;               // Non-blocking read
        raw.c_cc[VTIME] = 0;              // No timeout
        
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        
        // Make stdin non-blocking
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        
        RCLCPP_INFO(this->get_logger(), "Terminal setup complete - raw mode enabled");
    }
    
    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        RCLCPP_INFO(this->get_logger(), "Terminal restored");
    }
    
    void checkKeyboard()
    {
        char key;
        
        // Reset all velocities first
        twist_msg_.linear.x = 0.0;
        twist_msg_.linear.y = 0.0;
        twist_msg_.linear.z = 0.0;
        twist_msg_.angular.x = 0.0;
        twist_msg_.angular.y = 0.0;
        twist_msg_.angular.z = 0.0;
        
        // Try to read a character
        if (read(STDIN_FILENO, &key, 1) > 0) {
            RCLCPP_INFO(this->get_logger(), "Key detected: '%c' (ASCII: %d)", key, (int)key);
            
            // Process the key
            switch (key) {
                case 'w':
                    twist_msg_.linear.x = 0.1;
                    RCLCPP_INFO(this->get_logger(), "Moving forward (w)");
                    break;
                case 's':
                    twist_msg_.linear.x = -0.1;
                    RCLCPP_INFO(this->get_logger(), "Moving backward (s)");
                    break;
                case 'a':
                    twist_msg_.linear.y = 0.1;
                    RCLCPP_INFO(this->get_logger(), "Moving left (a)");
                    break;
                case 'd':
                    twist_msg_.linear.y = -0.1;
                    RCLCPP_INFO(this->get_logger(), "Moving right (d)");
                    break;
                case 'q':
                    twist_msg_.angular.z = 0.5;
                    RCLCPP_INFO(this->get_logger(), "Rotating left (q)");
                    break;
                case 'e':
                    twist_msg_.angular.z = -0.5;
                    RCLCPP_INFO(this->get_logger(), "Rotating right (e)");
                    break;
                case ' ':
                    // All velocities already set to 0
                    RCLCPP_INFO(this->get_logger(), "STOP (space)");
                    break;
                case 27: // ESC key
                    RCLCPP_INFO(this->get_logger(), "ESC pressed - shutting down");
                    rclcpp::shutdown();
                    return;
                default:
                    RCLCPP_INFO(this->get_logger(), "Unknown key: '%c'", key);
                    break;
            }
        }
        
        // Always publish the twist message (even if it's all zeros)
        twist_pub_->publish(twist_msg_);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_msg_;
    struct termios original_termios_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SimpleKeyboardNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}
