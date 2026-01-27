#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

class JoystickCartesianTeleop : public rclcpp::Node
{
public:
    using MoveGroup = moveit_msgs::action::MoveGroup;
    using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

    JoystickCartesianTeleop() : Node("joystick_cartesian_teleop")
    {
        // Parameters
        this->declare_parameter("planning_group", "arm_group");
        this->declare_parameter("end_effector_link", "fake_link");
        this->declare_parameter("base_frame", "world");
        //this->declare_parameter("use_sim_time", true);
        
        planning_group_ = this->get_parameter("planning_group").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        
        step_size_linear_ = 0.01;  // 1cm
        step_size_angular_ = 0.05; // ~3 degrees
        update_rate_ = 10.0;       // Hz
        deadzone_ = 0.15;
        
        // State
        deadman_pressed_ = false;
        gripper_open_ = true;
        last_button_1_state_ = 0;
        goal_in_progress_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Initializing joystick cartesian teleop...");
        
        // TF2 for getting current end effector pose
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // MoveGroup Action Client (same as RViz uses)
        move_group_client_ = rclcpp_action::create_client<MoveGroup>(
            this, "/move_action");
        
        // Gripper publisher
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gripper_controller/commands", 10);
        
        // Joystick subscriber
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoystickCartesianTeleop::joyCallback, this, std::placeholders::_1));
        
        // Timer for periodic updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            std::bind(&JoystickCartesianTeleop::updateCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Joystick Cartesian Teleop Node Created ===");
        RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector: %s", end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for MoveGroup action server...");
        
        // Wait for action server
        if (!move_group_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup action server not available!");
        } else {
            RCLCPP_INFO(this->get_logger(), "=== Ready! ===");
            RCLCPP_INFO(this->get_logger(), "HOLD TRIGGER (button 0) to enable motion");
        }
    }
    
private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        joy_msg_ = msg;
        
        // Deadman switch (trigger - button 0)
        if (msg->buttons.size() > 0) {
            bool was_pressed = deadman_pressed_;
            deadman_pressed_ = msg->buttons[0] == 1;
            
            if (deadman_pressed_ && !was_pressed) {
                RCLCPP_INFO(this->get_logger(), "DEADMAN ACTIVE - Motion enabled");
            } else if (!deadman_pressed_ && was_pressed) {
                RCLCPP_INFO(this->get_logger(), "DEADMAN RELEASED - Motion disabled");
            }
        }
        
        // Gripper toggle (button 1)
        if (msg->buttons.size() > 1) {
            if (msg->buttons[1] == 1 && last_button_1_state_ == 0) {
                toggleGripper();
            }
            last_button_1_state_ = msg->buttons[1];
        }
        
        // Speed adjustment (buttons 2 and 3)
        if (msg->buttons.size() > 2 && msg->buttons[2] == 1) {
            step_size_linear_ *= 1.2;
            step_size_angular_ *= 1.2;
            RCLCPP_INFO(this->get_logger(), "Speed UP: %.4f m/s", step_size_linear_);
        }
        
        if (msg->buttons.size() > 3 && msg->buttons[3] == 1) {
            step_size_linear_ *= 0.8;
            step_size_angular_ *= 0.8;
            RCLCPP_INFO(this->get_logger(), "Speed DOWN: %.4f m/s", step_size_linear_);
        }
    }
    
    void toggleGripper()
    {
        gripper_open_ = !gripper_open_;
        
        auto msg = std_msgs::msg::Float64MultiArray();
        if (gripper_open_) {
            msg.data = {0.0};  // Open
            RCLCPP_INFO(this->get_logger(), "Gripper: OPEN");
        } else {
            msg.data = {0.8};  // Closed
            RCLCPP_INFO(this->get_logger(), "Gripper: CLOSED");
        }
        
        gripper_pub_->publish(msg);
    }
    
    double applyDeadzone(double value)
    {
        if (std::abs(value) < deadzone_) {
            return 0.0;
        }
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone_) / (1.0 - deadzone_);
    }
    
    geometry_msgs::msg::PoseStamped getCurrentEndEffectorPose()
    {
        geometry_msgs::msg::PoseStamped current_pose;
        
        try {
            auto transform = tf_buffer_->lookupTransform(
                base_frame_, end_effector_link_, tf2::TimePointZero);
            
            current_pose.header = transform.header;
            current_pose.pose.position.x = transform.transform.translation.x;
            current_pose.pose.position.y = transform.transform.translation.y;
            current_pose.pose.position.z = transform.transform.translation.z;
            current_pose.pose.orientation = transform.transform.rotation;
            
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Could not get transform: %s", ex.what());
        }
        
        return current_pose;
    }
    
    void updateCallback()
    {
        if (!deadman_pressed_ || !joy_msg_) {
            return;
        }
        
        if (joy_msg_->axes.size() < 4) {
            return;
        }
        
        // Don't send new goals if one is already in progress
        if (goal_in_progress_) {
            return;
        }
        
        // Apply deadzone to joystick axes
        double joy_x = applyDeadzone(joy_msg_->axes[0]);      // Left/Right
        double joy_y = applyDeadzone(joy_msg_->axes[1]);      // Forward/Back
        double joy_z = applyDeadzone(joy_msg_->axes[2]);      // Twist
        double throttle = (joy_msg_->axes.size() > 3) ? joy_msg_->axes[3] : 0.0;
        
        // Check if any significant motion
        if (std::abs(joy_x) < 0.01 && std::abs(joy_y) < 0.01 && 
            std::abs(joy_z) < 0.01 && std::abs(throttle) < 0.01) {
            return;
        }
        
        // Get current end effector pose
        auto current_pose = getCurrentEndEffectorPose();
        if (current_pose.header.frame_id.empty()) {
            return;  // Failed to get transform
        }
        
        // Calculate Cartesian deltas
        double delta_x = -joy_y * step_size_linear_;  // Forward/back
        double delta_y = joy_x * step_size_linear_;   // Left/right
        double delta_z = joy_z * step_size_linear_;   // Up/down
        double delta_rotation = -throttle * step_size_angular_;
        
        // Create target pose
        geometry_msgs::msg::PoseStamped target_pose = current_pose;
        target_pose.pose.position.x += delta_x;
        target_pose.pose.position.y += delta_y;
        target_pose.pose.position.z += delta_z;
        
        // Handle orientation
        if (std::abs(delta_rotation) > 0.001) {
            tf2::Quaternion current_quat(
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w
            );
            
            tf2::Quaternion delta_quat;
            delta_quat.setRPY(0, 0, delta_rotation);
            
            tf2::Quaternion new_quat = current_quat * delta_quat;
            new_quat.normalize();
            
            target_pose.pose.orientation.x = new_quat.x();
            target_pose.pose.orientation.y = new_quat.y();
            target_pose.pose.orientation.z = new_quat.z();
            target_pose.pose.orientation.w = new_quat.w();
        }
        
        // Send goal to MoveGroup
        sendPoseGoal(target_pose);
    }
    
    void sendPoseGoal(const geometry_msgs::msg::PoseStamped& target_pose)
    {
        if (!move_group_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "MoveGroup action server not ready");
            return;
        }
        
        // Create goal message - exactly like RViz does
        auto goal_msg = MoveGroup::Goal();
        
        // Motion plan request
        goal_msg.request.group_name = planning_group_;
        goal_msg.request.num_planning_attempts = 3;
        goal_msg.request.allowed_planning_time = 0.5;
        goal_msg.request.max_velocity_scaling_factor = 0.3;
        goal_msg.request.max_acceleration_scaling_factor = 0.3;
        
        // Workspace
        goal_msg.request.workspace_parameters.header.frame_id = base_frame_;
        goal_msg.request.workspace_parameters.min_corner.x = -2.0;
        goal_msg.request.workspace_parameters.min_corner.y = -2.0;
        goal_msg.request.workspace_parameters.min_corner.z = -2.0;
        goal_msg.request.workspace_parameters.max_corner.x = 2.0;
        goal_msg.request.workspace_parameters.max_corner.y = 2.0;
        goal_msg.request.workspace_parameters.max_corner.z = 2.0;
        
        // Goal constraints - pose constraint
        moveit_msgs::msg::Constraints goal_constraint;
        
        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header = target_pose.header;
        position_constraint.link_name = end_effector_link_;
        
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        box.dimensions.resize(1);
        box.dimensions[0] = 0.001;  // 1mm tolerance
        
        position_constraint.constraint_region.primitives.push_back(box);
        position_constraint.constraint_region.primitive_poses.push_back(target_pose.pose);
        position_constraint.weight = 1.0;
        
        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header = target_pose.header;
        orientation_constraint.link_name = end_effector_link_;
        orientation_constraint.orientation = target_pose.pose.orientation;
        orientation_constraint.absolute_x_axis_tolerance = 0.01;
        orientation_constraint.absolute_y_axis_tolerance = 0.01;
        orientation_constraint.absolute_z_axis_tolerance = 0.01;
        orientation_constraint.weight = 1.0;
        
        goal_constraint.position_constraints.push_back(position_constraint);
        goal_constraint.orientation_constraints.push_back(orientation_constraint);
        
        goal_msg.request.goal_constraints.push_back(goal_constraint);
        
        // Planning options
        goal_msg.planning_options.plan_only = false;  // Plan AND execute
        goal_msg.planning_options.planning_scene_diff.is_diff = true;
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
        
        // Send goal with callbacks
        auto send_goal_options = rclcpp_action::Client<MoveGroup>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](const GoalHandleMoveGroup::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_WARN(this->get_logger(), "Goal rejected");
                    goal_in_progress_ = false;
                } else {
                    goal_in_progress_ = true;
                }
            };
        
        send_goal_options.result_callback =
            [this](const GoalHandleMoveGroup::WrappedResult & result) {
                goal_in_progress_ = false;
                
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_DEBUG(this->get_logger(), "Motion succeeded");
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                       "Motion planning failed");
                }
            };
        
        move_group_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // Members
    rclcpp_action::Client<MoveGroup>::SharedPtr move_group_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    sensor_msgs::msg::Joy::SharedPtr joy_msg_;
    
    std::string planning_group_;
    std::string end_effector_link_;
    std::string base_frame_;
    double step_size_linear_;
    double step_size_angular_;
    double update_rate_;
    double deadzone_;
    
    bool deadman_pressed_;
    bool gripper_open_;
    int last_button_1_state_;
    bool goal_in_progress_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickCartesianTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
