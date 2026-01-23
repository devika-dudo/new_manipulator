#!/usr/bin/env python3
"""
Joystick Cartesian Teleoperation Node for 5-DOF Arm
Uses MoveIt Commander for Cartesian control

Logitech Extreme 3D Pro Joystick Mapping:
- X-axis (left/right): End-effector Y movement
- Y-axis (forward/back): End-effector X movement
- Z-axis (twist): End-effector Z movement
- Throttle slider: End-effector rotation around Z-axis
- Trigger (Button 0): Deadman switch (MUST be held for motion)
- Button 1 (side thumb): Gripper toggle (open/close)
- Button 2: Increase step size
- Button 3: Decrease step size
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
import moveit_commander
import sys
from scipy.spatial.transform import Rotation as R


class JoystickCartesianTeleop(Node):
    def __init__(self):
        super().__init__('joystick_cartesian_teleop')
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Parameters
        self.planning_group = 'arm_group'
        self.step_size_linear = 0.01  # 1cm per update
        self.step_size_angular = 0.05  # ~3 degrees per update
        self.update_rate = 10.0  # Hz (reduced for stability)
        self.deadzone = 0.15
        
        # State variables
        self.deadman_pressed = False
        self.gripper_open = True
        self.last_button_1_state = 0
        
        # Initialize MoveIt
        self.get_logger().info('Initializing MoveIt Commander...')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander(self.planning_group)
        
        # Set planning parameters
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        self.arm_group.set_planning_time(1.0)
        
        self.get_logger().info(f'Planning group: {self.planning_group}')
        self.get_logger().info(f'End effector: {self.arm_group.get_end_effector_link()}')
        
        # Get current pose
        self.current_pose = self.arm_group.get_current_pose().pose
        
        # Publisher for gripper commands
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )
        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Timer for periodic Cartesian updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_callback)
        
        # Joystick state
        self.joy_msg = None
        
        self.get_logger().info('=== Joystick Cartesian Teleop Started! ===')
        self.get_logger().info('HOLD TRIGGER (button 0) to enable motion')
        self.get_logger().info('Move joystick to control end-effector')
        
    def apply_deadzone(self, value):
        """Apply deadzone to joystick axis"""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg):
        """Store latest joystick message"""
        self.joy_msg = msg
        
        if len(msg.buttons) > 0:
            was_pressed = self.deadman_pressed
            self.deadman_pressed = msg.buttons[0] == 1
            if self.deadman_pressed and not was_pressed:
                self.get_logger().info('DEADMAN ACTIVE - Motion enabled')
            elif not self.deadman_pressed and was_pressed:
                self.get_logger().info('DEADMAN RELEASED - Motion disabled')
            
        if len(msg.buttons) > 1:
            if msg.buttons[1] == 1 and self.last_button_1_state == 0:
                self.toggle_gripper()
            self.last_button_1_state = msg.buttons[1]
            
        if len(msg.buttons) > 2 and msg.buttons[2] == 1:
            self.step_size_linear *= 1.2
            self.step_size_angular *= 1.2
            self.get_logger().info(f'Speed UP: {self.step_size_linear:.4f} m/s')
            
        if len(msg.buttons) > 3 and msg.buttons[3] == 1:
            self.step_size_linear *= 0.8
            self.step_size_angular *= 0.8
            self.get_logger().info(f'Speed DOWN: {self.step_size_linear:.4f} m/s')
    
    def toggle_gripper(self):
        """Toggle gripper between open and closed"""
        self.gripper_open = not self.gripper_open
        
        msg = Float64MultiArray()
        if self.gripper_open:
            msg.data = [0.0]  # Open position
            self.get_logger().info('Gripper: OPEN')
        else:
            msg.data = [0.8]  # Closed position
            self.get_logger().info('Gripper: CLOSED')
            
        self.gripper_pub.publish(msg)
    
    def update_callback(self):
        """Periodic callback to compute and execute Cartesian motion"""
        if not self.deadman_pressed or self.joy_msg is None:
            return
        
        if len(self.joy_msg.axes) < 4:
            return
            
        # Apply deadzone to joystick axes
        joy_x = self.apply_deadzone(self.joy_msg.axes[0])
        joy_y = self.apply_deadzone(self.joy_msg.axes[1])
        joy_z = self.apply_deadzone(self.joy_msg.axes[2])
        throttle = self.joy_msg.axes[3] if len(self.joy_msg.axes) > 3 else 0.0
        
        # Check if any significant motion
        if abs(joy_x) < 0.01 and abs(joy_y) < 0.01 and abs(joy_z) < 0.01 and abs(throttle) < 0.01:
            return
        
        # Get current pose
        current = self.arm_group.get_current_pose().pose
        
        # Calculate Cartesian deltas
        delta_x = -joy_y * self.step_size_linear  # Forward/back
        delta_y = joy_x * self.step_size_linear   # Left/right
        delta_z = joy_z * self.step_size_linear   # Up/down
        delta_rotation = -throttle * self.step_size_angular
        
        # Create target pose
        target_pose = Pose()
        target_pose.position.x = current.position.x + delta_x
        target_pose.position.y = current.position.y + delta_y
        target_pose.position.z = current.position.z + delta_z
        
        # Handle rotation
        if abs(delta_rotation) > 0.001:
            # Convert current quaternion to rotation
            current_quat = [
                current.orientation.x,
                current.orientation.y,
                current.orientation.z,
                current.orientation.w
            ]
            current_rot = R.from_quat(current_quat)
            
            # Create rotation delta around Z axis
            delta_rot = R.from_euler('z', delta_rotation)
            
            # Combine rotations
            new_rot = current_rot * delta_rot
            new_quat = new_rot.as_quat()
            
            target_pose.orientation.x = new_quat[0]
            target_pose.orientation.y = new_quat[1]
            target_pose.orientation.z = new_quat[2]
            target_pose.orientation.w = new_quat[3]
        else:
            # Keep same orientation
            target_pose.orientation = current.orientation
        
        # Execute motion
        self.execute_cartesian_move(target_pose)
        
    def execute_cartesian_move(self, target_pose):
        """Execute Cartesian move to target pose"""
        try:
            # Set pose target
            self.arm_group.set_pose_target(target_pose)
            
            # Plan and execute
            success = self.arm_group.go(wait=False)  # Non-blocking
            
            if not success:
                self.get_logger().warn('Motion failed - target unreachable')
                
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JoystickCartesianTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
