#!/usr/bin/env python3
"""
3-Mode Switcher:
1. PWM mode: Direct motor control
2. PID/Trajectory mode: MoveIt sends trajectories
3. Servo mode: Cartesian control sends direct joint commands
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ModeSwitch(Node):
    def __init__(self):
        super().__init__('mode_switch_handler')
        
        # QoS profiles
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Listen to encoder positions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states_teensy', 
            self.joint_state_callback,
            qos_best_effort)
        
        # Listen to servo mode changes
        self.servo_mode_sub = self.create_subscription(
            Bool, '/servo_mode', self.servo_mode_callback, qos_reliable)
        
        # Listen to control mode changes (for PWM vs PID)
        self.control_mode_sub = self.create_subscription(
            String, '/control_mode', self.control_mode_callback, qos_reliable)
        
        # Connect to trajectory controller
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory')
        
        self.current_positions = [0.0] * 6
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 
                           'joint_4', 'joint_5', 'fake_joint']
        
        self.servo_mode = False
        self.teensy_mode = 'PID_CONTROL'  # Track Teensy state
        
        self.get_logger().info('🎮 3-Mode Handler Ready')
        self.get_logger().info('   PWM Mode: Direct motor control')
        self.get_logger().info('   PID Mode: Trajectory controller active')
        self.get_logger().info('   Servo Mode: Cartesian control active')
    
    def joint_state_callback(self, msg):
        """Track current encoder positions"""
        if len(msg.position) >= 6:
            self.current_positions = list(msg.position[:6])
    
    def servo_mode_callback(self, msg):
        """
        Servo mode switch:
        - True: Servo takes control (Teensy stays in PID but hardware interface stops publishing)
        - False: Return to previous mode (trajectory or PWM)
        """
        new_servo = msg.data
        
        if new_servo == self.servo_mode:
            return
        
        self.servo_mode = new_servo
        
        if self.servo_mode:
            # ========== ENTERING SERVO MODE ==========
            self.get_logger().info('🎮 SERVO MODE ON')
            self.get_logger().info('   Teensy: PID mode (servo sends joint commands)')
            self.get_logger().info('   Hardware interface: STOPPED publishing')
            
            # Teensy should be in PID mode to accept joint commands from servo
            # Hardware interface will stop publishing in write() when servo_mode=True
            
        else:
            # ========== EXITING SERVO MODE ==========
            self.get_logger().info('🤖 SERVO MODE OFF - Returning to trajectory mode')
            
            # Sync trajectory controller to current position
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = self.current_positions
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=0, nanosec=500000000)
            
            goal.trajectory.points = [point]
            
            self.trajectory_client.send_goal_async(goal)
            
            self.get_logger().info(f'✓ Synced controller to: {[f"{p:.3f}" for p in self.current_positions]}')
    
    def control_mode_callback(self, msg):
        """
        Track Teensy control mode changes (PWM vs PID)
        This is separate from servo mode
        """
        self.teensy_mode = msg.data
        
        if msg.data == 'PID_CONTROL' and not self.servo_mode:
            self.get_logger().info('🤖 Teensy: PID mode (trajectory control)')
            
            # Sync controller when switching to PID from PWM
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = self.current_positions
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=0, nanosec=500000000)
            
            goal.trajectory.points = [point]
            
            self.trajectory_client.send_goal_async(goal)
            self.get_logger().info('✓ Synced controller to current position')
            
        elif msg.data == 'PWM_CONTROL':
            self.get_logger().info('⚡ Teensy: PWM mode (direct motor control)')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ModeSwitch())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
