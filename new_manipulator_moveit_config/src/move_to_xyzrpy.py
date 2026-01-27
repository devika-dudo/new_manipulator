#!/usr/bin/env python3
"""
Author: Move to XYZ RPY position using MoveIt 2 MoveGroup Action
Description: Sends goal pose to move_group action server with user input
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math
import sys


class MoveToXYZRPY(Node):
    def __init__(self):
        super().__init__('move_to_xyz_rpy')
        
        # Action client for move_group
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        self.get_logger().info('Waiting for move_group action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to move_group action server')
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles (RPY) to quaternion
        Args:
            roll, pitch, yaw: angles in radians
        Returns:
            (x, y, z, w) quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (x, y, z, w)
    
    def create_pose_goal(self, x, y, z, roll, pitch, yaw, frame_id='world'):
        """
        Create a PoseStamped message from XYZ and RPY
        Args:
            x, y, z: position in meters
            roll, pitch, yaw: orientation in radians
            frame_id: reference frame
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Orientation (convert RPY to quaternion)
        quat = self.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose
    
    def move_to_pose(self, x, y, z, roll, pitch, yaw, 
                     planning_group='arm_group',
                     end_effector_link='fake_link',
                     frame_id='world',
                     planner_id='RRTConnect',
                     planning_time=5.0,
                     max_velocity_scaling=0.1,
                     max_acceleration_scaling=0.1):
        """
        Move to the specified XYZ RPY pose
        
        Args:
            x, y, z: target position in meters
            roll, pitch, yaw: target orientation in radians
            planning_group: name of the planning group (e.g., 'arm', 'manipulator')
            end_effector_link: name of end effector link
            frame_id: reference frame
            planner_id: planning algorithm ('RRTConnect', 'RRT', 'PRM', etc.)
            planning_time: allowed planning time in seconds
            max_velocity_scaling: velocity scaling factor (0.0 to 1.0)
            max_acceleration_scaling: acceleration scaling factor (0.0 to 1.0)
        """
        
        # Create the goal pose
        target_pose = self.create_pose_goal(x, y, z, roll, pitch, yaw, frame_id)
        
        self.get_logger().info(f'Planning to move to:')
        self.get_logger().info(f'  Position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        self.get_logger().info(f'  Orientation (RPY): r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f}')
        self.get_logger().info(f'  Orientation (deg): r={math.degrees(roll):.1f}°, p={math.degrees(pitch):.1f}°, y={math.degrees(yaw):.1f}°')
        
        # Create MoveGroup goal
        goal_msg = MoveGroup.Goal()
        
        # Setup motion plan request
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.workspace_parameters.header.frame_id = frame_id
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        # Set workspace bounds (adjust as needed)
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        # Set planning group
        goal_msg.request.group_name = planning_group
        
        # Set planner
        goal_msg.request.planner_id = planner_id
        goal_msg.request.allowed_planning_time = planning_time
        goal_msg.request.num_planning_attempts = 10
        
        # Set velocity and acceleration scaling
        goal_msg.request.max_velocity_scaling_factor = max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = max_acceleration_scaling
        
        # Create position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = end_effector_link
        
        # Define constraint region (small box around target)
        constraint_region = BoundingVolume()
        constraint_box = SolidPrimitive()
        constraint_box.type = SolidPrimitive.BOX
        constraint_box.dimensions = [0.001, 0.001, 0.001]  # Very tight tolerance
        constraint_region.primitives.append(constraint_box)
        constraint_region.primitive_poses.append(target_pose.pose)
        
        position_constraint.constraint_region = constraint_region
        position_constraint.weight = 1.0
        
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        
        # Add constraints to goal
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        # Planning options
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False  # Plan and execute
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        goal_msg.planning_options.replan_delay = 2.0
        
        # Send goal
        self.get_logger().info('Sending goal to move_group...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by move_group')
            return False
        
        self.get_logger().info('Goal accepted, executing...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion completed successfully!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {result.result.error_code.val}')
            return False
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current state: {feedback.state}')


def get_float_input(prompt, default=None):
    """Helper function to get float input from user"""
    while True:
        try:
            if default is not None:
                user_input = input(f"{prompt} (default: {default}): ").strip()
                if user_input == "":
                    return default
            else:
                user_input = input(f"{prompt}: ").strip()
            
            return float(user_input)
        except ValueError:
            print("Invalid input! Please enter a valid number.")
        except KeyboardInterrupt:
            print("\nExiting...")
            sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveToXYZRPY()
    
    print("\n" + "="*60)
    print("MoveIt 2 - Move to XYZ RPY Position")
    print("="*60)
    print("\nEnter target pose (press Ctrl+C to exit):\n")
    
    # Get position input
    print("--- Position (in meters) ---")
    x = get_float_input("X", 0.3)
    y = get_float_input("Y", 0.2)
    z = get_float_input("Z", 0.4)
    
    # Get orientation input
    print("\n--- Orientation ---")
    print("Choose input format:")
    print("1. Radians")
    print("2. Degrees")
    
    angle_format = input("Enter choice (1 or 2, default: 2): ").strip()
    if angle_format == "" or angle_format == "2":
        use_degrees = True
        print("\nEnter orientation in degrees:")
    else:
        use_degrees = False
        print("\nEnter orientation in radians:")
    
    roll_input = get_float_input("Roll", 0.0)
    pitch_input = get_float_input("Pitch", 0.0)
    yaw_input = get_float_input("Yaw", 90.0 if use_degrees else 1.57)
    
    # Convert to radians if needed
    if use_degrees:
        roll = math.radians(roll_input)
        pitch = math.radians(pitch_input)
        yaw = math.radians(yaw_input)
    else:
        roll = roll_input
        pitch = pitch_input
        yaw = yaw_input
    
    # Get optional parameters
    print("\n--- Optional Parameters (press Enter for defaults) ---")
    planning_group = input("Planning group name (default: arm): ").strip() or "arm"
    end_effector_link = input("End effector link (default: link6): ").strip() or "link6"
    frame_id = input("Reference frame (default: world): ").strip() or "world"
    planner_id = input("Planner ID (default: RRTConnect): ").strip() or "RRTConnect"
    
    planning_time = get_float_input("Planning time in seconds", 5.0)
    max_velocity_scaling = get_float_input("Max velocity scaling (0.0-1.0)", 0.1)
    max_acceleration_scaling = get_float_input("Max acceleration scaling (0.0-1.0)", 0.1)
    
    print("\n" + "="*60)
    print("Starting motion planning...")
    print("="*60 + "\n")
    
    success = node.move_to_pose(
        x=x, 
        y=y, 
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        planning_group=planning_group,
        end_effector_link=end_effector_link,
        frame_id=frame_id,
        planner_id=planner_id,
        planning_time=planning_time,
        max_velocity_scaling=max_velocity_scaling,
        max_acceleration_scaling=max_acceleration_scaling
    )
    
    print("\n" + "="*60)
    if success:
        print("✓ Successfully moved to target pose!")
    else:
        print("✗ Failed to move to target pose")
    print("="*60 + "\n")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
