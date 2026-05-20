#!/usr/bin/env python3
"""
Author: Move to XYZ RPY position using MoveIt 2 MoveGroup Action
Description: Sends goal pose to move_group action server with user input.
             Also supports gripper control via joint-space goal on 'hand' group.
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
    JointConstraint,
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
        Convert Euler angles (RPY) to quaternion.
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
        Create a PoseStamped message from XYZ and RPY.
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        quat = self.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    # ------------------------------------------------------------------
    # Gripper control — joint-space goal on the 'hand' planning group
    # ------------------------------------------------------------------
    def move_gripper(self, joint_value,
                     joint_name='joint_6',
                     planning_group='hand',
                     planner_id='RRTConnect',
                     planning_time=5.0,
                     max_velocity_scaling=0.5,
                     max_acceleration_scaling=0.5,
                     tolerance=0.01):
        """
        Move the gripper by commanding joint_6 to the requested value.

        Args:
            joint_value           : target position for joint_6 (radians).
                                    joint_6 limits: lower=0, upper=10
            joint_name            : name of the gripper joint (default 'joint_6')
            planning_group        : MoveIt group that owns the joint (default 'hand')
            planner_id            : planning algorithm
            planning_time         : allowed planning time in seconds
            max_velocity_scaling  : 0.0–1.0
            max_acceleration_scaling: 0.0–1.0
            tolerance             : joint position tolerance in radians
        """

        # Clamp to joint limits
        lower_limit = 0.0
        upper_limit = 10.0
        clamped = max(lower_limit, min(upper_limit, joint_value))
        if clamped != joint_value:
            self.get_logger().warn(
                f'Requested value {joint_value:.4f} out of limits '
                f'[{lower_limit}, {upper_limit}]. Clamped to {clamped:.4f}.'
            )
            joint_value = clamped

        self.get_logger().info(
            f'Planning gripper move: {joint_name} → {joint_value:.4f} rad '
            f'({math.degrees(joint_value):.2f}°)'
        )

        # Build the MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()

        goal_msg.request.group_name = planning_group
        goal_msg.request.planner_id = planner_id
        goal_msg.request.allowed_planning_time = planning_time
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = max_acceleration_scaling

        # Joint constraint for joint_6
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = joint_value
        jc.tolerance_above = tolerance
        jc.tolerance_below = tolerance
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(goal_constraints)

        # Planning options — plan and execute
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        goal_msg.planning_options.replan_delay = 2.0

        self.get_logger().info('Sending gripper goal to move_group...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected by move_group')
            return False

        self.get_logger().info('Gripper goal accepted, executing...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Gripper motion completed successfully!')
            return True
        else:
            self.get_logger().error(
                f'Gripper motion failed with error code: {result.result.error_code.val}'
            )
            return False

    # ------------------------------------------------------------------
    # Arm control — Cartesian pose goal
    # ------------------------------------------------------------------
    def move_to_pose(self, x, y, z, roll, pitch, yaw,
                     planning_group='arm_group',
                     end_effector_link='fake_link',
                     frame_id='world',
                     planner_id='RRTstarkConfigDefault',
                     planning_time=5.0,
                     max_velocity_scaling=0.1,
                     max_acceleration_scaling=0.1):
        """
        Move to the specified XYZ RPY pose.
        """

        target_pose = self.create_pose_goal(x, y, z, roll, pitch, yaw, frame_id)

        self.get_logger().info('Planning to move to:')
        self.get_logger().info(f'  Position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        self.get_logger().info(f'  Orientation (RPY): r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f}')
        self.get_logger().info(
            f'  Orientation (deg): r={math.degrees(roll):.1f}°, '
            f'p={math.degrees(pitch):.1f}°, y={math.degrees(yaw):.1f}°'
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.workspace_parameters.header.frame_id = frame_id
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()

        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        goal_msg.request.group_name = planning_group
        goal_msg.request.planner_id = planner_id
        goal_msg.request.allowed_planning_time = planning_time
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = max_acceleration_scaling

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = end_effector_link

        constraint_region = BoundingVolume()
        constraint_box = SolidPrimitive()
        constraint_box.type = SolidPrimitive.BOX
        constraint_box.dimensions = [0.001, 0.001, 0.001]
        constraint_region.primitives.append(constraint_box)
        constraint_region.primitive_poses.append(target_pose.pose)

        position_constraint.constraint_region = constraint_region
        position_constraint.weight = 1.0

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        goal_msg.planning_options.replan_delay = 2.0

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

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion completed successfully!')
            return True
        else:
            self.get_logger().error(
                f'Motion failed with error code: {result.result.error_code.val}'
            )
            return False

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current state: {feedback.state}')


# --------------------------------------------------------------------------
# Helper utilities
# --------------------------------------------------------------------------

def get_float_input(prompt, default=None):
    """Get a float value from the user with an optional default."""
    while True:
        try:
            if default is not None:
                user_input = input(f'{prompt} (default: {default}): ').strip()
                if user_input == '':
                    return default
            else:
                user_input = input(f'{prompt}: ').strip()
            return float(user_input)
        except ValueError:
            print('Invalid input! Please enter a valid number.')
        except KeyboardInterrupt:
            print('\nExiting...')
            sys.exit(0)


def get_choice_input(prompt, choices):
    """Get a validated choice from the user. choices is a list of str."""
    while True:
        try:
            user_input = input(prompt).strip()
            if user_input in choices:
                return user_input
            print(f'Please enter one of: {", ".join(choices)}')
        except KeyboardInterrupt:
            print('\nExiting...')
            sys.exit(0)


# --------------------------------------------------------------------------
# Gripper sub-flow
# --------------------------------------------------------------------------

def run_gripper_flow(node):
    """Interactively ask for gripper target and execute."""
    print('\n--- Gripper Control ---')
    print('joint_6 limits: 0.0 (closed) to 10.0 (open) radians')

    print('\nChoose input format:')
    print('  1. Radians')
    print('  2. Degrees')
    fmt = get_choice_input('Enter choice (1 or 2, default: 1): ', ['1', '2', ''])
    use_degrees = (fmt == '2')

    if use_degrees:
        raw = get_float_input('Target joint_6 value (degrees)', 0.0)
        joint_val = math.radians(raw)
    else:
        joint_val = get_float_input('Target joint_6 value (radians)', 0.0)

    print('\n--- Optional Parameters (press Enter for defaults) ---')
    planning_time = get_float_input('Planning time in seconds', 5.0)
    max_vel = get_float_input('Max velocity scaling (0.0–1.0)', 0.5)
    max_acc = get_float_input('Max acceleration scaling (0.0–1.0)', 0.5)

    print('\n' + '=' * 60)
    print('Executing gripper command...')
    print('=' * 60 + '\n')

    success = node.move_gripper(
        joint_value=joint_val,
        joint_name='joint_6',
        planning_group='hand',
        planning_time=planning_time,
        max_velocity_scaling=max_vel,
        max_acceleration_scaling=max_acc,
    )

    print('\n' + '=' * 60)
    if success:
        print('✓ Gripper successfully moved!')
    else:
        print('✗ Gripper motion failed')
    print('=' * 60 + '\n')


# --------------------------------------------------------------------------
# Arm sub-flow
# --------------------------------------------------------------------------

def run_arm_flow(node):
    """Interactively ask for target pose and execute."""
    print('\n--- Position (in meters) ---')
    x = get_float_input('X', 0.3)
    y = get_float_input('Y', 0.2)
    z = get_float_input('Z', 0.4)

    print('\n--- Orientation ---')
    print('Choose input format:')
    print('  1. Radians')
    print('  2. Degrees')

    fmt = get_choice_input('Enter choice (1 or 2, default: 2): ', ['1', '2', ''])
    use_degrees = (fmt != '1')

    if use_degrees:
        print('\nEnter orientation in degrees:')
    else:
        print('\nEnter orientation in radians:')

    roll_input  = get_float_input('Roll',  0.0)
    pitch_input = get_float_input('Pitch', 0.0)
    yaw_input   = get_float_input('Yaw',   0.0)

    if use_degrees:
        roll  = math.radians(roll_input)
        pitch = math.radians(pitch_input)
        yaw   = math.radians(yaw_input)
    else:
        roll, pitch, yaw = roll_input, pitch_input, yaw_input

    print('\n--- Optional Parameters (press Enter for defaults) ---')
    planning_group      = input('Planning group name (default: arm_group): ').strip() or 'arm_group'
    end_effector_link   = input('End effector link (default: fake_link): ').strip() or 'fake_link'
    frame_id            = input('Reference frame (default: world): ').strip() or 'world'
    planner_id          = input('Planner ID (default: RRTstarkConfigDefault): ').strip() or 'RRTstarkConfigDefault'

    planning_time           = get_float_input('Planning time in seconds', 5.0)
    max_velocity_scaling    = get_float_input('Max velocity scaling (0.0–1.0)', 0.1)
    max_acceleration_scaling = get_float_input('Max acceleration scaling (0.0–1.0)', 0.1)

    print('\n' + '=' * 60)
    print('Starting motion planning...')
    print('=' * 60 + '\n')

    success = node.move_to_pose(
        x=x, y=y, z=z,
        roll=roll, pitch=pitch, yaw=yaw,
        planning_group=planning_group,
        end_effector_link=end_effector_link,
        frame_id=frame_id,
        planner_id=planner_id,
        planning_time=planning_time,
        max_velocity_scaling=max_velocity_scaling,
        max_acceleration_scaling=max_acceleration_scaling,
    )

    print('\n' + '=' * 60)
    if success:
        print('✓ Successfully moved to target pose!')
    else:
        print('✗ Failed to move to target pose')
    print('=' * 60 + '\n')


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = MoveToXYZRPY()

    print('=' * 60)
    print('  MoveIt 2 — Arm & Gripper Controller')
    print('=' * 60)
    print('\nWhat would you like to do?')
    print('  1. Move arm to XYZ RPY pose')
    print('  2. Move gripper (open / close / any position)')

    choice = get_choice_input('\nEnter choice (1 or 2): ', ['1', '2'])

    if choice == '1':
        run_arm_flow(node)
    else:
        run_gripper_flow(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()