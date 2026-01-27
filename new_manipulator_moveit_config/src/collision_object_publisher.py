#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class CollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__('collision_object_publisher')
        self.publisher = self.create_publisher(
            CollisionObject, 
            '/collision_object', 
            10
        )
        # Alternative: publish complete planning scene
        self.scene_publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
    def add_box(self, name, pose, size):
        """Add a box collision object"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"  # or "base_link"
        collision_object.id = name
        
        # Define box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size  # [x, y, z]
        
        # Set pose
        box_pose = Pose()
        box_pose.position.x = pose[0]
        box_pose.position.y = pose[1]
        box_pose.position.z = pose[2]
        box_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Wait for subscribers
        time.sleep(0.5)
        self.publisher.publish(collision_object)
        self.get_logger().info(f'Added collision object: {name}')

def main():
    rclpy.init()
    node = CollisionObjectPublisher()
    
    # Add a table
    node.add_box('rover_main', [0.0, -0.7, -0.2], [0.6, 1.2, 0.4])
    node.add_box('base', [0.0, 0.0, -0.28], [0.6, 0.5, 0.2])
    node.add_box('motor1', [0.30, 0.25, -0.08], [0.07, 0.07, 0.17])
    node.add_box('motor2', [-0.30, 0.25, -0.08], [0.07, 0.07, 0.17])
    
    # Add a wall
  #  node.add_box('wall', [0.0, 0.6, 0.5], [2.0, 0.1, 2.0])
    
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
