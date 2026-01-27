#!/usr/bin/env python3
"""Simple keyboard mode switcher - Press S for servo, T for trajectory"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sys
import termios
import tty

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.pub = self.create_publisher(Bool, '/servo_mode', qos)
        self.sub = self.create_subscription(Bool, '/servo_mode_status', self.status_cb, 10)
        
        self.current = None
        
        print("\n╔════════════════════════════════════╗")
        print("║   MODE SWITCHER                    ║")
        print("╚════════════════════════════════════╝")
        print("\nCommands:")
        print("  [S] - Servo mode (joystick)")
        print("  [T] - Trajectory mode (MoveIt)")
        print("  [Q] - Quit\n")
    
    def status_cb(self, msg):
        mode = "🎮 SERVO" if msg.data else "🤖 TRAJECTORY"
        if mode != self.current:
            self.current = mode
            print(f"\rCurrent: {mode}  ", end="", flush=True)
    
    def switch(self, servo_mode):
        msg = Bool()
        msg.data = servo_mode
        self.pub.publish(msg)

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = ModeSwitcher()
    
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    
    try:
        while rclpy.ok():
            key = get_key().lower()
            if key == 's':
                node.switch(True)
            elif key == 't':
                node.switch(False)
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
