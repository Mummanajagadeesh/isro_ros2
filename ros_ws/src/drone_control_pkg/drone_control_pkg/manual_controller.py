#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ManualController(Node):
    def __init__(self):
        super().__init__('manual_controller')
        self.publisher = self.create_publisher(String, 'manual_drone_cmds', 10)
        
    def run_interactive(self):
        print("Manual Drone Controller")
        print("Type your command and press Enter:")
        
        while rclpy.ok():
            try:
                command = input(">> ").strip()
                
                if command.lower() == 'quit':
                    break
                
                if command:
                    msg = String()
                    msg.data = command
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published: {command}')
                    
            except KeyboardInterrupt:
                break
                
        print("Goodbye!")

def main():
    rclpy.init()
    controller = ManualController()
    
    try:
        controller.run_interactive()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

