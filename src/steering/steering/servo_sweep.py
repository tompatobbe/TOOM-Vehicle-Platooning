#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class ServoSweep(Node):
    def __init__(self):
        super().__init__('servo_sweep_tester')
        self.publisher_ = self.create_publisher(Float32, '/servo/angle', 10)
        
        # --- CONFIGURATION ---
        # RC Car steering usually binds before 0 or 180. 
        # Watch carefully and CTRL+C if it hits the physical limit!
        self.min_angle = 0.0   # Start slightly safe
        self.max_angle = 90.0  # End slightly safe
        self.step = 2.0         # How many degrees to move per tick
        self.speed = 0.05       # Seconds to wait between moves (lower = faster)
        
        self.current_angle = self.min_angle
        self.direction = 1      # 1 for up, -1 for down
        
        # Create a timer to run the loop
        self.timer = self.create_timer(self.speed, self.timer_callback)
        self.get_logger().info('Starting Servo Sweep... Press Ctrl+C to stop.')

    def timer_callback(self):
        msg = Float32()
        msg.data = self.current_angle
        self.publisher_.publish(msg)
        
        # Calculate next position
        self.current_angle += (self.step * self.direction)

        # Reverse direction if we hit limits
        if self.current_angle >= self.max_angle:
            self.current_angle = self.max_angle
            self.direction = -1
            self.get_logger().info(f'Max Limit Reached: {self.max_angle}')
            
        elif self.current_angle <= self.min_angle:
            self.current_angle = self.min_angle
            self.direction = 1
            self.get_logger().info(f'Min Limit Reached: {self.min_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()