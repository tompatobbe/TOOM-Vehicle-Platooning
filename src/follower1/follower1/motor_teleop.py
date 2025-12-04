#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import select
import termios
import tty

# Defines key mappings
settings = termios.tcgetattr(sys.stdin)

msg = """
Control Your Motor!
---------------------------
Moving around:
   Up Arrow    : Increase Speed (+0.1)
   Down Arrow  : Decrease Speed / Reverse (-0.1)
   Space / 'k' : FORCE STOP (0.0)

CTRL-C to quit
"""

class MotorTeleop(Node):
    def __init__(self):
        super().__init__('motor_teleop_node')
        
        # Publishers
        self.pub_throttle = self.create_publisher(Float32, 'follower1/motor_throttle', 10)
        self.pub_reverse = self.create_publisher(Float32, 'follower1/motor_reverse', 10)
        
        self.speed = 0.0
        self.step = 0.1
        self.max_speed = 1.0

    def publish_speed(self):
        throttle_msg = Float32()
        reverse_msg = Float32()

        # Logic: 
        # Positive speed -> Throttle
        # Negative speed -> Reverse
        if self.speed > 0.0:
            throttle_msg.data = min(self.speed, self.max_speed)
            reverse_msg.data = 0.0
        elif self.speed < 0.0:
            throttle_msg.data = 0.0
            reverse_msg.data = min(abs(self.speed), self.max_speed)
        else:
            throttle_msg.data = 0.0
            reverse_msg.data = 0.0

        self.pub_throttle.publish(throttle_msg)
        self.pub_reverse.publish(reverse_msg)

        # Print current status cleanly
        sys.stdout.write(f"\rCurrent Speed: {self.speed:.1f}   (Fwd: {throttle_msg.data:.1f} | Rev: {reverse_msg.data:.1f})    ")
        sys.stdout.flush()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            # Handle arrow keys which are escape sequences (e.g., \x1b[A)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main():
    rclpy.init()
    node = MotorTeleop()
    
    print(msg)

    try:
        while True:
            key = node.get_key()
            
            if key == '\x03': # Ctrl-C
                break
                
            elif key == '\x1b[A': # Up Arrow
                node.speed += node.step
                if node.speed > node.max_speed:
                    node.speed = node.max_speed
                node.publish_speed()
                
            elif key == '\x1b[B': # Down Arrow
                node.speed -= node.step
                if node.speed < -node.max_speed:
                    node.speed = -node.max_speed
                node.publish_speed()
                
            elif key == ' ' or key == 'k': # Stop
                node.speed = 0.0
                node.publish_speed()

    except Exception as e:
        print(e)

    finally:
        # Send stop command before exiting
        node.speed = 0.0
        node.publish_speed()
        print("\nStopping motor and exiting...")
        
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()