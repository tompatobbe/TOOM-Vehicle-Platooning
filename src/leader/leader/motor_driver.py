#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio
import time

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # --- Parameters ---
        self.declare_parameter('gpio_pin', 13)
        self.pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value
        
        self.declare_parameter('max_power_limit', 0.50) # Increased slightly for reverse torque
        self.power_limit = self.get_parameter('max_power_limit').get_parameter_value().double_value

        self.get_logger().info(f"Initializing Motor Driver on GPIO {self.pin}...")

        # --- Hardware Setup ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon!")
            exit()

        self.NEUTRAL_PW = 1500

        # --- State Variables ---
        # We need to store these because callbacks happen independently
        self.current_fwd = 0.0
        self.current_rev = 0.0

        # --- Subscribers ---
        
        # 1. Forward Topic
        self.sub_fwd = self.create_subscription(
            Float32,
            'leader/motor_throttle',
            self.fwd_callback,
            10)
            
        # 2. Reverse Topic
        self.sub_rev = self.create_subscription(
            Float32,
            'leader/motor_reverse',
            self.rev_callback,
            10)

        # Initial Hardware setup
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.pin, self.NEUTRAL_PW)
        time.sleep(2.0)
        self.get_logger().info("ESC Initialized. Ready.")

    def fwd_callback(self, msg):
        """Updates the forward value and triggers motor update"""
        self.current_fwd = msg.data
        self.update_motor_output()

    def rev_callback(self, msg):
        """Updates the reverse value and triggers motor update"""
        self.current_rev = msg.data
        self.update_motor_output()

    def update_motor_output(self):
        """Combines Forward and Reverse inputs into one ESC command"""
        
        # Logic: Net Speed = Forward Input - Reverse Input
        # If R2 is pressed (1.0) and L2 is empty (0.0) -> Result 1.0
        # If L2 is pressed (1.0) and R2 is empty (0.0) -> Result -1.0
        net_input = self.current_fwd - self.current_rev

        # Clamp to -1.0 to 1.0 just in case
        net_input = max(-1.0, min(net_input, 1.0))

        # Apply Power Limit
        # Example: If limit is 0.5:
        # Input 1.0 -> 0.5
        # Input -1.0 -> -0.5
        scaled_throttle = net_input * self.power_limit
        
        # Convert to Pulse Width
        # 0.0  -> 1500
        # 0.5  -> 1500 + 250 = 1750
        # -0.5 -> 1500 - 250 = 1250
        target_pw = self.NEUTRAL_PW + (scaled_throttle * 500)
        
        self.set_speed(int(target_pw))
        
        # Debug logging
        # self.get_logger().info(f"Fwd:{self.current_fwd:.2f} Rev:{self.current_rev:.2f} -> PW:{int(target_pw)}")

    def set_speed(self, pulse_width):
        pulse_width = max(1000, min(pulse_width, 2000))
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)

    def cleanup(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MotorTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()