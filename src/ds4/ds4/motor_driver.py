#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio
import time
import math

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # --- Parameters ---
        self.declare_parameter('gpio_pin', 13)
        self.pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value
        
        # Safety Limit: 0.50 means max power is limited to 50%
        # 20% might be too weak to overcome the motor's deadzone/friction.
        self.declare_parameter('max_power_limit', 0.15)
        self.power_limit = self.get_parameter('max_power_limit').get_parameter_value().double_value

        self.get_logger().info(f"Initializing Motor Driver on GPIO {self.pin} with Max Power {self.power_limit*100}%")

        # --- Hardware Setup ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon! Did you run 'sudo pigpiod'?")
            exit()

        self.NEUTRAL_PW = 1500

        # --- Subscriber ---
        # Listens for a float between -1.0 and 1.0
        self.subscription = self.create_subscription(
            Float32,
            'motor_throttle',
            self.listener_callback,
            10)

        # Initial Hardware setup
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.pin, self.NEUTRAL_PW) # Initialize at Neutral to Arm ESC
        
        # Give it a moment to arm physically
        time.sleep(2.0)
        self.get_logger().info("ESC Initialized. Ready for topics.")

    def set_speed(self, pulse_width):
        """Safely writes the pulse width to the ESC"""
        # Hard limits for standard ESCs (1000-2000)
        pulse_width = max(1000, min(pulse_width, 2000))
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)

    def listener_callback(self, msg):
        # 1. Clamp input to -1.0 to 1.0 for safety
        input_throttle = max(-1.0, min(msg.data, 1.0))
        
        # 2. Apply Power Limit (scaling)
        # If limit is 0.2, input 1.0 becomes 0.2
        scaled_throttle = input_throttle * self.power_limit
        
        # 3. Convert to Pulse Width
        # 0.0 -> 1500
        # 1.0 -> 1500 + 500 = 2000
        # -1.0 -> 1500 - 500 = 1000
        target_pw = self.NEUTRAL_PW + (scaled_throttle * 500)
        
        self.set_speed(int(target_pw))
        
        # Log the output so we can debug deadzones
        self.get_logger().info(f"Input: {input_throttle:.2f} -> PW: {int(target_pw)}")

    def cleanup(self):
        """Stop motor on shutdown"""
        self.get_logger().info("Stopping Motor...")
        self.pi.set_servo_pulsewidth(self.pin, 0) # 0 kills the signal
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