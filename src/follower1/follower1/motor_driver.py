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
        
        self.declare_parameter('max_power_limit', 0.50) 
        self.power_limit = self.get_parameter('max_power_limit').get_parameter_value().double_value

        self.get_logger().info(f"Initializing Motor Driver on GPIO {self.pin}...")

        # --- Hardware Setup ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon!")
            exit()

        self.NEUTRAL_PW = 1500

        # --- State Variables ---
        self.current_fwd = 0.0
        self.current_rev = 0.0
        self.last_log_time = time.time()

        # --- Subscribers ---
        # 1. Forward Topic
        self.sub_fwd = self.create_subscription(
            Float32,
            'motor_throttle',
            self.fwd_callback,
            10)
            
        # 2. Reverse Topic
        self.sub_rev = self.create_subscription(
            Float32,
            'motor_reverse',
            self.rev_callback,
            10)

        # --- Heartbeat Timer ---
        # Creates a timer that runs every 5 seconds to prove the node is alive
        self.timer = self.create_timer(5.0, self.timer_callback)

        # Initial Hardware setup
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.pin, self.NEUTRAL_PW)
        time.sleep(2.0)
        self.get_logger().info("ESC Initialized. Ready. Listening on 'motor_throttle' & 'motor_reverse'...")

    def timer_callback(self):
        """Prints a heartbeat if no commands are being processed"""
        if self.current_fwd == 0.0 and self.current_rev == 0.0:
            self.get_logger().info("Node is alive. Waiting for input on topics...")

    def fwd_callback(self, msg):
        self.current_fwd = msg.data
        self.update_motor_output()

    def rev_callback(self, msg):
        self.current_rev = msg.data
        self.update_motor_output()

    def update_motor_output(self):
        # Logic: Net Speed = Forward Input - Reverse Input
        net_input = self.current_fwd - self.current_rev

        # Clamp
        net_input = max(-1.0, min(net_input, 1.0))

        # Apply Power Limit
        scaled_throttle = net_input * self.power_limit
        
        # Convert to Pulse Width
        target_pw = self.NEUTRAL_PW + (scaled_throttle * 500)
        
        self.set_speed(int(target_pw))
        
        # Log every update
        self.get_logger().info(
            f"Incoming Net Speed: {net_input:.2f} | "
            f"Outgoing Speed (Scaled): {scaled_throttle:.2f} | "
            f"Pin PW: {int(target_pw)}"
        )

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