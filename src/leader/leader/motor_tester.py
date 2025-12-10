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
        
        # MAX POWER: Limits top speed (0.25 = 25% max power)
        self.declare_parameter('max_power_limit', 0.25)
        self.power_limit = self.get_parameter('max_power_limit').get_parameter_value().double_value

        # STARTUP POWER: The minimum power needed to spin the wheels (0.0 to 1.0)
        # If your motor hums but doesn't move at low throttle, increase this (e.g., 0.05 or 0.10)
        self.declare_parameter('min_startup_power', 0.05) 
        self.min_startup = self.get_parameter('min_startup_power').get_parameter_value().double_value

        # CURVE: 1.0 = Linear, 2.0 = Quadratic (Smoother low end), 3.0 = Cubic (Very smooth low end)
        # This helps fix "too much gas at the end" perception by giving you more resolution in the early trigger pull.
        self.declare_parameter('throttle_curve', 2.0)
        self.curve_exponent = self.get_parameter('throttle_curve').get_parameter_value().double_value

        self.get_logger().info(f"GPIO: {self.pin} | Max: {self.power_limit} | Min Start: {self.min_startup} | Curve: {self.curve_exponent}")

        # --- Hardware Setup ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon! Did you run 'sudo pigpiod'?")
            exit()

        self.NEUTRAL_PW = 1500
        
        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Float32,
            'motor_throttle',
            self.listener_callback,
            10)

        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.pin, self.NEUTRAL_PW)
        time.sleep(2.0)
        self.get_logger().info("ESC Initialized.")

    def set_speed(self, pulse_width):
        pulse_width = max(1000, min(pulse_width, 2000))
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)

    def listener_callback(self, msg):
        # 1. Clamp input
        raw_input = max(-1.0, min(msg.data, 1.0))
        
        # 2. Separate direction and magnitude
        direction = 1.0 if raw_input >= 0 else -1.0
        magnitude = abs(raw_input)

        # 3. Handle Deadzone (If input is tiny, just stop)
        if magnitude < 0.01:
            self.set_speed(self.NEUTRAL_PW)
            return

        # 4. Apply Curve (Exponential) for smoother control
        # This makes the beginning of the trigger less sensitive
        curved_magnitude = math.pow(magnitude, self.curve_exponent)

        # 5. Map to Output Range [Min_Startup ... Max_Limit]
        # The logic: Start at min_startup, and add the remaining range based on the curved input
        # Formula: Output = Min + (Input * (Max - Min))
        final_power = self.min_startup + (curved_magnitude * (self.power_limit - self.min_startup))

        # 6. Convert to Pulse Width
        # 1500 + (Direction * Power * 500)
        target_pw = self.NEUTRAL_PW + (direction * final_power * 500)
        
        self.set_speed(int(target_pw))
        
        # Debug log (Limit frequency if running fast)
        # self.get_logger().info(f"In: {raw_input:.2f} -> Pwr: {final_power:.3f} -> PW: {int(target_pw)}")

    def cleanup(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.pi.stop()

# ... main function remains the same ...
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