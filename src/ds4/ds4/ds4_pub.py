#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        
        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        
        # Publisher for Servo (Steering)
        self.servo_publisher = self.create_publisher(
            Float32,
            '/servo/angle',
            10)

        # Publisher for Motor (Throttle) - LINKED TO MOTOR NODE
        self.motor_publisher = self.create_publisher(
            Float32,
            '/motor_throttle',
            10)
        
        self.get_logger().info("Waiting for DualShock 4 input (R2 for Gas)...")

        # Servo control parameters
        self.min_servo_angle = 0
        self.max_servo_angle = 60
        self.current_angle = 30  # Start at middle

    def listener_callback(self, msg):
        # --- 1. SERVO CONTROL (Left Stick L/R) ---
        if len(msg.axes) > 0:
            axis_val = msg.axes[0]  # Left Stick L/R
            if abs(axis_val) > 0.05:  # Deadzone filter
                # Map joystick range [-1, 1] to servo angle range
                angle_range = self.max_servo_angle - self.min_servo_angle
                self.current_angle = self.min_servo_angle + (angle_range * (axis_val + 1) / 2)
            else:
                # Reset to middle when no input
                self.current_angle = (self.min_servo_angle + self.max_servo_angle) / 2
            
            # Publish servo command
            servo_msg = Float32()
            servo_msg.data = float(self.current_angle)
            self.servo_publisher.publish(servo_msg)

        # --- 2. MOTOR CONTROL (R2 Trigger) ---
        # Axis 5 is usually R2 on DualShock 4
        if len(msg.axes) > 5:
            r2_val = msg.axes[5]
            
            # The User specified: R2 goes from 1.0 (Released) to -1.0 (Pressed)
            # We want: 0.0 (Stop) to 1.0 (Full Forward)
            
            # Formula: 
            # 1.0 - 1.0 = 0.0 / 2 = 0.0 (Stop)
            # 1.0 - (-1.0) = 2.0 / 2 = 1.0 (Full Gas)
            
            throttle = (1.0 - r2_val) / 2.0

            # Clamp ensure we stay 0.0 to 1.0 (handles floating point noise)
            throttle = max(0.0, min(throttle, 1.0))

            # Publish to Motor Driver
            motor_msg = Float32()
            motor_msg.data = throttle
            self.motor_publisher.publish(motor_msg)

            # Optional: Print status
            self.get_logger().info(f"R2: {r2_val:.2f} -> Throttle: {throttle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()