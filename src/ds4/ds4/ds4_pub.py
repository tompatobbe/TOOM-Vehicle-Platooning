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

        # Publisher for Forward (R2)
        self.motor_publisher = self.create_publisher(
            Float32,
            '/motor_throttle',
            10)

        # --- NEW: Publisher for Reverse (L2) ---
        self.reverse_publisher = self.create_publisher(
            Float32,
            '/motor_reverse',
            10)
        
        self.get_logger().info("Waiting for DualShock 4 (R2=Gas, L2=Reverse)...")

        # Servo control parameters
        self.min_servo_angle = 0
        self.max_servo_angle = 60
        self.current_angle = 30 

    def listener_callback(self, msg):
        # --- 1. SERVO CONTROL (Left Stick L/R) ---
        if len(msg.axes) > 0:
            axis_val = msg.axes[0] 
            if abs(axis_val) > 0.05: 
                angle_range = self.max_servo_angle - self.min_servo_angle
                self.current_angle = self.min_servo_angle + (angle_range * (axis_val + 1) / 2)
            else:
                self.current_angle = (self.min_servo_angle + self.max_servo_angle) / 2
            
            servo_msg = Float32()
            servo_msg.data = float(self.current_angle)
            self.servo_publisher.publish(servo_msg)

        # --- 2. FORWARD MOTOR CONTROL (R2 Trigger - Axis 5) ---
        throttle_fwd = 0.0
        if len(msg.axes) > 5:
            r2_val = msg.axes[5]
            # Map 1.0 (release) -> -1.0 (press) to 0.0 -> 1.0
            throttle_fwd = (1.0 - r2_val) / 2.0
            throttle_fwd = max(0.0, min(throttle_fwd, 1.0))

            motor_msg = Float32()
            motor_msg.data = throttle_fwd
            self.motor_publisher.publish(motor_msg)

        # --- 3. NEW: REVERSE MOTOR CONTROL (L2 Trigger - Axis 2) ---
        throttle_rev = 0.0
        if len(msg.axes) > 2:
            l2_val = msg.axes[2] # Axis 2 is L2
            
            # Same math as R2: Map 1.0 -> -1.0 to 0.0 -> 1.0
            throttle_rev = (1.0 - l2_val) / 2.0
            throttle_rev = max(0.0, min(throttle_rev, 1.0))

            rev_msg = Float32()
            rev_msg.data = throttle_rev
            self.reverse_publisher.publish(rev_msg)

            # Log status
            # self.get_logger().info(f"Fwd: {throttle_fwd:.2f} | Rev: {throttle_rev:.2f}")

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