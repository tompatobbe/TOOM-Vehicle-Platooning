#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import AngularServo
from gpiozero.pins.rpigpio import RPiGPIOFactory
import math

class ServoController(Node):
    def __init__(self):
        super().__init__('s5252_servo_controller')

        # --- Configuration ---
        # Declare parameters so you can change pins without recompiling
        self.declare_parameter('gpio_pin', 12)
        self.declare_parameter('min_pulse', 0.0005) # 0.5ms (standard for many servos)
        self.declare_parameter('max_pulse', 0.0025) # 2.5ms
        
        self.pin = self.get_parameter('gpio_pin').value
        min_p = self.get_parameter('min_pulse').value
        max_p = self.get_parameter('max_pulse').value

        # --- Hardware Setup ---
        # We use PiGPIOFactory for better PWM timing (reduces servo jitter)
        # Make sure 'sudo pigpiod' is running
        try:
            factory = PiGPIOFactory()
            self.servo = AngularServo(
                self.pin, 
                min_angle=0, 
                max_angle=180,
                min_pulse_width=min_p, 
                max_pulse_width=max_p,
                pin_factory=factory
            )
            self.get_logger().info(f'Servo initialized on GPIO {self.pin}')
        except Exception as e:
            self.get_logger().error(f'Failed to init hardware: {e}')
            self.servo = None

        # --- ROS Communication ---
        self.subscription = self.create_subscription(
            Float32,
            '/servo/angle',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        """
        Expects an angle in degrees (0.0 to 180.0)
        """
        target_angle = msg.data
        
        if self.servo:
            # Safety clamping
            target_angle = max(0.0, min(180.0, target_angle))
            
            self.servo.angle = target_angle
            self.get_logger().info(f'Moving to: {target_angle:.2f} degrees')

    def cleanup(self):
        if self.servo:
            self.servo.close()

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()

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