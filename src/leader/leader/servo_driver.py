#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory 

class ServoController(Node):
    def __init__(self):
        super().__init__('s5252_servo_controller')

        # --- Configuration ---
        self.declare_parameter('gpio_pin', 12)
        self.declare_parameter('min_pulse', 0.0005)
        self.declare_parameter('max_pulse', 0.0025)
        # Adjust this value in your ROS params file to straighten wheels
        self.declare_parameter('middle_offset', -11) 
        
        self.pin = self.get_parameter('gpio_pin').value
        min_p = self.get_parameter('min_pulse').value
        max_p = self.get_parameter('max_pulse').value
        self.middle_offset = self.get_parameter('middle_offset').value
        
        
        # --- Servo Range ---
        self.max_angle = 60
        self.min_angle = 0

        # --- Hardware Setup ---
        try:
            # Ensure 'sudo pigpiod' is running before starting this node
            factory = PiGPIOFactory()
            self.servo = AngularServo(
                self.pin, 
                min_angle=self.min_angle, 
                max_angle=self.max_angle,
                min_pulse_width=min_p, 
                max_pulse_width=max_p,
                pin_factory=factory
            )
            self.get_logger().info(f'Servo initialized on GPIO {self.pin}')
        
            # Set servo to middle position on startup
            middle_angle = (self.min_angle + self.max_angle) / 2
            
            # Apply offset for initial state
            initial_pos = middle_angle + self.middle_offset
            
            # Initial Clamp to ensure start position is safe
            initial_pos = max(self.min_angle, min(self.max_angle, initial_pos))
            
            self.servo.angle = initial_pos
            self.get_logger().info(f'Servo set to start position: {initial_pos:.2f}° (Offset: {self.middle_offset})')

        except Exception as e:
            self.get_logger().error(f'Failed to init hardware: {e}')
            self.get_logger().error('Did you run "sudo pigpiod"?')
            self.servo = None

        # --- ROS Communication ---
        self.subscription = self.create_subscription(
            Float32,
            'leader/servo/angle',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        # 1. Calculate the base angle from the incoming message
        base_angle = 60 - msg.data
        
        # 2. Apply the offset to the calculated angle
        # [FIX]: This was missing in your original code
        target_angle = base_angle + self.middle_offset
        
        if self.servo:
            # 3. Safety Clamp 
            # We clamp AFTER adding the offset to ensure we don't go out of bounds
            final_angle = max(self.min_angle, min(self.max_angle, target_angle))
            
            self.servo.angle = final_angle
            
            # Enhanced logging to see the calculation
            self.get_logger().info(f'Input: {msg.data:.1f} | Base: {base_angle:.1f} | Offset: {self.middle_offset} | Final: {final_angle:.2f}°')


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