#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import AngularServo
# CHANGED: Import the correct factory for better performance
from gpiozero.pins.pigpio import PiGPIOFactory 
import threading
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('s5252_servo_controller')

        # --- Configuration ---
        self.declare_parameter('gpio_pin', 12)
        self.declare_parameter('min_pulse', 0.0005)
        self.declare_parameter('max_pulse', 0.0025)
        self.declare_parameter('middle_offset', 20.5)
        self.declare_parameter('max_speed', 45.0)  # degrees per second
        
        self.pin = self.get_parameter('gpio_pin').value
        min_p = self.get_parameter('min_pulse').value
        max_p = self.get_parameter('max_pulse').value
        self.middle_offset = self.get_parameter('middle_offset').value
        self.max_speed = self.get_parameter('max_speed').value
        
        # --- Servo Range ---
        self.max_angle = 60
        self.min_angle = 0
        
        # --- Control variables ---
        self.current_angle = 30.0  # Start at middle
        self.target_duty_cycle = 0.0
        self.running = True
        self.lock = threading.Lock()

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
            self.servo.angle = middle_angle
            self.current_angle = middle_angle
            self.get_logger().info(f'Servo set to middle position: {middle_angle:.2f}°')
        except Exception as e:
            self.get_logger().error(f'Failed to init hardware: {e}')
            # Fallback message helpful for debugging
            self.get_logger().error('Did you run "sudo pigpiod"?')
            self.servo = None

        # --- ROS Communication ---
        self.subscription = self.create_subscription(
            Float32,
            '/servo/duty_cycle',
            self.duty_cycle_callback,
            10)
        
        # Publisher for feedback
        self.feedback_pub = self.create_publisher(
            Float32,
            '/servo/feedback_angle',
            10)
        
        # Start servo control thread
        self.servo_thread = threading.Thread(target=self.servo_control_loop, daemon=True)
        self.servo_thread.start()
        
    def duty_cycle_callback(self, msg):
        """Receive duty cycle command (-100 to 100)"""
        with self.lock:
            self.target_duty_cycle = max(-100.0, min(100.0, msg.data))
    
    def servo_control_loop(self):
        """Control servo speed based on duty cycle"""
        update_rate = 0.01  # 100Hz
        
        while self.running:
            with self.lock:
                duty_cycle = self.target_duty_cycle
            
            if abs(duty_cycle) > 1.0:  # Deadzone
                # Calculate speed in degrees/second
                speed = (duty_cycle / 100.0) * self.max_speed
                
                # Update angle
                angle_change = speed * update_rate
                new_angle = self.current_angle + angle_change
                
                # Clamp to servo range
                new_angle = max(self.min_angle, min(self.max_angle, new_angle))
                
                # Apply middle offset and set servo
                servo_angle = new_angle + self.middle_offset
                servo_angle = max(self.min_angle, min(self.max_angle, servo_angle))
                
                if self.servo:
                    self.servo.angle = servo_angle
                
                self.current_angle = new_angle
                
                # Publish feedback
                feedback_msg = Float32()
                feedback_msg.data = new_angle
                self.feedback_pub.publish(feedback_msg)
            
            time.sleep(update_rate)

    def cleanup(self):
        self.running = False
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