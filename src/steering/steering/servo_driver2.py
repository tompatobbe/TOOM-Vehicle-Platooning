#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Servo # Note: We use standard Servo, not AngularServo for custom mapping
from gpiozero.pins.pigpio import PiGPIOFactory

class ServoController(Node):
    def __init__(self):
        super().__init__('s5252_servo_controller')

        # --- CALIBRATION CONFIGURATION ---
        # REPLACE THESE 3 NUMBERS WITH THE ONES YOU WROTE DOWN
        self.calib_left_pulse   = 0.001100  # The pulse when wheels are full left
        self.calib_center_pulse = 0.001450  # The pulse when wheels are straight
        self.calib_right_pulse  = 0.001900  # The pulse when wheels are full right
        
        self.gpio_pin = 12

        # --- Hardware Setup ---
        factory = PiGPIOFactory()
        
        # We initialize a generic Servo with the absolute min/max safety limits
        self.servo = Servo(
            self.gpio_pin, 
            min_pulse_width=0.0005, 
            max_pulse_width=0.0025, 
            pin_factory=factory
        )
        
        self.subscription = self.create_subscription(
            Float32, '/servo/angle', self.listener_callback, 10)
        
        self.get_logger().info('Calibrated Servo Driver Started')

    def map_angle_to_pulse(self, angle):
        """
        Maps 0-180 degrees to the specific calibrated pulse widths.
        0-90   maps to  Left_Pulse - Center_Pulse
        90-180 maps to  Center_Pulse - Right_Pulse
        """
        # Clamp input
        angle = max(0.0, min(180.0, angle))

        if angle < 90.0:
            # Map 0-90 to Left-Center
            ratio = angle / 90.0
            # Linear interpolation between Left and Center
            pulse = self.calib_left_pulse + (self.calib_center_pulse - self.calib_left_pulse) * ratio
        else:
            # Map 90-180 to Center-Right
            ratio = (angle - 90.0) / 90.0
            pulse = self.calib_center_pulse + (self.calib_right_pulse - self.calib_center_pulse) * ratio
            
        return pulse

    def listener_callback(self, msg):
        target_pulse = self.map_angle_to_pulse(msg.data)
        
        # Convert pulse to gpiozero value (-1 to 1 range based on 1.5ms center)
        # gpiozero expects: -1 = min_pulse, 1 = max_pulse. 
        # Since we are doing custom mapping, we set the value directly via pulse width
        
        # Gpiozero 'value' property is tricky with custom ranges, 
        # so we use the internal pulse width setter if available, 
        # or recalculate value manually:
        
        # Math to turn pulse (e.g. 0.0015) into gpiozero value (-1 to 1)
        # value = (pulse - mid_point) / (half_range)
        mid_point = (0.0005 + 0.0025) / 2
        half_range = (0.0025 - 0.0005) / 2
        servo_value = (target_pulse - mid_point) / half_range
        
        self.servo.value = servo_value
        # self.get_logger().info(f'Angle: {msg.data} -> Pulse: {target_pulse:.6f}')

    def cleanup(self):
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