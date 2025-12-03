import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

# Constants for WP 1040 ESC (Standard Servo Protocol)
PWM_PIN = 13          # Change to your actual GPIO pin (BCM numbering)
PWM_FREQ = 50         # 50Hz is standard for ESCs
DUTY_CYCLE_STOP = 7.5 # 1.5ms pulse / 20ms period * 100 = 7.5%
DUTY_CYCLE_MAX = 10.0 # 2.0ms pulse / 20ms period * 100 = 10.0%
DUTY_CYCLE_MIN = 5.0  # 1.0ms pulse / 20ms period * 100 = 5.0%

class WP1040Driver(Node):
    def __init__(self):
        super().__init__('wp1040_driver')
        
        # --- 1. GPIO Setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN, GPIO.OUT)
        
        # Start PWM at 50Hz
        self.pwm = GPIO.PWM(PWM_PIN, PWM_FREQ)
        
        # Initialize ESC:
        # Most ESCs need to see a "Neutral" signal to arm for safety.
        self.get_logger().info('Arming ESC...')
        self.pwm.start(DUTY_CYCLE_STOP) 
        
        # --- 2. ROS Subscriber ---
        # Subscription to 'throttle' topic (Float32, 0.0 to 1.0)
        self.subscription = self.create_subscription(
            Float32,
            'throttle',
            self.throttle_callback,
            10
        )
        
        self.get_logger().info('WP 1040 Driver Node Started. Listening on /throttle')

    def throttle_callback(self, msg):
        """
        Receives throttle value between 0.0 and 1.0
        Maps 0.0 -> Stop (Neutral)
        Maps 1.0 -> Full Speed Forward
        """
        throttle_input = msg.data
        
        # Clamp input between 0.0 and 1.0 for safety
        throttle_input = max(0.0, min(1.0, throttle_input))

        # --- 3. Calculate Duty Cycle ---
        # Linear mapping: Output = Neutral + (Input * (Max - Neutral))
        # This maps 0.0 to 7.5% (Stop) and 1.0 to 10.0% (Full Forward)
        target_duty_cycle = DUTY_CYCLE_STOP + (throttle_input * (DUTY_CYCLE_MAX - DUTY_CYCLE_STOP))
        
        # Apply to hardware
        self.pwm.ChangeDutyCycle(target_duty_cycle)
        
        # Debug logging (optional, comment out for high-speed loops)
        # self.get_logger().debug(f'Input: {throttle_input:.2f} -> Duty: {target_duty_cycle:.2f}')

    def stop_motor(self):
        """Safety stop"""
        self.pwm.ChangeDutyCycle(DUTY_CYCLE_STOP)

    def __del__(self):
        """Cleanup on shutdown"""
        self.stop_motor()
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info('GPIO Cleaned up.')

def main(args=None):
    rclpy.init(args=args)
    
    driver_node = WP1040Driver()

    try:
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()