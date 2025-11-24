import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# --- Configuration ---
# Pin definitions (BCM numbering)
IN1 = 17 # Left Motor Direction
IN2 = 27 # Left Motor Direction
ENA = 22 # Left Motor PWM
IN3 = 23 # Right Motor Direction
IN4 = 24 # Right Motor Direction
ENB = 25 # Right Motor PWM

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # 1. Hardware Setup
        self.setup_gpio()
        
        # 2. Robot Physical Properties
        self.wheel_base = 0.20  # Distance between wheels in meters (Adjust this!)
        self.max_pwm_val = 100  # Duty cycle usually 0-100
        
        # 3. Create Subscriber
        # Subscribes to 'cmd_vel' topic with a queue size of 10
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.get_logger().info('Motor Driver Node Started. Ready for cmd_vel.')

    def setup_gpio(self):
        """Initialize Raspberry Pi GPIO pins."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
        
        # Set PWM frequency to 100Hz
        self.pwm_left = GPIO.PWM(ENA, 100)
        self.pwm_right = GPIO.PWM(ENB, 100)
        
        # Start at 0 speed
        self.pwm_left.start(0)
        self.pwm_right.start(0)

    def listener_callback(self, msg):
        """
        Callback function triggered every time a message arrives on /cmd_vel.
        msg.linear.x  = Forward/Backward speed (m/s)
        msg.angular.z = Rotation speed (rad/s)
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # --- Differential Drive Math ---
        # Calculate speed for left and right wheels
        left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_base / 2.0)

        # Convert m/s to PWM duty cycle (0-100)
        # NOTE: You must tune 'scaling_factor' based on your motor's max speed
        scaling_factor = 100.0 
        left_pwm = left_speed * scaling_factor
        right_pwm = right_speed * scaling_factor

        self.set_motor_speeds(left_pwm, right_pwm)

    def set_motor_speeds(self, left_pwm, right_pwm):
        """Apply power to motors."""
        
        # --- Control Left Motor ---
        # Clamp values between -100 and 100
        left_pwm = max(min(left_pwm, 100), -100)
        
        if left_pwm > 0:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        elif left_pwm < 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            
        self.pwm_left.ChangeDutyCycle(abs(left_pwm))

        # --- Control Right Motor ---
        # Clamp values between -100 and 100
        right_pwm = max(min(right_pwm, 100), -100)
        
        if right_pwm > 0:
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
        elif right_pwm < 0:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
        else:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)

        self.pwm_right.ChangeDutyCycle(abs(right_pwm))

    def on_shutdown(self):
        """Clean up GPIO on shutdown"""
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()