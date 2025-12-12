import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

# Configuration
HALL_PIN = 4  # Change this to your specific BCM GPIO pin

class HallSensorTester(Node):
    def __init__(self):
        super().__init__('hall_sensor_tester')
        
        # Publisher to visualize data in ROS
        self.publisher_ = self.create_publisher(Bool, 'hall_state', 10)
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        # We use PUD_UP so the pin floats High by default, and goes Low when magnet is near
        GPIO.setup(HALL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup Interrupt
        # We look for a FALLING edge (transition from High to Low) which indicates magnet detection
        GPIO.add_event_detect(HALL_PIN, GPIO.FALLING, callback=self.sensor_callback, bouncetime=20)
        
        self.get_logger().info(f"Hall Sensor Test Node Started on GPIO {HALL_PIN}. Waiting for magnets...")

    def sensor_callback(self, channel):
        """Called automatically when the sensor is triggered."""
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info('Magnet Detected! (Signal Low)')

    def cleanup(self):
        """Clean up GPIOs on shutdown."""
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = HallSensorTester()

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