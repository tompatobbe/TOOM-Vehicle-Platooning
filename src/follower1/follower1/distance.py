#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32 # Added: Needed for communicating with PID controller
import RPi.GPIO as GPIO
import time

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('hcsr04_sensor')
        
        # --- Configuration ---
        self.gpio_trigger = 23
        self.gpio_echo = 24
        self.min_range = 0.02
        self.max_range = 4.0
        self.fov = 0.26  # Approx 15 degrees

        # --- GPIO Setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_trigger, GPIO.OUT)
        GPIO.setup(self.gpio_echo, GPIO.IN)

        # --- Publisher Setup ---
        # 1. Existing Range publisher (Good for visualization/debugging)
        self.publisher_ = self.create_publisher(Range, 'sonar_dist', 10)

        # 2. NEW: Publisher for the PID Controller
        # The PID controller listens to 'state' to know the current value
        self.pub_controller = self.create_publisher(Float32, 'state', 10)
        
        # --- Timer Setup ---
        # 10 Hz = 0.1 seconds period
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("Ultrasonic Sensor Node (ROS 2) Started...")

    def measure_distance(self):
        # Ensure trigger is low
        GPIO.output(self.gpio_trigger, False)
        time.sleep(0.000002)

        # Send 10us pulse
        GPIO.output(self.gpio_trigger, True)
        time.sleep(0.00001)
        GPIO.output(self.gpio_trigger, False)

        start_time = time.time()
        stop_time = time.time()

        # Wait for Echo to go HIGH
        while GPIO.input(self.gpio_echo) == 0:
            start_time = time.time()
            # Safety timeout (0.1s)
            if start_time - stop_time > 0.1:
                return -1.0

        # Wait for Echo to go LOW
        while GPIO.input(self.gpio_echo) == 1:
            stop_time = time.time()
            # Safety timeout
            if stop_time - start_time > 0.1:
                return -1.0

        # Calculate distance
        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 343) / 2
        return distance

    def timer_callback(self):
        dist = self.measure_distance()

        if dist > 0:
            # 1. Publish Range Message (Old way)
            msg_range = Range()
            msg_range.header.stamp = self.get_clock().now().to_msg()
            msg_range.header.frame_id = "ultrasonic_link"
            msg_range.radiation_type = Range.ULTRASOUND
            msg_range.field_of_view = self.fov
            msg_range.min_range = self.min_range
            msg_range.max_range = self.max_range
            msg_range.range = dist
            self.publisher_.publish(msg_range)

            # 2. Publish Float32 for Controller (New way)
            msg_pid = Float32()
            msg_pid.data = float(dist)
            self.pub_controller.publish(msg_pid)
            
            # Log to console
            self.get_logger().info(f'Distance: {dist:.2f} m')
        else:
            self.get_logger().warn('Sensor timeout or out of range')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Crucial to cleanup GPIO so pins don't stay HIGH
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()