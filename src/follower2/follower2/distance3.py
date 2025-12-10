#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class MultiUltrasonicNode(Node):
    def __init__(self):
        super().__init__('multi_hcsr04_sensor')
        
        # --- Configuration ---
        # 1. Center Sensor (Existing)
        self.gpio_trig_c = 5
        self.gpio_echo_c = 6
        
        # 2. Left Sensor (NEW - UPDATE THESE PINS)
        self.gpio_trig_l = 17 
        self.gpio_echo_l = 27
        
        # 3. Right Sensor (NEW - UPDATE THESE PINS)
        self.gpio_trig_r = 22
        self.gpio_echo_r = 23

        self.min_range = 0.02
        self.max_range = 4.0
        self.fov = 0.26  # Approx 15 degrees

        # --- GPIO Setup ---
        GPIO.setmode(GPIO.BCM)
        
        # Setup Center
        GPIO.setup(self.gpio_trig_c, GPIO.OUT)
        GPIO.setup(self.gpio_echo_c, GPIO.IN)
        
        # Setup Left
        GPIO.setup(self.gpio_trig_l, GPIO.OUT)
        GPIO.setup(self.gpio_echo_l, GPIO.IN)
        
        # Setup Right
        GPIO.setup(self.gpio_trig_r, GPIO.OUT)
        GPIO.setup(self.gpio_echo_r, GPIO.IN)

        # --- Publisher Setup ---
        # Publishes the SMALLEST distance found for PID control
        self.publisher_ = self.create_publisher(Range, 'follower2/sonar_dist', 10)
        self.pub_controller = self.create_publisher(Float32, 'follower2/state', 10)
        
        # --- Timer Setup ---
        # 10 Hz = 0.1 seconds period
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("Multi-Ultrasonic Node Started. Monitoring 3 sensors...")

    def measure_single_sensor(self, trig_pin, echo_pin):
        """
        Generic function to read a single HC-SR04 sensor.
        Returns distance in meters, or -1.0 on failure.
        """
        # Ensure trigger is low
        GPIO.output(trig_pin, False)
        # Tiny sleep to ensure signal is clean
        time.sleep(0.000002)

        # Send 10us pulse
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        start_time = time.time()
        stop_time = time.time()

        # Wait for Echo to go HIGH
        # Added a tighter timeout loop to prevent locking up the thread
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 0:
            start_time = time.time()
            if start_time - timeout_start > 0.04: # Timeout if no echo start (40ms)
                return -1.0

        # Wait for Echo to go LOW
        while GPIO.input(echo_pin) == 1:
            stop_time = time.time()
            if stop_time - start_time > 0.04: # Timeout if echo too long (> 6m)
                return -1.0

        # Calculate distance
        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 343) / 2
        
        return distance

    def timer_callback(self):
        # Measure all 3 sensors sequentially
        # Note: We measure sequentially to prevent cross-talk (sensors hearing each other)
        
        dist_l = self.measure_single_sensor(self.gpio_trig_l, self.gpio_echo_l)
        time.sleep(0.005) # Small buffer between sensors
        
        dist_c = self.measure_single_sensor(self.gpio_trig_c, self.gpio_echo_c)
        time.sleep(0.005)
        
        dist_r = self.measure_single_sensor(self.gpio_trig_r, self.gpio_echo_r)

        # 1. Log all distances on one line
        # Helper string for logging (shows -1 as "Err")
        fmt = lambda d: f"{d:.2f}m" if d > 0 else "Err"
        self.get_logger().info(f"L: {fmt(dist_l)} | C: {fmt(dist_c)} | R: {fmt(dist_r)}")

        # 2. Logic to find the valid minimum distance
        valid_measurements = []
        if dist_l > 0: valid_measurements.append(dist_l)
        if dist_c > 0: valid_measurements.append(dist_c)
        if dist_r > 0: valid_measurements.append(dist_r)

        if len(valid_measurements) > 0:
            min_dist = min(valid_measurements)

            # Publish Range Message (Visualizer)
            msg_range = Range()
            msg_range.header.stamp = self.get_clock().now().to_msg()
            msg_range.header.frame_id = "ultrasonic_link"
            msg_range.radiation_type = Range.ULTRASOUND
            msg_range.field_of_view = self.fov
            msg_range.min_range = self.min_range
            msg_range.max_range = self.max_range
            msg_range.range = min_dist
            self.publisher_.publish(msg_range)

            # Publish Float32 for PID Controller
            msg_pid = Float32()
            msg_pid.data = float(min_dist)
            self.pub_controller.publish(msg_pid)
        else:
            self.get_logger().warn('All sensors out of range or timeout')

def main(args=None):
    rclpy.init(args=args)
    node = MultiUltrasonicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()