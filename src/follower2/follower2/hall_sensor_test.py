#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import RPi.GPIO as GPIO
import time

class HallEncoderNode(Node):
    def __init__(self):
        super().__init__('hall_encoder_sensor')
        
        # --- Configuration ---
        self.hall_pin = 4       # Change this to your BCM GPIO pin
        self.magnets_on_wheel = 1 # How many magnets are on your wheel?
        
        # --- Variables ---
        self.tick_count = 0      # Total ticks counted
        self.prev_tick_count = 0 # For calculating RPM
        self.prev_time = time.time()

        # --- GPIO Setup ---
        # We match the BCM mode from your working Ultrasonic code
        GPIO.setmode(GPIO.BCM)
        # Setup pin as Input with Pull-Up Resistor (Default High, Magnet pulls Low)
        GPIO.setup(self.hall_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # --- Interrupt Setup ---
        # Instead of polling, we listen for a "Falling Edge" (High -> Low transition)
        # This function runs automatically in the background whenever the magnet passes
        GPIO.add_event_detect(self.hall_pin, GPIO.FALLING, callback=self.sensor_callback, bouncetime=10)

        # --- Publisher Setup ---
        # 1. Publish total ticks (for position/odometry)
        self.pub_ticks = self.create_publisher(Int32, 'encoder/ticks', 10)
        # 2. Publish RPM (for speed checking)
        self.pub_rpm = self.create_publisher(Float32, 'encoder/rpm', 10)
        
        # --- Timer Setup ---
        # We use the timer only to PUBLISH the data, not to read the sensor.
        # 10 Hz = 0.1 seconds period
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Hall Encoder Node Started on Pin {self.hall_pin}...")

    def sensor_callback(self, channel):
        """
        This function runs every time the magnet passes the sensor.
        It must be fast!
        """
        self.tick_count += 1

    def timer_callback(self):
        """
        Calculates RPM and publishes the current state every 0.1s
        """
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # Calculate Delta Ticks (how many happened in the last 0.1s)
        delta_ticks = self.tick_count - self.prev_tick_count
        
        # Calculate RPM: (Ticks / Magnets) / (Time in minutes)
        if dt > 0:
            rpm = (delta_ticks / self.magnets_on_wheel) / (dt / 60.0)
        else:
            rpm = 0.0

        # 1. Publish Ticks
        msg_ticks = Int32()
        msg_ticks.data = self.tick_count
        self.pub_ticks.publish(msg_ticks)

        # 2. Publish RPM
        msg_rpm = Float32()
        msg_rpm.data = rpm
        self.pub_rpm.publish(msg_rpm)
        
        # Log to console only if moving (to reduce spam)
        if delta_ticks > 0:
            self.get_logger().info(f'Ticks: {self.tick_count} | RPM: {rpm:.2f}')

        # Update previous values for next calculation
        self.prev_tick_count = self.tick_count
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = HallEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Crucial to cleanup GPIO so pins don't stay locked
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()