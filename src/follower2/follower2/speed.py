#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from gpiozero import Button  # We use Button because it handles Pull-Up/Debounce logic perfectly
import time
import signal

class HallEncoderNode(Node):
    def __init__(self):
        super().__init__('hall_encoder_sensor')
        
        # --- Configuration ---
        self.hall_pin = 4       # BCM 17
        self.magnets_on_wheel = 6
        
        # --- Variables ---
        self.tick_count = 0
        self.prev_tick_count = 0
        self.prev_time = time.time()

        # --- GPIO Setup (The gpiozero way) ---
        # We treat the sensor like a Button:
        # pull_up=True:  Pin stays HIGH (3.3V) normally.
        # bounce_time=None: Set to 0.01 or 0.02 if you get "ghost" clicks, 
        #                   but None is best for encoders to catch fast spins.
        try:
            self.sensor = Button(self.hall_pin, pull_up=True, bounce_time=None)
            
            # "when_pressed" triggers when the pin goes LOW (Magnet Detected)
            self.sensor.when_pressed = self.sensor_callback
            
        except Exception as e:
            self.get_logger().error(f"GPIO Error: {e}")
            self.get_logger().error("Make sure you are not running other code using GPIO 17!")

        # --- Publisher Setup ---
        self.pub_ticks = self.create_publisher(Int32, 'follower2/encoder_ticks', 10)
        self.pub_rpm = self.create_publisher(Float32, 'follower2/encoder_rpm', 10)
        
        # --- Timer (10 Hz) ---
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Hall Encoder Node (gpiozero) Started on Pin {self.hall_pin}...")

    def sensor_callback(self):
        """
        Triggered automatically when magnet passes.
        """
        self.tick_count += 1

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        delta_ticks = self.tick_count - self.prev_tick_count
        
        # RPM Calculation
        if dt > 0:
            rpm = (delta_ticks / self.magnets_on_wheel) / (dt / 60.0)
        else:
            rpm = 0.0

        # Publish
        msg_ticks = Int32()
        msg_ticks.data = self.tick_count
        self.pub_ticks.publish(msg_ticks)

        msg_rpm = Float32()
        msg_rpm.data = rpm
        self.pub_rpm.publish(msg_rpm)
        
        if delta_ticks > 0:
            self.get_logger().info(f'Ticks: {self.tick_count} | RPM: {rpm:.2f}')

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
        # gpiozero cleans up automatically, but we destroy the node nicely
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()