#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from gpiozero import Button
import time
import math  # Required for pi

class HallEncoderNode(Node):
    def __init__(self):
        super().__init__('hall_encoder_sensor')
        
        # --- Configuration ---
        self.hall_pin = 4       # BCM 4
        self.magnets_on_wheel = 2
        self.stop_timeout = 1.0 # Seconds to wait before assuming 0 speed
        
        # !!! UPDATE THIS VALUE !!!
        self.wheel_radius = 0.0318  # Radius in meters (e.g., 0.032m = 32mm)
        
        # --- State Variables ---
        self.tick_count = 0
        self.last_tick_time = None
        self.current_rpm = 0.0
        
        # --- GPIO Setup ---
        try:
            self.sensor = Button(self.hall_pin, pull_up=True, bounce_time=None)
            self.sensor.when_pressed = self.sensor_callback
        except Exception as e:
            self.get_logger().error(f"GPIO Error: {e}")

        # --- Publishers ---
        self.pub_ticks = self.create_publisher(Int32, 'follower1/encoder_ticks', 10)
        self.pub_rpm = self.create_publisher(Float32, 'follower1/encoder_rpm', 10)
        # New publisher for m/s
        self.pub_speed = self.create_publisher(Float32, 'follower1/encoder_speed_mps', 10)
        
        # --- Timer ---
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Encoder Node Started. Wheel Radius: {self.wheel_radius}m")

    def sensor_callback(self):
        """
        Triggered when magnet passes. Calculates raw RPM.
        """
        now = time.time()
        self.tick_count += 1
        
        if self.last_tick_time is not None:
            dt = now - self.last_tick_time
            
            # debouncing: ignore impossibly fast signals (< 1ms)
            if dt > 0.001: 
                # RPM = (Revolutions / Seconds) * 60
                # Revolutions = 1 / magnets_on_wheel
                self.current_rpm = (1.0 / self.magnets_on_wheel) / (dt / 60.0)
        
        self.last_tick_time = now

    def timer_callback(self):
        """
        Calculates deceleration, converts to m/s, and publishes.
        """
        now = time.time()
        
        # --- Logic: Handle Stopping / Decelerating ---
        if self.last_tick_time is not None:
            time_since_last_tick = now - self.last_tick_time
            
            # 1. Check for complete stop (Timeout)
            if time_since_last_tick > self.stop_timeout:
                self.current_rpm = 0.0
                
            # 2. Check for deceleration
            else:
                theoretical_max_rpm = (1.0 / self.magnets_on_wheel) / (time_since_last_tick / 60.0)
                if self.current_rpm > theoretical_max_rpm:
                    self.current_rpm = theoretical_max_rpm

        # --- Conversion: RPM to m/s ---
        # Formula: (RPM * Circumference) / 60
        circumference = 2 * math.pi * self.wheel_radius
        current_speed_mps = (self.current_rpm * circumference) / 60.0

        # --- Publish ---
        # Ticks
        msg_ticks = Int32()
        msg_ticks.data = self.tick_count
        self.pub_ticks.publish(msg_ticks)

        # RPM
        msg_rpm = Float32()
        msg_rpm.data = float(self.current_rpm)
        self.pub_rpm.publish(msg_rpm)
        
        # Speed (m/s)
        msg_speed = Float32()
        msg_speed.data = float(current_speed_mps)
        self.pub_speed.publish(msg_speed)
        
        # Log info
        if self.current_rpm > 0.1:
            self.get_logger().info(f'RPM: {self.current_rpm:.2f} | Speed: {current_speed_mps:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = HallEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()