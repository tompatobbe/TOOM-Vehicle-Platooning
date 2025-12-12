#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from gpiozero import Button
import time

class HallEncoderNode(Node):
    def __init__(self):
        super().__init__('hall_encoder_sensor')
        
        # --- Configuration ---
        self.hall_pin = 4       # BCM 4 (Check your wiring!)
        self.magnets_on_wheel = 2
        self.stop_timeout = 1.0 # Seconds to wait before assuming 0 RPM
        
        # --- State Variables ---
        self.tick_count = 0
        self.last_tick_time = None  # Timestamp of the last magnet pass
        self.current_rpm = 0.0      # The latest calculated speed
        
        # --- GPIO Setup ---
        try:
            # bounce_time=None is crucial for encoders to capture fast signals
            self.sensor = Button(self.hall_pin, pull_up=True, bounce_time=None)
            self.sensor.when_pressed = self.sensor_callback
        except Exception as e:
            self.get_logger().error(f"GPIO Error: {e}")

        # --- Publishers ---
        self.pub_ticks = self.create_publisher(Int32, 'leader/encoder_ticks', 10)
        self.pub_rpm = self.create_publisher(Float32, 'leader/encoder_rpm', 10)
        
        # --- Timer ---
        # Runs at 10Hz to publish data continuously
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Continuous Encoder Node Started on Pin {self.hall_pin}")

    def sensor_callback(self):
        """
        Triggered IMMEDIATELY when the magnet passes.
        Calculates exact RPM based on time since the last pass.
        """
        now = time.time()
        self.tick_count += 1
        
        if self.last_tick_time is not None:
            # Calculate time difference (dt)
            dt = now - self.last_tick_time
            
            # debouncing: ignore impossibly fast signals (< 1ms)
            if dt > 0.001: 
                # RPM = (Revolutions / Seconds) * 60
                # Revolutions = 1 / magnets_on_wheel
                self.current_rpm = (1.0 / self.magnets_on_wheel) / (dt / 60.0)
        
        self.last_tick_time = now

    def timer_callback(self):
        """
        Publishes the current state and handles stopping logic.
        """
        now = time.time()
        
        # --- Logic: Handle Stopping / Decelerating ---
        if self.last_tick_time is not None:
            time_since_last_tick = now - self.last_tick_time
            
            # 1. Check for complete stop (Timeout)
            if time_since_last_tick > self.stop_timeout:
                self.current_rpm = 0.0
                
            # 2. Check for deceleration
            # If the time we've been waiting is LONGER than the time implied 
            # by the current speed, we are definitely slowing down.
            # We calculate the "maximum possible RPM" given the current wait time.
            else:
                theoretical_max_rpm = (1.0 / self.magnets_on_wheel) / (time_since_last_tick / 60.0)
                
                # If our stored RPM is higher than physics allows (given the wait), clamp it down.
                if self.current_rpm > theoretical_max_rpm:
                    self.current_rpm = theoretical_max_rpm

        # --- Publish ---
        msg_ticks = Int32()
        msg_ticks.data = self.tick_count
        self.pub_ticks.publish(msg_ticks)

        msg_rpm = Float32()
        msg_rpm.data = float(self.current_rpm)
        self.pub_rpm.publish(msg_rpm)
        
        # Log periodically (every ~1 second approx, or if fast)
        # We don't log 0 to keep terminal clean
        if self.current_rpm > 0.1:
            self.get_logger().info(f'RPM: {self.current_rpm:.2f} | Ticks: {self.tick_count}')

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