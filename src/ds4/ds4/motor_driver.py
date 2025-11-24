#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pigpio
import time
import math

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester_node')
        
        # --- Parameters ---
        # Change this to the BCM GPIO pin you are using (e.g., Pin 12 is GPIO 18)
        self.declare_parameter('gpio_pin', 18)
        self.pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value

        self.get_logger().info(f"Initializing Motor Tester on GPIO {self.pin}")

        # --- Hardware Setup ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon! Did you run 'sudo pigpiod'?")
            exit()

        # --- ESC Constants for WP-1040 ---
        # 50Hz frequency is standard for RC ESCs
        # 1500us is center (neutral)
        # 1000us is full reverse
        # 2000us is full forward
        self.NEUTRAL_PW = 1500

        # SAFETY LIMIT: Only go to 15% power for testing
        # Full range is +/- 500. 15% is +/- 75.
        self.POWER_LIMIT = 0.15 
        
        self.MAX_FWD_PW = 1500 + int(500 * self.POWER_LIMIT)
        self.MAX_REV_PW = 1500 - int(500 * self.POWER_LIMIT)
        
        # State variables for the test sequence
        self.test_state = 'ARMING'
        self.current_pw = self.NEUTRAL_PW
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Create a timer to run the control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initial Hardware setup
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.pin, 0) # Off initially

    def set_speed(self, pulse_width):
        """Safely writes the pulse width to the ESC"""
        # Clamp values for safety
        pulse_width = max(self.MAX_REV_PW, min(pulse_width, self.MAX_FWD_PW))
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        # --- PHASE 1: ARMING ---
        # The WP-1040 needs to see Neutral for a few seconds to "Arm" (Stop beeping)
        if self.test_state == 'ARMING':
            self.set_speed(self.NEUTRAL_PW)
            self.get_logger().info(f"Arming ESC (Neutral)... {int(elapsed)}s")
            
            if elapsed > 3.0:
                self.test_state = 'RAMP_UP'
                self.get_logger().info("Arming Complete. Starting Forward Ramp.")

        # --- PHASE 2: RAMP FORWARD ---
        elif self.test_state == 'RAMP_UP':
            # Increment pulse width
            self.current_pw += 5 # Reduced step size (was 20) for slower acceleration
            
            if self.current_pw >= self.MAX_FWD_PW:
                self.current_pw = self.MAX_FWD_PW
                self.test_state = 'RAMP_DOWN'
                self.get_logger().info("Max Forward Reached. Ramping Down.")
            
            self.set_speed(self.current_pw)
            self.get_logger().info(f"Testing Forward: {self.current_pw}us")

        # --- PHASE 3: RAMP DOWN TO NEUTRAL ---
        elif self.test_state == 'RAMP_DOWN':
            self.current_pw -= 5 # Reduced step size
            
            if self.current_pw <= self.NEUTRAL_PW:
                self.current_pw = self.NEUTRAL_PW
                self.test_state = 'PAUSE'
                self.pause_start = now
                self.get_logger().info("Neutral Reached. Pausing before Reverse.")
            
            self.set_speed(self.current_pw)

        # --- PHASE 4: PAUSE ---
        elif self.test_state == 'PAUSE':
            self.set_speed(self.NEUTRAL_PW)
            if (now - self.pause_start) > 2.0:
                self.test_state = 'RAMP_REVERSE'
                self.get_logger().info("Starting Reverse Ramp.")

        # --- PHASE 5: RAMP REVERSE ---
        elif self.test_state == 'RAMP_REVERSE':
            self.current_pw -= 5 # Reduced step size
            
            if self.current_pw <= self.MAX_REV_PW:
                self.current_pw = self.MAX_REV_PW
                self.test_state = 'FINISHED'
                self.get_logger().info("Max Reverse Reached. Test Complete.")
            
            self.set_speed(self.current_pw)
            self.get_logger().info(f"Testing Reverse: {self.current_pw}us")

        # --- PHASE 6: FINISHED ---
        elif self.test_state == 'FINISHED':
            self.set_speed(self.NEUTRAL_PW)
            self.get_logger().info("Test Cycle Done. Idling.")
            # Optional: self.destroy_node() to exit

    def cleanup(self):
        """Stop motor on shutdown"""
        self.get_logger().info("Stopping Motor...")
        self.pi.set_servo_pulsewidth(self.pin, 0) # 0 kills the signal
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MotorTester()
    
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