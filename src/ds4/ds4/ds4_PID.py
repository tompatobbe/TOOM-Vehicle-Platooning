#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class ServoPIDController(Node):
    def __init__(self):
        super().__init__('servo_pid_controller')

        # --- PID Configuration ---
        # default kp=0.1, ki=0.0, kd=0.0 acts as a P-controller
        self.declare_parameter('kp', 0.1)   # Proportional (Speed/Strength)
        self.declare_parameter('ki', 0.0)   # Integral (Fixes steady drift)
        self.declare_parameter('kd', 0.0)   # Derivative (Dampens oscillation)
        self.declare_parameter('control_axis', 4) 

        # --- PID Variables ---
        self.prev_error = 0.0       # For Derivative
        self.integral_sum = 0.0     # For Integral
        self.integral_max = 10.0    # Anti-windup cap (prevents I from getting too huge)
        
        # --- Simulation/State Variables ---
        self.current_angle = 30.0   # Estimated current state
        self.target_angle = 30.0    # Desired state from Joystick
        self.servo_min = 0.0
        self.servo_max = 60.0

        # --- Timing ---
        self.frequency = 20.0       # Hz
        self.dt = 1.0 / self.frequency

        # --- Communication ---
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.servo_pub = self.create_publisher(Float32, '/servo/angle', 10)

        # --- Control Loop ---
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("PID Controller Initialized")
        self.get_logger().info(f"Gains -> P: {self.get_parameter('kp').value}, I: {self.get_parameter('ki').value}, D: {self.get_parameter('kd').value}")

    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joy_callback(self, msg):
        axis = self.get_parameter('control_axis').value
        if axis < len(msg.axes):
            # Map Joystick (-1..1) to Servo Angle (0..60)
            self.target_angle = self.map_value(msg.axes[axis], -1.0, 1.0, self.servo_min, self.servo_max)

    def control_loop(self):
        # Update parameters dynamically (allows tuning while running)
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        # 1. Calculate Error
        error = self.target_angle - self.current_angle

        # 2. Proportional Term
        p_out = kp * error

        # 3. Integral Term (Accumulates error over time)
        self.integral_sum += error * self.dt
        # Anti-windup: Clamp the integral sum so it doesn't grow infinitely
        self.integral_sum = max(min(self.integral_sum, self.integral_max), -self.integral_max)
        i_out = ki * self.integral_sum

        # 4. Derivative Term (Rate of change of error)
        derivative = (error - self.prev_error) / self.dt
        d_out = kd * derivative

        # 5. Total Control Output
        control_signal = p_out + i_out + d_out

        # 6. Apply Output to System
        # Since we don't have a real physics plant, we simulate the servo moving
        # by adding the control signal to the current angle.
        self.current_angle += control_signal

        # 7. Clamp physical limits
        self.current_angle = max(self.servo_min, min(self.servo_max, self.current_angle))

        # 8. Store error for next loop (for D term)
        self.prev_error = error

        # 9. Publish
        msg = Float32()
        msg.data = float(self.current_angle)
        self.servo_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServoPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()