#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# CHANGE: Use relative import for files in the same ROS2 package
from .PID import PID

# --- The ROS2 PID Node ---
class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # --- Parameters (Tunable via command line or yaml) ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('frequency', 20.0) # Hz

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        freq = self.get_parameter('frequency').value
        self.dt = 1.0 / freq

        self.get_logger().info(f"PID Initialized with Kp={kp}, Ki={ki}, Kd={kd}, dt={self.dt}")

        # Initialize PID Class
        # We clamp output between -1.0 and 1.0 because the motor driver 
        # expects generic float inputs which it then scales by its own power_limit.
        self.pid = PID(kp, ki, kd, self.dt, pid_min=0.0, pid_max=1.0)

        # --- State Variables ---
        self.target_value = 0.15
        self.current_value = 0.0
        self.last_update_time = self.get_clock().now()

        # --- Publishers & Subscribers ---
        
        # Publisher: Sends calculated effort to the MotorDriver
        self.pub_throttle = self.create_publisher(Float32, 'motor_throttle', 10)

        # Subscriber: The Desired Target (e.g., Target Velocity)
        self.sub_setpoint = self.create_subscription(
            Float32, 'setpoint', self.setpoint_callback, 10)

        # Subscriber: The Actual System State (e.g., Encoder Velocity)
        self.sub_state = self.create_subscription(
            Float32, 'state', self.state_callback, 10)

        # Timer: Runs the control loop at the specific frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

    def setpoint_callback(self, msg):
        """Updates the target value (Set Point)"""
        self.target_value = msg.data
        # Optional: Reset integrator if setpoint changes drastically?
        # self.pid.error_integral = 0 

    def state_callback(self, msg):
        """Updates the current value (Process Variable)"""
        self.current_value = msg.data

    def control_loop(self):
        """Calculates PID output and publishes to motor driver"""
        
        # Calculate Error
        error = self.target_value - self.current_value

        # Get Control Output from PID Class
        output = self.pid.update(error)

        # Log the current state, PID constants, and calculated throttle
        self.get_logger().info(
            f"Throttle: {output:.3f} | Error: {error:.3f} | "
            f"Kp: {self.pid.Kp} Ki: {self.pid.Ki} Kd: {self.pid.Kd}"
        )

        # Publish to Motor Driver
        # Note: Your motor driver logic is: net = fwd - rev.
        # Sending a negative value to 'motor_throttle' (fwd) results in a negative net,
        # effectively driving the motor in reverse.
        msg = Float32()
        msg.data = float(output)
        self.pub_throttle.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()