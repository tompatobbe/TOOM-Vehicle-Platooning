#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from .PID import PID

# --- The ROS2 PID Node ---
class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # --- Parameters (Tunable via command line or yaml) ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('frequency', 20.0) # Hz

        # --- ENABLE DYNAMIC UPDATES ---
        self.add_on_set_parameters_callback(self.parameters_callback)

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
        self.target_value = 0.5
        self.current_value = 0.0
        self.last_update_time = self.get_clock().now()

        # --- Publishers & Subscribers ---
        
        # Publisher: Sends calculated effort to the MotorDriver
        self.pub_throttle = self.create_publisher(Float32, 'follower1/motor_throttle', 10)

        # Subscriber: The Desired Target (e.g., Target Velocity)
        self.sub_setpoint = self.create_subscription(
            Float32, 'follower1/setpoint', self.setpoint_callback, 10)

        # Subscriber: The Actual System State (e.g., Encoder Velocity)
        self.sub_state = self.create_subscription(
            Float32, 'follower1/state', self.state_callback, 10)

        # Timer: Runs the control loop at the specific frequency
        self.timer = self.create_timer(self.dt, self.control_loop)


    def parameters_callback(self, params):
        """Called when user updates params via rqt or CLI"""
        for param in params:
            if param.name == 'kp':
                self.pid.Kp = param.value
            elif param.name == 'ki':
                self.pid.Ki = param.value
            elif param.name == 'kd':
                self.pid.Kd = param.value
                
            self.get_logger().info(f"Updated {param.name} to {param.value}")
            
        return SetParametersResult(successful=True)

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
        error = -error  # Invert error if necessary (e.g., for distance control)

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

    ## Syntax: ros2 param set <node_name> <param_name> <value>

    # ros2 param set /pid_controller_node kp 2.5
    # ros2 param set /pid_controller_node ki 0.1