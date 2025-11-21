#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .PID import PID


class SteeringController(Node):
    def __init__(self):
        super().__init__('steering_controller')
        
        # --- Parameters ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('dt', 0.01)  # 10ms update rate
        self.declare_parameter('min_angle', 0.0)
        self.declare_parameter('max_angle', 60.0)
        
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        dt = self.get_parameter('dt').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        
        # --- PID Controller ---
        self.pid = PID(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            pid_min=self.min_angle,
            pid_max=self.max_angle
        )
        
        # --- ROS Communication ---
        # Subscribe to desired angle from DS4
        self.desired_sub = self.create_subscription(
            Float32,
            '/servo/desired_angle',
            self.desired_callback,
            10
        )
        
        # Subscribe to actual servo angle feedback (optional, for better control)
        self.feedback_sub = self.create_subscription(
            Float32,
            '/servo/feedback_angle',
            self.feedback_callback,
            10
        )
        
        # Publish corrected servo command
        self.servo_pub = self.create_publisher(
            Float32,
            '/servo/angle',
            10
        )
        
        self.current_desired = 30.0  # Start at middle
        self.current_feedback = 30.0
        
        self.get_logger().info("Steering controller initialized")
    
    def desired_callback(self, msg):
        """Receive desired angle from DS4 controller"""
        self.current_desired = msg.data
    
    def feedback_callback(self, msg):
        """Receive actual angle feedback from servo (if available)"""
        self.current_feedback = msg.data
        
        # Calculate error and apply PID control
        error = self.current_desired - self.current_feedback
        corrected_angle = self.pid.update(error)
        
        # Publish corrected command
        servo_msg = Float32()
        servo_msg.data = corrected_angle
        self.servo_pub.publish(servo_msg)
        
        self.get_logger().debug(
            f"Desired: {self.current_desired:.2f}° | "
            f"Feedback: {self.current_feedback:.2f}° | "
            f"Corrected: {corrected_angle:.2f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SteeringController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
