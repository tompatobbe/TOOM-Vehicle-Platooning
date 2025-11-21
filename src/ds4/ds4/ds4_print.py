import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class DS4PrintNode(Node):
    def __init__(self):
        super().__init__('ds4_print_node')
        
        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        
        # Publish desired angle to steering controller
        self.servo_publisher = self.create_publisher(
            Float32,
            '/servo/desired_angle',
            10)
        
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Waiting for DualShock 4 input...")

        # DS4 Mapping
        self.buttons_map = {
            0: "Cross", 1: "Circle", 2: "Triangle", 3: "Square",
            4: "L1", 5: "R1", 6: "L2", 7: "R2",
            8: "Share", 9: "Options", 10: "PS Button",
            11: "L3", 12: "R3"
        }

        self.axes_map = {
            0: "Left Stick L/R", 1: "Left Stick U/D",
            2: "L2 Analog", 3: "Right Stick L/R",
            4: "Right Stick U/D", 5: "R2 Analog"
        }
        
        self.enabled_buttons = {} 
        self.button_handlers = {}
        
        # Servo control parameters
        self.min_servo_angle = 0
        self.max_servo_angle = 60
        self.current_angle = 30  # Start at middle

    # Button handler methods (unused)
    def on_cross(self): pass
    def on_circle(self): pass
    def on_triangle(self): pass
    def on_square(self): pass

    def listener_callback(self, msg):
        # Print Button Inputs (Only if enabled)
        for i, button_val in enumerate(msg.buttons):
            if button_val == 1:
                if i in self.button_handlers:
                    try:
                        self.button_handlers[i]()
                    except Exception as e:
                        self.get_logger().error(f"Error in handler: {e}")
                elif i in self.enabled_buttons:
                    button_name = self.buttons_map.get(i, f"Button {i}")
                    self.get_logger().info(f"Pressed: {button_name}")

        # Control servo with Left Stick L/R (axis 0)
        if len(msg.axes) > 0:
            axis_val = msg.axes[0]  # Left Stick L/R
            if abs(axis_val) > 0.05:  # Deadzone filter
                # Map joystick range [-1, 1] to servo angle range
                angle_range = self.max_servo_angle - self.min_servo_angle
                self.current_angle = self.min_servo_angle + (angle_range * (axis_val + 1) / 2)
            else:
                # Reset to middle when no input
                self.current_angle = (self.min_servo_angle + self.max_servo_angle) / 2
            
            # Publish servo command
            servo_msg = Float32()
            servo_msg.data = self.current_angle
            self.servo_publisher.publish(servo_msg)
            
            self.get_logger().info(f"Servo angle: {self.current_angle:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = DS4PrintNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()