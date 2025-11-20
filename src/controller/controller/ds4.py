import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class DS4PrintNode(Node):
    def __init__(self):
        super().__init__('ds4_print_node')
        
        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Waiting for DualShock 4 input...")

        # DS4 Mapping (Standard Bluetooth/USB config on Linux)
        # Note: Mappings can vary depending on connection (wired vs bluetooth)
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
        # Configure which buttons should trigger actions (by index)
        # Change this set to only enable the buttons you want.
        self.enabled_buttons = {0, 1}  # example: Cross (0) and Circle (1)

        # Optional: map button indices to handler methods
        self.button_handlers = {
            0: self.on_cross,
            1: self.on_circle,
        }

    # Button handler methods - customize these to perform actions
    def on_cross(self):
        self.get_logger().info("Cross (0) handler: action executed")

    def on_circle(self):
        self.get_logger().info("Circle (1) handler: action executed")

    def listener_callback(self, msg):
        # Print Button Inputs — only for enabled buttons
        for i, button_val in enumerate(msg.buttons):
            if button_val == 1:
                if i in self.button_handlers:
                    # Call the specific handler for this button
                    try:
                        self.button_handlers[i]()
                    except Exception as e:
                        self.get_logger().error(f"Error in handler for button {i}: {e}")
                elif i in self.enabled_buttons:
                    # Enabled but no handler defined: log the press
                    button_name = self.buttons_map.get(i, f"Button {i}")
                    self.get_logger().info(f"Pressed: {button_name}")
                # otherwise: ignore button presses for non-enabled buttons

        # Print Axis Inputs (only if they are moved significantly)
        for i, axis_val in enumerate(msg.axes):
            if abs(axis_val) > 0.1: # Deadzone filter
                axis_name = self.axes_map.get(i, f"Axis {i}")
                self.get_logger().info(f"Moving {axis_name}: {axis_val:.2f}")

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