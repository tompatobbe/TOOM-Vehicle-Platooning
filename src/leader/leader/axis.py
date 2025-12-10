import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickDebugger(Node):
    def __init__(self):
        super().__init__('joy_debugger')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.get_logger().info("Press R2 to see which axis changes...")

    def listener_callback(self, msg):
        # Print all axes to see which one moves when you press R2
        # usually it is index 5
        self.get_logger().info(f'Axes: {msg.axes}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()