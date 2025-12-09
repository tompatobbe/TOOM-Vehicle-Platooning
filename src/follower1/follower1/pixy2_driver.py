#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Standard ROS message for X,Y,Z
import spidev
import struct

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        
        # 1. CREATE PUBLISHER
        # We will publish to the topic '/pixy/target'
        # Message Type: Point (x=X coord, y=Y coord, z=Width)
        self.publisher_ = self.create_publisher(Point, '/pixy/target', 10)
        
        # --- SPI SETUP ---
        self.spi_bus = 0
        self.spi_device = 0
        self.spi = spidev.SpiDev()
        
        try:
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000 
            self.spi.mode = 0b00
            self.get_logger().info("Pixy2 Node Started. Publishing to /pixy/target")
        except Exception as e:
            self.get_logger().error(f"SPI connection failed: {e}")
            return

        # Poll rapidly (25Hz = 0.04s)
        self.timer = self.create_timer(0.04, self.update)

    def update(self):
        # Request Block Data (Sig 1)
        req = [0xae, 0xc1, 0x20, 0x02, 1, 0x02]
        try:
            self.spi.xfer2(req)
        except OSError:
            return 

        # Scan for Sync (0xaf 0xc1)
        for _ in range(50):
            try:
                b = self.spi.readbytes(1)[0]
                if b == 0xaf:
                    if self.spi.readbytes(1)[0] == 0xc1:
                        self.process_packet()
                        return
            except:
                pass

    def process_packet(self):
        header = self.spi.readbytes(4)
        if len(header) < 4: return
        
        pkt_type = header[0]
        payload_len = header[1]
        
        if pkt_type != 0x21 or payload_len == 0: return

        payload = self.spi.readbytes(payload_len)
        self.parse_blocks(payload)

    def parse_blocks(self, payload):
        # We only care about the first/largest block for following
        # Pixy sorts them by size automatically (largest first)
        
        try:
            # Unpack first 12 bytes: Sig, X, Y, Width, Height, Angle
            data = struct.unpack_from('<HHHHh', payload, offset=0)
            sig, x, y, w, h, angle = data
            
            # 2. CREATE AND PUBLISH MESSAGE
            msg = Point()
            msg.x = float(x) # Pixy X (0-315)
            msg.y = float(y) # Pixy Y (0-207)
            msg.z = float(w) # We use Z to store Width (approx proximity)
            
            self.publisher_.publish(msg)
            
            # Optional: Print occasionally so you know it's working
            self.get_logger().info(f"Published: X={x}, Y={y}")
            
        except struct.error:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = Pixy2SpiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()