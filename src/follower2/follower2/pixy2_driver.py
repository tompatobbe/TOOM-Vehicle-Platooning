#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # CHANGED: Needs to match ServoController input
import spidev
import struct

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        
        # CHANGED: Update topic and type to match the Servo Controller
        self.publisher_ = self.create_publisher(Float32, 'follower2/servo/angle', 10)
        
        # SPI Setup
        self.spi_bus = 0
        self.spi_device = 0
        self.spi = spidev.SpiDev()
        self.max_turn_pixy = 315    
        self.max_turn_angle = 60
        self.offset_angle = 10.0
        
        try:
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000
            self.spi.mode = 0b00
            self.get_logger().info("Pixy2 Driver: SPI Connected. Scanning for blocks...")
        except Exception as e:
            self.get_logger().error(f"SPI Failed: {e}")
            return

        # Poll at 30Hz
        self.timer = self.create_timer(0.03, self.update)

    def update(self):
        # 1. Send Request (Ask for Sig 1)
        req = [0xae, 0xc1, 0x20, 0x02, 1, 0x02]
        try:
            self.spi.xfer2(req)
        except OSError:
            return

        # 2. Read a BIG chunk (64 bytes) to capture the response
        response = self.spi.readbytes(64)
        data_bytes = bytearray(response)

        # 3. Find the Sync Word (0xaf 0xc1)
        try:
            idx = data_bytes.find(b'\xaf\xc1')
            
            if idx != -1:
                # Safety check: do we have enough bytes left for a header?
                if idx + 6 > len(data_bytes):
                    return

                pkt_type = data_bytes[idx + 2]
                payload_len = data_bytes[idx + 3]
                
                # Check if it's the right packet (GetBlocks Response = 0x21)
                if pkt_type == 0x21 and payload_len > 0:
                    payload_start = idx + 6
                    
                    if payload_start + payload_len <= len(data_bytes):
                        payload = data_bytes[payload_start : payload_start + payload_len]
                        self.parse_block(payload)

        except Exception as e:
            self.get_logger().warn(f"Scan Error: {e}")

    def parse_block(self, payload):
        # We need at least 12 bytes for the coordinates (Sig, X, Y, W, H, Angle)
        if len(payload) >= 12:
            try:
                # Format: H(Sig) H(X) H(Y) H(W) H(H) h(Angle) -> 6 items
                data = struct.unpack_from('<HHHHHh', payload, offset=0)
                
                sig, x, y, w, h, angle = data
                
                # --- CALCULATION LOGIC ---
                # Pixy2 X range is 0 to 315. Center is ~157.
                # Servo code does: target = 60 - msg.data
                # We want Center (157) to result in Servo Middle (~50 deg).
                # 50 = 60 - msg -> msg must be 10 when centered.
                
                # Proportional Controller (P-Controller)
                center_x = 157.5    
                # Positive error means object is to the left (X < 157.5)

                control_val = x / self.max_turn_pixy * self.max_turn_angle

                control_val = self.max_turn_angle - control_val + self.offset_angle
                
                # Gain (Kp): Maps pixel error to angle. 0.3 is a standard starting point.
                # If X=0 (Far Left), Error=157.5. Val = 157.5*0.3 + 10 = ~57. Servo = 60-57 = 3 deg.
                # If X=315 (Far Right), Error=-157.5. Val = -157.5*0.3 + 10 = ~-37. Servo = 60-(-37) = 97 deg.
                kp = 0.2
                #control_val = (error * kp) #+ 10.0

                # Publish
                msg = Float32()
                msg.data = float(control_val)
                self.publisher_.publish(msg)

                # Print to terminal
                #self.get_logger().info(f"Target X={x} | Sending Val={control_val:.2f}")
                
            except struct.error as e:
                self.get_logger().warn(f"Unpack Error: {e}")

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