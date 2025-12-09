#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import spidev
import struct

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        self.publisher_ = self.create_publisher(Point, '/pixy/target', 10)
        
        # SPI Setup
        self.spi_bus = 0
        self.spi_device = 0
        self.spi = spidev.SpiDev()
        
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
        # [Sync, Sync, Type=32, Len=2, Sig=1, MaxBlocks=2]
        req = [0xae, 0xc1, 0x20, 0x02, 1, 0x02]
        try:
            self.spi.xfer2(req)
        except OSError:
            return

        # 2. Read a BIG chunk (64 bytes)
        # This captures the response even if there are garbage bytes at the start
        response = self.spi.readbytes(64)
        data_bytes = bytearray(response)

        # 3. Find the Sync Word (0xaf 0xc1) in the chunk
        # We search through the array to find where the packet actually starts
        try:
            # find() returns the index of the sequence, or -1 if not found
            idx = data_bytes.find(b'\xaf\xc1')
            
            if idx != -1:
                # We found the start! 
                # The Header is 4 bytes AFTER the sync word (idx + 2)
                # Header: [Type, Length, CsumL, CsumH]
                
                # Safety check: do we have enough bytes left for a header?
                if idx + 6 > len(data_bytes):
                    return

                pkt_type = data_bytes[idx + 2]
                payload_len = data_bytes[idx + 3]
                
                # Check if it's the right packet type (0x21 = 33 = GetBlocks Response)
                if pkt_type == 0x21 and payload_len > 0:
                    
                    # The payload starts after the header (6 bytes after sync)
                    payload_start = idx + 6
                    
                    # Safety check: do we have enough bytes for the payload?
                    if payload_start + payload_len <= len(data_bytes):
                        payload = data_bytes[payload_start : payload_start + payload_len]
                        self.parse_block(payload)

        except Exception as e:
            self.get_logger().warn(f"Scan Error: {e}")

    def parse_block(self, payload):
        # We only care about the first block (14 bytes)
        if len(payload) >= 14:
            try:
                # Unpack: Sig(H), X(H), Y(H), Width(H), Height(H), Angle(h)
                # < = Little Endian
                data = struct.unpack_from('<HHHHh', payload, offset=0)
                sig, x, y, w, h, angle = data
                
                # --- PUBLISH ---
                msg = Point()
                msg.x = float(x)
                msg.y = float(y)
                msg.z = float(w) # Width acts as "Distance"
                self.publisher_.publish(msg)

                # --- PRINT (So you can see it!) ---
                self.get_logger().info(f"Target Found! X={x}, Y={y}, Width={w}")
                
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