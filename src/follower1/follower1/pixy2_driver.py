#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import spidev
import struct

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        
        # --- CONFIGURATION ---
        self.spi_bus = 0
        self.spi_device = 0
        
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000 
            self.spi.mode = 0b00
            self.get_logger().info("Pixy2 Driver Running. Polling for objects...")
        except Exception as e:
            self.get_logger().error(f"SPI Error: {e}")
            return

        self.timer = self.create_timer(0.04, self.update)

    def update(self):
        # Request blocks: [Sync, Sync, Type=32, Len=2, Sig=1, MaxBlocks=2]
        # (Sig=255 requests ALL signatures)
        req = [0xae, 0xc1, 0x20, 0x02, 255, 0x02]
        
        try:
            self.spi.xfer2(req)
        except OSError:
            return 

        # Hunt for Sync (0xaf, 0xc1)
        for _ in range(50):
            try:
                b = self.spi.readbytes(1)[0]
                if b == 0xaf:
                    if self.spi.readbytes(1)[0] == 0xc1:
                        self.process_packet()
                        return
            except Exception:
                pass

    def process_packet(self):
        # Read Header: [Type, Length, ChecksumL, ChecksumH]
        header = self.spi.readbytes(4)
        packet_type = header[0]
        payload_len = header[1]
        
        if payload_len == 0 or packet_type != 0x21:
            return

        # Read the full payload
        payload = self.spi.readbytes(payload_len)
        self.parse_blocks(payload)

    def parse_blocks(self, payload):
        # Standard Pixy2 block is 14 bytes.
        BLOCK_SIZE = 14 
        
        # Determine how many full blocks we have
        num_blocks = len(payload) // BLOCK_SIZE
        
        for i in range(num_blocks):
            start_idx = i * BLOCK_SIZE
            
            # --- THE FIX ---
            # Instead of requiring an exact buffer match, we use 'unpack_from'.
            # We only care about the first 12 bytes (Sig, X, Y, W, H, Angle).
            # We safely ignore the last 2 bytes (Index, Age) to prevent errors.
            
            try:
                # < = Little Endian
                # H = unsigned short (2 bytes)
                # h = signed short (2 bytes)
                # Format: Sig(H), X(H), Y(H), W(H), H(H), Angle(h) = 12 bytes
                data = struct.unpack_from('<HHHHh', payload, offset=start_idx)
                
                sig, x, y, w, h, angle = data
                
                # Print to terminal
                self.get_logger().info(f"Detected: Sig={sig} X={x} Y={y} W={w} H={h}")
                
            except struct.error as e:
                self.get_logger().warn(f"Parsing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Pixy2SpiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt