#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import spidev
import struct
import time

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        
        # --- CONFIGURATION ---
        self.spi_bus = 0
        self.spi_device = 0
        self.target_signature = 1  # Look for Signature 1 by default
        
        # Initialize SPI
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000 # 2 MHz is reliable for Pixy2
            self.spi.mode = 0b00
            self.get_logger().info(f"SPI Connected. Polling for objects...")
        except Exception as e:
            self.get_logger().error(f"Failed to open SPI: {e}")
            return

        # Poll at 20Hz (approx every 0.05s)
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        # --- STEP 1: Send Request ---
        # Pixy2 Packet for "getBlocks": 
        # [No Checksum(0xae), C1, Type(32), Length(2), Sigmap(1), MaxBlocks(255)]
        # 0xae, 0xc1 = Sync (no checksum)
        # 0x20 = Type 32 (getBlocks)
        # 0x02 = Data Length (2 bytes follow)
        # 0x01 = Signature 1 (or use 255 for all)
        # 0x02 = Max blocks to return (keep small to save SPI bandwidth)
        
        request = [0xae, 0xc1, 0x20, 0x02, self.target_signature, 0x02]
        self.spi.xfer2(request)

        # --- STEP 2: Read Header ---
        # We need to read enough bytes to catch the response. 
        # A response header is 6 bytes, plus 14 bytes per detected block.
        # Let's read 32 bytes to be safe.
        response = self.spi.readbytes(32)
        
        # --- STEP 3: Parse Response ---
        # Look for the sync bytes 0xAF, 0xC1 in the response
        # Note: Pixy2 sends 0xAF, 0xC1 as the response sync.
        
        frame_start = -1
        for i in range(len(response) - 1):
            if response[i] == 0xaf and response[i+1] == 0xc1:
                frame_start = i
                break
        
        if frame_start == -1:
            # No sync found (Pixy might be busy or sees nothing)
            return

        # Decode the Header
        # Offset: 0=AF, 1=C1, 2=Type, 3=Length, 4=ChecksumL, 5=ChecksumH
        try:
            data_type = response[frame_start + 2]
            length = response[frame_start + 3]
            
            # Type 33 (0x21) is the response to getBlocks
            if data_type == 0x21 and length > 0:
                # The payload starts at index 6
                payload_idx = frame_start + 6
                
                # Each block is 14 bytes long
                num_blocks = length // 14
                
                if num_blocks > 0:
                    self.parse_blocks(response, payload_idx, num_blocks)
            else:
                # Type 33 with length 0 means "No blocks detected"
                pass 

        except IndexError:
            pass # Read buffer wasn't big enough, ignore frame

    def parse_blocks(self, buffer, start_index, count):
        current_idx = start_index
        
        print("-" * 30)
        for i in range(count):
            if current_idx + 14 > len(buffer):
                break
                
            # Parse one block (14 bytes)
            # Bytes: 0-1:Sig, 2-3:X, 4-5:Y, 6-7:W, 8-9:H, 10-11:Angle, etc.
            # Using struct.unpack to read little-endian short integers (<H)
            
            block_bytes = bytearray(buffer[current_idx : current_idx+14])
            
            # Format: H(Sig), H(X), H(Y), H(W), H(H), h(Angle), B(Index), B(Age)
            try:
                sig, x, y, w, h, angle, idx, age = struct.unpack('<HHHHhBB', block_bytes)
                
                # Print to terminal as requested
                self.get_logger().info(f"OBJ {i+1}: Sig={sig} | X={x}, Y={y} | W={w}, H={h}")
                
            except struct.error:
                self.get_logger().warn("Packet parsing error")

            current_idx += 14
            
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