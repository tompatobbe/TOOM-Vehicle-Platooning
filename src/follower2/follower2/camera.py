#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import spidev
import time

class PixySpiNode(Node):
    def __init__(self):
        super().__init__('pixy_spi_viewer')
        
        # 1. Parameterize configuration (optional, but good practice)
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('frequency', 10.0) # Hz

        bus = self.get_parameter('spi_bus').value
        device = self.get_parameter('spi_device').value
        
        # 2. Setup SPI Connection
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(bus, device)
            
            # Pixy specific SPI settings
            # Pixy usually works well with Mode 0 and speeds around 1-2 MHz usually
            self.spi.max_speed_hz = 1000000 
            self.spi.mode = 0b00 
            
            self.get_logger().info(f"SPI Initialized on Bus {bus}, Device {device}")

        except Exception as e:
            self.get_logger().error(f"Failed to open SPI: {e}")
            # We don't exit here to keep the node alive, but logic won't work
            self.spi = None

        # 3. Create a timer to poll the SPI bus
        # We use a timer instead of a while loop so ROS callbacks remain active
        timer_period = 1.0 / self.get_parameter('frequency').value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.spi is None:
            return

        try:
            # 4. Read data from SPI
            # SPI is full-duplex. To read, we technically "send" dummy bytes (usually 0x00).
            # Pixy packets vary in size, but let's read a chunk (e.g., 16 bytes) to see activity.
            bytes_to_read = 16
            
            # readbytes() sends 0x00 while reading
            rx_data = self.spi.readbytes(bytes_to_read)

            # 5. Format and Print the data
            # Convert decimal list to Hex strings for easier reading
            hex_data = ['{:02x}'.format(x) for x in rx_data]
            
            # Pixy 'sync' words are usually 0xaf, 0xc1 (or similar depending on version)
            # This helps you visually spot if you are aligned with a packet
            self.get_logger().info(f"Incoming SPI Data: {hex_data}")

        except Exception as e:
            self.get_logger().error(f"Error reading SPI: {e}")

    def destroy_node(self):
        # Clean up SPI on shutdown
        if self.spi:
            self.spi.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PixySpiNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()