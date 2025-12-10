import spidev
import time
import sys

def test_spi():
    print("--- PIXY2 SPI HARDWARE DIAGNOSTIC ---")
    
    # 1. Open SPI Bus
    try:
        spi = spidev.SpiDev()
        spi.open(0, 0) # Bus 0, Device 0 (CE0)
        spi.max_speed_hz = 2000000 
        spi.mode = 0b00
        print("[OK] SPI Port Opened (Bus 0, Device 0)")
    except Exception as e:
        print(f"[FAIL] Could not open SPI port: {e}")
        print(" -> Check if SPI is enabled (sudo raspi-config)")
        sys.exit(1)

    print("\n[INFO] Polling for 5 seconds... Watch the output below.")
    print("-----------------------------------------------------")

    # 2. Send Request and Analyze Return Data
    # Packet: [Sync, Sync, Type=GetBlocks, Len, Sig, MaxBlocks]
    req = [0xae, 0xc1, 0x20, 0x02, 255, 0x02]
    
    start_time = time.time()
    packet_count = 0
    
    while time.time() - start_time < 5.0:
        try:
            # Send request
            spi.xfer2(req)
            
            # Read 16 bytes of response
            resp = spi.readbytes(16)
            
            # Analyze what we see
            hex_str = " ".join([f"{x:02X}" for x in resp])
            
            # Check 1: Is it all ZEROS? (Wire broken / No Power)
            if all(x == 0x00 for x in resp):
                print(f"Reading: {hex_str} -> ALL ZEROS (Check MISO wire or Pixy Power)")
                
            # Check 2: Is it all ONES? (Chip Select broken / Pull-up issue)
            elif all(x == 0xFF for x in resp):
                print(f"Reading: {hex_str} -> ALL ONES (Check GPIO 8 / CE0 connection)")
                
            # Check 3: Do we see the Magic Sync (AF C1)?
            elif 0xAF in resp and 0xC1 in resp:
                 # Check if they are adjacent
                 if bytes([0xAF, 0xC1]) in bytes(resp):
                     print(f"Reading: {hex_str} -> SUCCESS! Pixy detected.")
                     print(" -> CONCLUSION: Hardware is perfect. Problem is in software/training.")
                     spi.close()
                     return
                 else:
                     print(f"Reading: {hex_str} -> Partial Sync (Data Misaligned)")

            # Check 4: Random Junk?
            else:
                print(f"Reading: {hex_str} -> NOISE (Check wiring quality / Baud rate)")

            time.sleep(0.5)
            packet_count += 1
            
        except KeyboardInterrupt:
            break

    print("-----------------------------------------------------")
    print("DIAGNOSTIC COMPLETE.")
    spi.close()

if __name__ == "__main__":
    test_spi()