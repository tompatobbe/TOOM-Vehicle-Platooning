#!/usr/bin/env python3
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import sys

# --- SETUP ---
PIN = 12  # Your GPIO Pin
factory = PiGPIOFactory()

# Standard servo range is 0.5ms to 2.5ms (0.0005 to 0.0025)
# We start at neutral 1.5ms
current_pulse = 0.0015 

servo = Servo(PIN, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

print("--- SERVO CALIBRATION TOOL ---")
print("Instructions:")
print("  'a' + Enter:  Move LEFT  (Micro Adjustment)")
print("  'd' + Enter:  Move RIGHT (Micro Adjustment)")
print("  'A' + Enter:  Move LEFT  (Coarse Adjustment)")
print("  'D' + Enter:  Move RIGHT (Coarse Adjustment)")
print("  'q' + Enter:  Quit")
print("------------------------------")

try:
    while True:
        servo.value = (current_pulse - 0.0015) * 2 # Convert pulse to -1 to 1 value for gpiozero
        
        print(f"Current Pulse Width: {current_pulse:.6f}")
        cmd = input("Command: ")

        if cmd == 'a':
            current_pulse -= 0.00001  # -10 microseconds
        elif cmd == 'd':
            current_pulse += 0.00001  # +10 microseconds
        elif cmd == 'A':
            current_pulse -= 0.0001   # -100 microseconds
        elif cmd == 'D':
            current_pulse += 0.0001   # +100 microseconds
        elif cmd == 'q':
            break
            
        # Safety clamps (Don't go below 0.5ms or above 2.5ms)
        current_pulse = max(0.0005, min(0.0025, current_pulse))

except KeyboardInterrupt:
    pass