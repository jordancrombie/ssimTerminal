#!/usr/bin/env python3
"""Simple serial monitor for debugging ssimTerminal"""
import serial
import sys
import time

PORT = "/dev/cu.usbmodem2101"
BAUD = 115200
TIMEOUT = 30  # seconds

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to {PORT} at {BAUD} baud")
        print("Monitoring for {} seconds... (Ctrl+C to stop)\n".format(TIMEOUT))
        print("-" * 60)

        start = time.time()
        while time.time() - start < TIMEOUT:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore')
                print(line, end='')

        ser.close()
        print("\n" + "-" * 60)
        print("Monitoring complete")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped by user")
        ser.close()

if __name__ == "__main__":
    main()
