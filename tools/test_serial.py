#!/usr/bin/env python3
import serial
import time

port = '/dev/tty.usbmodem2101'
baud = 115200

print(f"Opening {port} at {baud} baud...")
ser = serial.Serial(port, baud, timeout=1)
time.sleep(0.5)

# Clear any buffered data
ser.reset_input_buffer()
ser.reset_output_buffer()

# Send a test command
cmd = "km.move(10, 10)\n"
print(f"Sending: {repr(cmd)}")
ser.write(cmd.encode())
ser.flush()

# Wait and read response
time.sleep(0.1)
available = ser.in_waiting
print(f"Bytes available: {available}")

if available > 0:
    response = ser.read(available)
    print(f"Response: {repr(response)}")
    print(f"Decoded: {response.decode('ascii', errors='replace')}")
else:
    print("No response received")

# Try reading for a bit longer
print("\nWaiting 2 seconds for any data...")
time.sleep(2)
available = ser.in_waiting
if available > 0:
    response = ser.read(available)
    print(f"Delayed response: {repr(response)}")
else:
    print("Still no response")

ser.close()
