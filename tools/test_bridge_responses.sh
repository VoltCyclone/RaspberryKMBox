#!/bin/bash
# Test if bridge is sending responses

PORT="/dev/tty.usbmodem2101"

echo "Sending test command to $PORT..."
echo "km.move(10, 10)" > "$PORT"

echo "Waiting for response (2 seconds)..."
read -t 2 response < "$PORT"

if [ -n "$response" ]; then
    echo "✓ Received: $response"
else
    echo "✗ No response received"
fi
