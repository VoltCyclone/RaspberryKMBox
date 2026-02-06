#!/bin/bash
# Quick Test Script for KMBox Humanization
# Usage: ./quick_test.sh

echo "🎯 KMBox Humanization Quick Test"
echo "================================="
echo ""

# Find the device
DEVICE=$(ls /dev/tty.usbmodem* 2>/dev/null | head -n1)

if [ -z "$DEVICE" ]; then
    echo "❌ No KMBox device found on /dev/tty.usbmodem*"
    exit 1
fi

echo "✅ Found device: $DEVICE"
echo ""

# Check if stress test exists
if [ ! -f "kmbox_stress_test.py" ]; then
    echo "❌ kmbox_stress_test.py not found"
    echo "   Run this script from the tools/ directory"
    exit 1
fi

echo "Running quick diagnostic tests..."
echo ""

# Test 1: Rapid movements
echo "📍 Test 1: Rapid Small Movements (1000 iterations)"
python3 -c "
import serial
import time

ser = serial.Serial('$DEVICE', 921600, timeout=0.1)
time.sleep(0.5)

start = time.time()
for i in range(1000):
    ser.write(b'km.move(0, 10)\n')
    ser.write(b'km.move(0, -10)\n')

elapsed = time.time() - start
rate = 2000 / elapsed
print(f'  Rate: {rate:.0f} commands/sec')
print(f'  Result: {\"✅ PASS\" if rate > 500 else \"❌ FAIL\"}')

ser.close()
"

echo ""

# Test 2: Small precise
echo "📍 Test 2: Small Precise Movements"
python3 -c "
import serial
import time

ser = serial.Serial('$DEVICE', 921600, timeout=0.1)
time.sleep(0.5)

movements = [(1,0), (0,1), (-1,0), (0,-1), (2,0), (0,2), (-2,0), (0,-2)]
for dx, dy in movements * 10:
    ser.write(f'km.move({dx}, {dy})\n'.encode())
    time.sleep(0.01)

print('  Result: ✅ Watch cursor - light jitter = GOOD')

ser.close()
"

echo ""

# Test 3: Large flicks  
echo "📍 Test 3: Large Fast Flicks"
python3 -c "
import serial
import time

ser = serial.Serial('$DEVICE', 921600, timeout=0.1)
time.sleep(0.5)

flicks = [(200,0), (-200,0), (0,150), (0,-150)]
for dx, dy in flicks * 5:
    ser.write(f'km.move({dx}, {dy})\n'.encode())
    time.sleep(0.05)

print('  Result: ✅ Should feel SNAPPY, minimal jitter')

ser.close()
"

echo ""
echo "================================="
echo "Quick test complete!"
echo ""
echo "For full test suite, run:"
echo "  python3 kmbox_stress_test.py $DEVICE"
echo ""
echo "To adjust humanization:"
echo "  LOW:    Light (competitive)   - ±0.03px"
echo "  MEDIUM: Balanced (default)    - ±0.06px"
echo "  HIGH:   Maximum (stealth)     - ±0.1px"
