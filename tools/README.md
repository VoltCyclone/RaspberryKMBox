# Mouse Counteraction Tool

A C application that connects to a KMBox device and sends opposite mouse movements to counteract any detected movement, effectively locking the cursor in place.

## Features

- **Real-time counteraction**: Instantly sends opposite movements to cancel mouse motion
- **Configurable gain**: Adjust counteraction strength (0.0 - 2.0x)
- **Deadzone filtering**: Ignore small movements to reduce jitter
- **Cross-platform**: Works on Linux, macOS, and Windows
- **Statistics tracking**: Monitor movements detected, countered, and throughput
- **Verbose logging**: Optional detailed movement logging for debugging

## Building

### Prerequisites

- GCC or compatible C compiler
- Make (optional, can compile directly)

### Compile

```bash
# Using Make
cd tools
make

# Or compile directly
gcc -Wall -O2 -o mouse_counteract mouse_counteract.c
```

## Usage

### Basic Usage

```bash
# Linux
./mouse_counteract /dev/ttyUSB0

# macOS
./mouse_counteract /dev/tty.usbserial-*

# Windows
./mouse_counteract COM3
```

### Options

```
-b, --baud RATE      Serial baud rate (default: 115200)
-g, --gain FLOAT     Counteraction gain 0.0-2.0 (default: 1.0)
-d, --deadzone PX    Ignore movements below threshold pixels (default: 0)
-v, --verbose        Enable verbose logging
-h, --help           Show help message
```

### Examples

**Perfect counteraction (1:1 cancellation):**
```bash
./mouse_counteract -v /dev/ttyUSB0
```

**Overcorrect by 20% to compensate for latency:**
```bash
./mouse_counteract -g 1.2 /dev/ttyUSB0
```

**Ignore small jitter movements:**
```bash
./mouse_counteract -d 2 /dev/ttyUSB0
```

**Debug with verbose logging:**
```bash
./mouse_counteract -v -d 1 -g 1.1 /dev/ttyUSB0
```

**Higher baud rate (if bridge configured for it):**
```bash
./mouse_counteract -b 921600 /dev/ttyUSB0
```

## How It Works

1. **Listens** on serial port for KMBox bridge protocol packets
2. **Detects** mouse movement commands (`BRIDGE_CMD_MOUSE_MOVE`)
3. **Calculates** opposite movement: `counter = -detected * gain`
4. **Sends** counteracting movement back to KMBox
5. **Tracks** statistics for monitoring

The application uses the same binary protocol as the KMBox bridge system:

```c
// Detected packet: dx=+100, dy=+50
// Sent packet: dx=-100, dy=-50  (with gain=1.0)
```

## Statistics

Press `Ctrl+C` to gracefully stop and display statistics:

```
=== Mouse Counteraction Statistics ===
Runtime:             45 seconds
Movements detected:  1234 (27.4/sec)
Movements countered: 1234
Total motion:        dx=45678 dy=23456
Packets sent:        1234
Errors:              0
======================================
```

## Use Cases

### 1. Cursor Lock Testing
Test if your KMBox integration can keep the cursor stationary under external mouse input.

### 2. Movement Cancellation
Actually lock the cursor by counteracting all movements in real-time.

### 3. Latency Measurement
Use verbose mode to observe the delay between detection and counteraction:
```bash
./mouse_counteract -v /dev/ttyUSB0
# Watch timestamps between detected and countered movements
```

### 4. Gain Tuning
Find the optimal gain value to compensate for system latency:
- `gain < 1.0`: Partial counteraction (cursor drifts)
- `gain = 1.0`: Perfect counteraction (theoretically)
- `gain > 1.0`: Overcompensation (may oscillate)

### 5. Deadzone Testing
Determine what deadzone value eliminates jitter without affecting intentional movements.

## Protocol Details

The tool implements the KMBox bridge protocol (`bridge_protocol.h`):

- **Sync Byte**: `0xBD`
- **Mouse Move**: `[0xBD][0x01][x_lo][x_hi][y_lo][y_hi]`
- **Packet Size**: 6 bytes (little-endian)

## Troubleshooting

**No serial device found:**
```bash
# List available ports
ls /dev/tty* | grep -i usb  # Linux/macOS
mode                         # Windows
```

**Permission denied:**
```bash
# Linux - add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout/login

# Or run with sudo (not recommended)
sudo ./mouse_counteract /dev/ttyUSB0
```

**High error count:**
- Check baud rate matches bridge configuration
- Verify cable quality and connection
- Try lower baud rate: `-b 115200`
- Increase serial buffer size in kernel

**Cursor still moves:**
- Increase gain: `-g 1.2`
- Reduce deadzone: `-d 0`
- Check for processing latency with `-v`

## Performance

- **Latency**: ~2-5ms end-to-end (hardware dependent)
- **Throughput**: Handles 500+ movements/sec easily
- **CPU Usage**: <1% on modern systems
- **Memory**: ~100KB

## Advanced Configuration

### Custom Baud Rates

Edit `serial_open()` to add custom speeds:
```c
case 2000000: speed = B2000000; break;  // Linux
```

### Packet Filtering

Modify `process_mouse_packet()` to filter specific movement patterns:
```c
// Only counteract large movements
if (abs(dx) > 50 || abs(dy) > 50) {
    send_mouse_move(fd, counter_dx, counter_dy);
}
```

### Movement Scaling

Apply non-linear scaling:
```c
// Square root scaling - stronger for large movements
counter_dx = (int16_t)(-dx * g_config.gain * sqrt(abs(dx)));
```

## License

Same as parent project (see main README).

## Contributing

Bug reports and improvements welcome! This tool is designed to be simple and focused.
