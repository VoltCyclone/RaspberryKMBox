# KMBox Bridge - Advanced Autopilot Firmware

> **📖 For overview, architecture, and quick setup, see the [main README](../README.md#kmbox-bridge---advanced-autopilot).**

Advanced autopilot firmware for **Adafruit Feather RP2350** with color-based target tracking, PIO UART communication, and LED status indicators.

## Detailed Wiring

| Feather RP2350 Pin | Direction | KMBox Pico Pin | Description |
|--------------------|-----------|----------------|-------------|
| GPIO0 (TX)         | ────►     | RX pin         | Bridge transmits commands |
| GND                | ────      | GND            | Common ground |

**Note:** One-way communication (bridge → KMBox). No RX connection needed on bridge.

## Protocol Details

### PC → Bridge (USB CDC)
- Frame header: `['F', 'R', width_low, width_high, height_low, height_high, 0x00, 0x00]`
- Frame data: RGB888 pixels (width × height × 3 bytes)
- Config commands: `['C', r_min, r_max, g_max, b_max, gain_x*10, gain_y*10, deadzone, min_blob]`

### Bridge → KMBox (PIO UART @ 115200 baud)
- Mouse move: `[0x01, dx_low, dx_high, dy_low, dy_high, buttons]`

## Python Client Usage

Install pyserial:
```bash
pip install pyserial
```

### Command Line
```bash
python bridge_client.py --list    # List available ports
python bridge_client.py --test    # Test connection  
python bridge_client.py           # Interactive mode
```

### As Library
```python
from bridge_client import KMBoxBridge

with KMBoxBridge() as kmbox:
    kmbox.move(100, 50)      # Relative mouse movement
    kmbox.click('left')       # Left click
    kmbox.key_press('a')      # Press 'a' key
```

### Direct Serial Access
The bridge provides transparent KMBox passthrough:

```bash
# macOS: screen /dev/cu.usbmodem* 115200
# Linux: screen /dev/ttyACM0 115200
```

Then type KMBox commands directly:
```
km.move(10,20)
km.click(1)
```

## Performance & Troubleshooting

**Performance:**
- Baud rate: 115200 bps (~200 KB/s)
- Latency: <1ms typical through bridge
- Throughput: ~11,500 commands/second theoretical

**Common Issues:**
- Bridge not detected: Check USB cable, try different port, re-flash firmware
- No KMBox response: Verify wiring (TX→RX, GND→GND) and matching baud rates
- Garbled data: Check GND connection, try shorter wires
