# KMBox Bridge - Advanced Autopilot Firmware

Advanced autopilot firmware running on an **Adafruit Feather RP2350** with color-based target tracking, PIO UART communication, and LED status indicators.

## Features

- **USB CDC Interface**: Receives RGB frame data from PC capture tool
- **Color Tracking**: Hardware-accelerated blob detection and centroid calculation
- **PIO UART TX**: Sends mouse movement commands to RP2040 KMBox at 115200 baud
- **Status LEDs**:
  - **Onboard LED (GPIO25)**: Heartbeat (500ms blink)
  - **NeoPixel (GPIO22)**:
    - RED = Idle, waiting for frames
    - GREEN = Tracking active
    - YELLOW = Tracking disabled
    - BLUE = Brief activity flash

## Architecture

```
┌─────────────┐   USB CDC    ┌──────────────────┐   PIO UART   ┌───────────────┐
│   PC Tool   │  (RGB frames) │ Feather RP2350   │  (115200)    │ KMBox Pico    │
│  (capture)  │◄────────────►│   (Tracker)      │────────────►│ (USB Host)    │
└─────────────┘               └──────────────────┘              └───────────────┘
                                      │                               │
                                      │ GPIO0 (TX) ────►  RX         │
                                      │ GND        ────   GND        │
                                      └──────────────────────────────┘
```

## Wiring

Connect the Feather RP2350 to the Feather RP2040 USB Host:

| Feather RP2350 Pin | Direction | KMBox Pico Pin | Description |
|--------------------|-----------|----------------|-------------|
| GPIO0 (TX)         | ────►     | RX pin         | Bridge transmits commands |
| GND                | ────      | GND            | Common ground |

**Note:** This is one-way communication (bridge → KMBox only). No RX needed on the bridge.

## Protocol

### PC → Bridge (USB CDC)
- Frame header: `['F', 'R', width_low, width_high, height_low, height_high, 0x00, 0x00]`
- Frame data: RGB888 pixels (width × height × 3 bytes)
- Config commands: `['C', r_min, r_max, g_max, b_max, gain_x*10, gain_y*10, deadzone, min_blob]`

### Bridge → KMBox (PIO UART @ 115200 baud)
- Mouse move: `[0x01, dx_low, dx_high, dy_low, dy_high, buttons]`

## Building

```bash
cd bridge
./build.sh
```

The firmware will be at `build/kmbox_bridge.uf2`.

## Flashing

1. Hold the **BOOTSEL** button on the Feather RP2350
2. Connect USB to your computer
3. Release BOOTSEL - a drive named "RPI-RP2" will appear
4. Copy `kmbox_bridge.uf2` to the drive
5. The Feather will reboot and appear as a serial port

## Usage

### Python Client

Install pyserial:
```bash
pip install pyserial
```

List available ports:
```bash
python bridge_client.py --list
```

Test connection:
```bash
python bridge_client.py --test
```

Interactive mode:
```bash
python bridge_client.py
```

### As a Library

```python
from bridge_client import KMBoxBridge

with KMBoxBridge() as kmbox:
    kmbox.move(100, 50)      # Relative mouse movement
    kmbox.click('left')       # Left click
    kmbox.key_press('a')      # Press 'a' key
```

### Direct Serial Access

The bridge is a transparent pass-through. You can use any serial terminal:

```bash
# macOS
screen /dev/cu.usbmodem* 921600

# Linux  
screen /dev/ttyACM0 921600

# Or use minicom, picocom, etc.
```

Then type KMBox commands directly:
```
km.move(10,20)
km.click(1)
```

## Performance

- **Baud rate:** 921600 bps (~92 KB/s)
- **Latency:** <1ms typical through bridge
- **Throughput:** ~11,500 commands/second theoretical max

The UART speed is more than sufficient for mouse/keyboard input rates (typically <1000 Hz).

## LED Behavior

- **Blinking:** Data is being transferred
- **Off:** Idle (no data flowing)

## Troubleshooting

### Bridge not showing up as serial port

1. Check USB cable (some cables are charge-only)
2. Try a different USB port
3. Re-flash the firmware

### No response from KMBox

1. Check wiring (TX to RX, RX to TX, GND to GND)
2. Verify KMBox firmware is running
3. Check baud rate matches (921600)

### Garbled data

1. Check GND connection
2. Verify both devices use the same baud rate
3. Try shorter wires if using long jumpers
