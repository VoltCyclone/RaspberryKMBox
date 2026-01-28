# CP2110 KMBox Client

High-performance Python client for communicating with KMBox via the CP2110 USB UART5 Click board.

## Why CP2110?

The CP2110 is a **HID-to-UART bridge**, not a standard CDC/ACM serial device. This provides:

| Feature | CP2110 (HID) | Standard CDC/ACM |
|---------|--------------|------------------|
| **Polling Rate** | 1ms (1000 Hz) | 1-10ms typical |
| **Latency** | Lower, predictable | Higher, variable |
| **Max Baud Rate** | 1 Mbps | Varies by chip |
| **OS Overhead** | Minimal | CDC stack overhead |

## Installation

```bash
pip install hidapi
```

### Linux udev Rules

On Linux, you may need udev rules for non-root HID access:

```bash
sudo tee /etc/udev/rules.d/99-cp2110.rules << 'EOF'
# Silicon Labs CP2110 HID-to-UART Bridge
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea80", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea80", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Usage

### As a Library

```python
from cp2110_kmbox import CP2110KMBox

# Using context manager (recommended)
with CP2110KMBox() as kmbox:
    kmbox.move(100, 50)      # Relative mouse move
    kmbox.click('left')       # Left click
    kmbox.scroll(-3)          # Scroll down
    kmbox.key_press('a')      # Press 'a' key

# Manual connection management
kmbox = CP2110KMBox()
kmbox.connect(baud_rate=921600)
kmbox.move(10, 20)
kmbox.disconnect()
```

### High-Performance Smooth Movement

```python
with CP2110KMBox() as kmbox:
    # Smooth movement using CP2110's 1ms polling rate
    kmbox.move_smooth(100, 50, steps=20, delay_ms=1)
```

### Command Line

```bash
# List connected CP2110 devices
python cp2110_kmbox.py --list

# Test connection
python cp2110_kmbox.py --test

# Interactive mode
python cp2110_kmbox.py --interactive
```

## API Reference

### Mouse Commands

| Method | Description |
|--------|-------------|
| `move(x, y)` | Relative mouse movement |
| `move_to(x, y)` | Absolute mouse position (if supported) |
| `click(button, count)` | Click button ('left', 'right', 'middle') |
| `button_down(button)` | Press and hold button |
| `button_up(button)` | Release button |
| `scroll(delta)` | Scroll wheel (+up, -down) |
| `move_smooth(x, y, steps, delay_ms)` | Interpolated smooth movement |

### Keyboard Commands

| Method | Description |
|--------|-------------|
| `key_press(key)` | Press and release key |
| `key_down(key)` | Press and hold key |
| `key_up(key)` | Release key |

### Low-Level

| Method | Description |
|--------|-------------|
| `send_command(cmd, wait_response)` | Send raw KMBox command |
| `write(data)` | Write raw bytes to UART |
| `read(timeout_ms)` | Read raw bytes from UART |

## Advanced CP2110 Features

The CP2110 is **much more than a simple UART bridge**. It provides direct hardware control from the PC via HID feature reports:

### GPIO Control (10 pins)

The CP2110 has 10 GPIO pins that can be read/written directly from the PC, completely independent of UART data:

```python
with CP2110KMBox() as kmbox:
    # Read all GPIO pins
    latch, gpio = kmbox.get_gpio()
    print(f"Latch (outputs): {latch:010b}")
    print(f"GPIO (inputs):   {gpio:010b}")
    
    # Set GPIO pin 0 high (e.g., external trigger)
    kmbox.set_gpio(latch_value=0x01, write_mask=0x01)
    
    # Set pin 0 high, pin 1 low simultaneously
    kmbox.set_gpio(latch_value=0b01, write_mask=0b11)
```

**Use cases:**
- Control external LEDs or indicators
- Read buttons or switches
- Trigger external devices (cameras, relays)
- Out-of-band signaling without using UART

### UART Status Monitoring

Real-time monitoring of UART buffers and error flags:

```python
status = kmbox.get_uart_status()
print(status)  # "TX:45/480 RX:128/480 Errors:none"

# Check before sending large data
if status.tx_fifo_bytes < 400:
    kmbox.write(large_data)
    
# Check for received data
if status.rx_fifo_bytes > 0:
    data = kmbox.read()
    
# Error detection
if status.parity_error or status.overrun_error:
    print("Communication error detected!")
    kmbox.purge_fifos()  # Reset for clean state
```

### FIFO Management

Clear TX/RX buffers instantly for clean state:

```python
# Clear both buffers
kmbox.purge_fifos()

# Clear only TX buffer
kmbox.purge_fifos(FifoPurge.TX)

# Clear only RX buffer (discard pending data)
kmbox.purge_fifos(FifoPurge.RX)
```

### Line Break Control

Send UART break signals (sustained low on TX line):

```python
# Send 100ms break signal (e.g., for bootloader entry)
kmbox.send_line_break(100)

# Longer break for some protocols
kmbox.send_line_break(250)
```

### Dynamic UART Configuration

Change UART settings at runtime:

```python
from cp2110_kmbox import UARTConfig, Parity, FlowControl, DataBits, StopBits

# Get current config
config = kmbox.get_uart_config()
print(config)  # "115200 8N1"

# Configure for higher speed with flow control
new_config = UARTConfig(
    baud_rate=921600,
    data_bits=DataBits.EIGHT,
    parity=Parity.NONE,
    stop_bits=StopBits.ONE,
    flow_control=FlowControl.HARDWARE  # Enable CTS/RTS
)
kmbox.set_uart_config(new_config)
```

### Device Information

```python
# Get CP2110 chip info
version = kmbox.get_version()
print(version)  # "CP2110 v16"

# Check UART state
if kmbox.is_uart_enabled():
    print("UART is active")
```

### Device Reset

```python
# Full hardware reset (USB reconnection required)
kmbox.reset_device()
time.sleep(2)  # Wait for USB re-enumeration
kmbox.connect()  # Reconnect
```

### Advanced Feature Summary

| Feature | Method | Description |
|---------|--------|-------------|
| **GPIO Read** | `get_gpio()` | Read all 10 GPIO pin states |
| **GPIO Write** | `set_gpio(value, mask)` | Write GPIO pins with mask |
| **UART Status** | `get_uart_status()` | TX/RX FIFO levels, error flags |
| **FIFO Purge** | `purge_fifos()` | Clear TX/RX buffers |
| **Line Break** | `send_line_break(ms)` | Send UART break signal |
| **UART Config** | `get/set_uart_config()` | Change baud, parity, etc. |
| **UART Enable** | `is_uart_enabled()` | Check if UART active |
| **Device Info** | `get_version()` | Get part number & version |
| **Reset** | `reset_device()` | Full hardware reset |

## Hardware Setup

```
Control PC  ──[USB]──>  CP2110 (USB UART5 Click)
                              │
                              │ UART (115200 8N1)
                              │
                              ▼
                        Feather Click Shield
                              │
                              │ mikroBUS Socket 1
                              │ TX → RX (GPIO1)
                              │ RX ← TX (GPIO0)
                              ▼
                        Feather RP2040 USB Host
                              │
                              │ PIO USB Host
                              ▼
                        Target Mouse/Keyboard
```

## Troubleshooting

### "No CP2110 devices found"

1. Check USB cable connection
2. Verify USB UART5 Click is powered (LED should be on)
3. On Linux, ensure udev rules are installed
4. On macOS, no special permissions needed
5. On Windows, Silicon Labs drivers should auto-install

### "Permission denied" on Linux

Install the udev rules above and re-plug the device.

### Latency Issues

- Ensure `set_nonblocking(1)` is called (done automatically)
- Use `move_smooth()` for interpolated movement
- Reduce `delay_ms` parameter for faster updates
