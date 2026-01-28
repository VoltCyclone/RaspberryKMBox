# MCP2210 KMBox Client

High-performance Python client for communicating with KMBox via the MCP2210 USB-SPI Click board.

## Why MCP2210/SPI?

The MCP2210 is a **HID-to-SPI bridge**, providing direct SPI communication over USB:

| Feature | MCP2210 (SPI) | Standard CDC/ACM |
|---------|---------------|------------------|
| **Max Speed** | 12 Mbps | 1 Mbps typical |
| **Latency** | Lower, predictable | Higher, variable |
| **Protocol** | Synchronous (full-duplex) | Async serial |
| **OS Overhead** | Minimal (HID) | CDC stack overhead |

## Installation

```bash
pip install hidapi
```

### Linux udev Rules

On Linux, you may need udev rules for non-root HID access:

```bash
sudo tee /etc/udev/rules.d/99-mcp2210.rules << 'EOF'
# Microchip MCP2210 USB-to-SPI Bridge
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="00de", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="00de", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Usage

### As a Library

```python
from mcp2210_kmbox import MCP2210KMBox

# Using context manager (recommended)
with MCP2210KMBox() as kmbox:
    kmbox.move(100, 50)      # Relative mouse move
    kmbox.click('left')       # Left click
    kmbox.scroll(-3)          # Scroll down
    kmbox.key_press('a')      # Press 'a' key

# Manual connection management
kmbox = MCP2210KMBox()
kmbox.connect(bit_rate=1000000)  # 1 MHz SPI
kmbox.move(10, 20)
kmbox.disconnect()
```

### High-Performance Smooth Movement

```python
with MCP2210KMBox() as kmbox:
    # Smooth movement using SPI's low latency
    kmbox.move_smooth(100, 50, steps=20, delay_ms=1)
```

### Command Line

```bash
# List connected MCP2210 devices
python mcp2210_kmbox.py --list

# Test connection
python mcp2210_kmbox.py --test

# Interactive mode
python mcp2210_kmbox.py --interactive
```

## API Reference

### Mouse Commands

| Method | Description |
|--------|-------------|
| `move(x, y)` | Relative mouse movement |
| `click(button, count)` | Click button ('left', 'right', 'middle') |
| `button_down(button)` | Press and hold button |
| `button_up()` | Release button |
| `scroll(delta)` | Scroll wheel (+up, -down) |
| `move_smooth(x, y, steps, delay_ms)` | Interpolated smooth movement |

### Keyboard Commands

| Method | Description |
|--------|-------------|
| `key_press(key)` | Press and release key |
| `key_combo(keys, modifiers)` | Press key combination |

### MCP2210 Features

| Method | Description |
|--------|-------------|
| `get_chip_status()` | Get SPI bus and owner status |
| `get_gpio()` | Read 9 GPIO pin states |
| `set_gpio(value, mask)` | Set GPIO output values |
| `spi_transfer(data)` | Raw SPI transfer |
| `ping()` | Measure round-trip latency |

## Hardware Setup

```
Control PC  ──[USB]──>  MCP2210 (USB-SPI Click)
                              │
                              │ SPI (1-12 MHz)
                              │ SCK ─────> SCK
                              │ MOSI ────> MOSI (SDI on RP2040)
                              │ MISO <─── MISO (SDO on RP2040)
                              │ CS ──────> CS
                              ▼
                        Feather Click Shield
                              │
                              │ mikroBUS Socket 1 or 2
                              │
                              ▼
                        Feather RP2040 USB Host
                              │
                              │ PIO USB Host
                              ▼
                        Target Mouse/Keyboard
```

### mikroBUS Pin Mapping (USB-SPI Click)

| mikroBUS Pin | Signal | Direction | RP2040 Pin |
|--------------|--------|-----------|------------|
| 3 (CS) | GP0/CS | Out | GPIO (CS) |
| 4 (SCK) | SCK | Out | SPI SCK |
| 5 (MISO) | SDO | In | SPI RX |
| 6 (MOSI) | SDI | Out | SPI TX |

## SPI Protocol

The MCP2210 is the SPI **Master** - it controls the clock and chip select.
The RP2040 runs as SPI **Slave** - it responds to the master's commands.

### Packet Format

Same 8-byte binary protocol as UART version:

| Byte | Field | Description |
|------|-------|-------------|
| 0 | Command | FastCmd type (0x01-0xFF) |
| 1-7 | Payload | Command-specific data |

### SPI Mode

- **Mode 0** (CPOL=0, CPHA=0) - Clock idle low, data sampled on rising edge
- Default bit rate: 1 MHz (configurable up to 12 MHz)

## Troubleshooting

### "No MCP2210 devices found"

1. Check USB cable connection
2. Verify USB-SPI Click power LED
3. On Linux, ensure udev rules are installed
4. Run `python mcp2210_kmbox.py --list`

### "SPI busy" or "SPI not owner"

The MCP2210 may have an active SPI transfer. Try:
1. Disconnect and reconnect
2. Power cycle the USB-SPI Click

### No response to ping

The RP2040 firmware needs SPI slave support. Make sure:
1. Firmware has SPI slave handler
2. SPI pins are correctly connected
3. CS is properly driven low during transfers

### Latency Issues

- Increase SPI bit rate: `kmbox.connect(bit_rate=4000000)` for 4 MHz
- Use `move_smooth()` for interpolated movement
- Check for USB hub latency - connect directly to PC
