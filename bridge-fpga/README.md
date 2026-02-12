# KMBox FPGA Bridge (pico2-ice)

Hardware-accelerated KMBox bridge using the **pico2-ice** board (RP2350B + iCE40 UP5K).

The iCE40 FPGA handles all KMBox UART communication at 3 Mbaud, freeing the RP2350 entirely for USB CDC, color tracking, and display work.

## Architecture

```
PC ←(USB CDC)→ RP2350 ←(SPI regs)→ iCE40 FPGA ←(UART 3M)→ KMBox
                 │
                 └── TFT display (future)
```

### FPGA Responsibilities
- **UART TX/RX** at 3 Mbaud (48 MHz ÷ 16 = exact, 0 ppm error)
- **Bridge protocol** packet assembly (SYNC + CMD + payload)
- **Auto-ping** keepalive (2-second interval)
- **RX parser** for 8-byte binary response packets
- **64-byte RX FIFO** for response buffering
- **Connection state** tracking (5-second timeout)

### RP2350 Responsibilities
- **USB CDC** interface for PC client communication
- **FPGA CRAM** bitstream loading on every boot
- **SPI register** interface to FPGA command/status registers
- **Color tracking** (fixed-point centroid computation)
- **DFU** interface for over-USB FPGA bitstream updates

## Hardware

| Signal | pico2-ice Pin | iCE40 Pin | Function |
|--------|--------------|-----------|----------|
| SPI SCK | GPIO 6 | 15 | SPI clock (10 MHz) |
| SPI MOSI | GPIO 7 (ICE_SO) | 14 | RP2350 → FPGA |
| SPI MISO | GPIO 4 (ICE_SI) | 17 | FPGA → RP2350 |
| SPI CS | GPIO 5 | 16 | Chip select (active low) |
| FPGA CLK | GPIO 21 | 35 | 48 MHz clock from RP2350 |
| UART TX | — | 2 (PMOD A) | FPGA → KMBox |
| UART RX | — | 4 (PMOD A) | KMBox → FPGA |
| CDONE | GPIO 40 | — | Configuration done |
| CRESET | GPIO 31 | — | Configuration reset |

### KMBox Connection (PMOD A)
Connect the KMBox's UART lines to the pico2-ice PMOD A header:
- **PMOD A Pin 1** (iCE40 GPIO 2) → KMBox RX
- **PMOD A Pin 2** (iCE40 GPIO 4) ← KMBox TX
- **PMOD A GND** ↔ KMBox GND

## SPI Register Map

| Address | Name | R/W | Description |
|---------|------|-----|-------------|
| 0x00 | VERSION | R | Bridge engine version (0x01) |
| 0x01 | STATUS | R | Bit flags: connected, tx_busy, rx_avail, error |
| 0x02 | TX_COUNT_L | R | TX packet counter (low byte) |
| 0x03 | TX_COUNT_H | R | TX packet counter (high byte) |
| 0x04 | RX_COUNT_L | R | RX packet counter (low byte) |
| 0x05 | RX_COUNT_H | R | RX packet counter (high byte) |
| 0x06 | PING_COUNT_L | R | Auto-ping counter (low byte) |
| 0x07 | PING_COUNT_H | R | Auto-ping counter (high byte) |
| 0x08 | RX_FIFO_LEVEL | R | Bytes available in RX FIFO |
| 0x09 | CONN_TIMER_L | R | Connection timer (low byte) |
| 0x0A | CONN_TIMER_H | R | Connection timer (high byte) |
| 0x10 | CMD_TYPE | W | Command type (triggers TX) |
| 0x11 | CMD_DX_L | W | Mouse delta X (low) |
| 0x12 | CMD_DX_H | W | Mouse delta X (high) |
| 0x13 | CMD_DY_L | W | Mouse delta Y (low) |
| 0x14 | CMD_DY_H | W | Mouse delta Y (high) |
| 0x15 | CMD_WHEEL | W | Mouse wheel delta |
| 0x16 | CMD_BUTTONS_L | W | Button mask (low) |
| 0x17 | CMD_BUTTONS_H | W | Button mask (high) |
| 0x20 | RX_FIFO_DATA | R | Read one byte from RX FIFO |
| 0x21 | RX_FIFO_RESET | W | Reset/flush RX FIFO |

## Prerequisites

### FPGA Toolchain (OSS)
```bash
# macOS
brew install yosys nextpnr icestorm

# Linux (Debian/Ubuntu)
sudo apt install yosys nextpnr-ice40 fpga-icestorm
```

### Pico SDK
The build expects the Pico SDK at `~/.pico-sdk/sdk/2.2.0-fresh/` (VS Code Pico extension default).

### pico-ice-sdk
Clone the [pico-ice-sdk](https://github.com/tinyvision-ai-inc/pico-ice-sdk) as a sibling of the RaspberryKMBox repo:
```
code/
  ├── RaspberryKMBox/
  │   └── bridge-fpga/   ← this project
  └── pico-ice-sdk/       ← SDK here
```

## Building

```bash
# From the RaspberryKMBox root:
./build.sh bridge-fpga

# Or with clean rebuild:
./build.sh bridge-fpga clean

# Build and flash:
./build.sh bridge-fpga flash
```

The build automatically:
1. Synthesizes Verilog RTL → iCE40 bitstream (yosys → nextpnr → icepack)
2. Converts bitstream to C header (`bitstream.h`)
3. Compiles RP2350 firmware with embedded bitstream
4. Produces `bridge-fpga/build/kmbox_fpga_bridge.uf2`

## RTL Structure

```
rtl/
├── top.v              # Top-level wiring + power-on reset + LED status
├── bridge_engine.v    # Protocol engine: TX FSM, RX parser, auto-ping, status regs
├── spi_target.v       # SPI Mode 0 slave with register bus
├── uart_tx.v          # 8N1 UART transmitter (3 Mbaud)
├── uart_rx.v          # 8N1 UART receiver with 2-FF sync
├── sync_fifo.v        # Parameterized synchronous FIFO
├── bridge_regs.vh     # Register map definitions (shared with C header)
└── pico2_ice_bridge.pcf  # Pin constraints for pico2-ice
```

## LED Status

The pico2-ice RGB LED indicates bridge state:

| LED | Meaning |
|-----|---------|
| 🟢 Solid green | Connected to KMBox |
| 🔴 Blinking red | Disconnected / waiting |
| 🔵 Flash blue | SPI register activity |

## Protocol

Uses the same bridge protocol as the standard KMBox bridge:

| Command | ID | Payload |
|---------|-----|---------|
| MOUSE_MOVE | 0x01 | dx(16), dy(16) |
| MOUSE_WHEEL | 0x02 | delta(8) |
| BUTTON_SET | 0x03 | mask(16) |
| MOUSE_MOVE_WHEEL | 0x04 | dx(16), dy(16), wheel(8) |
| PING | 0xFE | — |
| RESET | 0xFF | — |

All packets start with sync byte `0xBD`, followed by the command byte and payload (little-endian).
