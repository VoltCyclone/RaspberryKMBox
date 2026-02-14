# FPGA Bridge Wiring & Pin Reference

Complete pin assignments for the pico2-ice FPGA bridge board and its
connections to the KMBox (Pico 2).

## System Overview

```
                          +-----------+
                          |  PC/Host  |
                          +-----+-----+
                                | USB CDC
                                |
+-------------------------------v-------------------------------+
|                       pico2-ice Board                         |
|  RP2350B (48 GPIO) + iCE40 UP5K FPGA                         |
|                                                               |
|  Internal SPI (RP2350 ↔ FPGA via on-board CRAM traces)        |
|  PMOD A top row (FPGA ↔ KMBox SPI, active after bitstream)   |
|  RP2350 GPIO 20/25 (direct UART to KMBox, bypasses FPGA)      |
+---+---+-------------------------------------------------------+
    |   |
    |   | UART 921600 baud (2 wires)
    |   |
    | SPI 12 MHz (4 wires)
    |   |
+---v---v-------------------------------------------------------+
|                    KMBox (Pico 2 or Metro RP2350)              |
|  SPI Slave on PIO2 (Pico: GPIO 2-5, Metro: GPIO 10-11/22-23) |
|  Side Channel UART1 (GPIO 8-9)                                |
|  USB Host PIO (Pico: GPIO 16-17, Metro: GPIO 32-33)          |
+----------------------------------------------------------------+
```

## External Wiring (board-to-board)

Only 6 signal wires + ground are needed between the two boards.

### 1. SPI Command Path — PMOD A → KMBox

High-speed 8-byte command relay. The FPGA generates the SPI clock.

**Pico 2** (`./build.sh pico2`) — uses GPIO 2-5:

| PMOD A pin | iCE40 pin | Signal | Dir | KMBox GPIO | KMBox function    |
|------------|-----------|--------|-----|------------|-------------------|
| Top IO2    | 2         | SCK    | →   | GPIO 3     | PIO2 SCK          |
| Top IO1    | 4         | MOSI   | →   | GPIO 2     | PIO2 MOSI         |
| Top IO4    | 47        | CS_N   | →   | GPIO 4     | GPIO input (idle) |
| Top IO3    | 45        | MISO   | ←   | GPIO 5     | PIO2 MISO (opt.)  |
| GND        | —         | GND    | —   | GND        | Common ground     |

**Metro RP2350** (`./build.sh metro-broken`) — uses D10-D11, D22-D23:

| PMOD A pin | iCE40 pin | Signal | Dir | KMBox GPIO | KMBox function    |
|------------|-----------|--------|-----|------------|-------------------|
| Top IO2    | 2         | SCK    | →   | GPIO 11    | PIO2 SCK (D11)    |
| Top IO1    | 4         | MOSI   | →   | GPIO 10    | PIO2 MOSI (D10)   |
| Top IO4    | 47        | CS_N   | →   | GPIO 22    | GPIO input (D22)  |
| Top IO3    | 45        | MISO   | ←   | GPIO 23    | PIO2 MISO (D23)   |
| GND        | —         | GND    | —   | GND        | Common ground     |

- **Protocol:** SPI Mode 0, 12 MHz, 64-bit (8-byte) fixed transactions
- **Latency:** ~5.3 µs per packet (260 clocks at 48 MHz)
- **PIO constraint:** SCK pin must be MOSI pin + 1 (adjacent GPIOs)

### 2. Side Channel UART — RP2350 → KMBox (direct, bypasses FPGA)

Low-bandwidth status and control link.

| Bridge RP2350 | Function   | Dir | KMBox GPIO | KMBox function |
|---------------|------------|-----|------------|----------------|
| GPIO 20       | UART0 TX   | →   | GPIO 9     | UART1 RX       |
| GPIO 25       | UART0 RX   | ←   | GPIO 8     | UART1 TX       |
| GND           | GND        | —   | GND        | Common ground  |

- **Protocol:** 921600 baud, framed packets `[0xCC][CMD][LEN][data…][CHK]`
- **Carries:** humanization mode, temperature, USB descriptor strings, VID/PID
- **Polling:** status every 500 ms; descriptor strings fetched once on connect

### 3. Common Ground

Both boards must share a common ground. If powered from separate USB ports,
connect at least one GND wire between them.

### Summary

```
Total signal wires:  6  (4 SPI + 2 UART)
Total ground wires:  1+ (shared GND)
```

---

## On-Board Connections (no external wiring needed)

These use traces on the pico2-ice PCB itself.

### 4. RP2350 ↔ FPGA Internal SPI (CRAM traces, reused after bitstream load)

After the RP2350 loads the iCE40 bitstream via CRAM (SPI0), it releases the
pins and reconfigures them as a PIO2 SPI master to talk to the FPGA engine.

| RP2350 GPIO | Function       | Dir | iCE40 pin | FPGA signal |
|-------------|----------------|-----|-----------|-------------|
| GPIO 7      | PIO2 SPI MOSI  | →   | 14        | SPI_MOSI    |
| GPIO 6      | PIO2 SPI SCK   | →   | 15        | SPI_SCK     |
| GPIO 5      | PIO2 SPI CS_N  | →   | 16        | SPI_CS_N    |
| GPIO 4      | PIO2 SPI MISO  | ←   | 17        | SPI_MISO    |

- **Clock:** 12 MHz (PIO-generated), 48 MHz system clock on FPGA
- **48 MHz reference:** RP2350 GPOUT0 on GPIO 21 → iCE40 pin 35

### 5. Status LEDs (three separate indicators)

The pico2-ice board has **three independent LED indicators**:

#### Pico RGB LED (RP2350-driven, GPIO 0/1/9)

| RP2350 GPIO | Color | Meaning                              |
|-------------|-------|--------------------------------------|
| 0           | Green | KMBox connected (solid)              |
| 1           | Red   | KMBox disconnected (blinks 4 Hz)     |
| 9           | Blue  | USB CDC connected (PC link active)   |

#### FPGA RGB LED (iCE40-driven, autonomous)

| iCE40 pin | Color | Meaning                              |
|-----------|-------|--------------------------------------|
| 39        | Green | KMBox SPI connected (solid)          |
| 41        | Red   | KMBox SPI disconnected (blinks 1 Hz) |
| 40        | Blue  | SPI activity (brief flash)           |

#### FPGA DONE LED (hardware, GPIO 40)

| RP2350 GPIO | Meaning                                  |
|-------------|------------------------------------------|
| 40 (CDONE)  | FPGA bitstream loaded and running         |

#### LED behavior at a glance

| Pico LED             | FPGA LED               | DONE | Meaning                          |
|----------------------|------------------------|------|----------------------------------|
| Green                | Green                  | On   | System fully healthy             |
| Green + Blue         | Green + Blue flash     | On   | Healthy + PC connected + active  |
| Green + Red          | Red blink              | On   | KMBox disconnected               |
| Fast red blink       | Off                    | Off  | FPGA failed to load              |
| All off              | All off                | Off  | No power or reset                |

---

## Pin Reference Tables

### pico2-ice RP2350B — All Used GPIOs

| GPIO | Peripheral | Function              | Connected to              |
|------|------------|-----------------------|---------------------------|
| 0    | GPIO (LED) | Pico RGB green        | Pico RGB LED (green)      |
| 1    | GPIO (LED) | Pico RGB red          | Pico RGB LED (red)        |
| 4    | PIO2 MISO  | FPGA SPI data in      | iCE40 pin 17 (on-board)  |
| 5    | PIO2 CS_N  | FPGA SPI chip select  | iCE40 pin 16 (on-board)  |
| 6    | PIO2 SCK   | FPGA SPI clock        | iCE40 pin 15 (on-board)  |
| 7    | PIO2 MOSI  | FPGA SPI data out     | iCE40 pin 14 (on-board)  |
| 9    | GPIO (LED) | Pico RGB blue         | Pico RGB LED (blue)       |
| 20   | UART0 TX   | Side channel TX       | KMBox GPIO 9 (UART1 RX)  |
| 21   | GPOUT0     | 48 MHz clock to FPGA  | iCE40 pin 35 (on-board)  |
| 25   | UART0 RX   | Side channel RX       | KMBox GPIO 8 (UART1 TX)  |
| 31   | GPIO       | FPGA CRESET           | iCE40 CRESET (on-board)  |
| 40   | GPIO (in)  | FPGA CDONE            | FPGA DONE LED (hardware) |

### KMBox (Pico 2) — Bridge-Related GPIOs

Build with `./build.sh pico2`:

| GPIO | Function              | Connected to                       |
|------|-----------------------|------------------------------------|
| 0    | UART0 TX              | PC serial (3 Mbaud)               |
| 1    | UART0 RX              | PC serial (3 Mbaud)               |
| 2    | PIO2 SPI MOSI (in)   | PMOD A Top IO1 / iCE40 pin 4      |
| 3    | PIO2 SPI SCK (in)    | PMOD A Top IO2 / iCE40 pin 2      |
| 4    | GPIO CS_N (in)       | PMOD A Top IO4 / iCE40 pin 47     |
| 5    | PIO2 SPI MISO (out)  | PMOD A Top IO3 / iCE40 pin 45     |
| 7    | Button                | Mode cycle (active low)            |
| 8    | UART1 TX             | Bridge RP2350 GPIO 25 (UART0 RX)  |
| 9    | UART1 RX             | Bridge RP2350 GPIO 20 (UART0 TX)  |
| 16   | USB Host D+          | Physical mouse/keyboard            |
| 17   | USB Host D−          | Physical mouse/keyboard            |

### KMBox (Metro RP2350) — Bridge-Related GPIOs

Build with `./build.sh metro-broken` (D0-D7 broken, D12-D19 are HSTX).
Available: D8-D11, D22-D23, A0-A5 (GPIO 41-46).

| GPIO | Header | Function              | Connected to                       |
|------|--------|-----------------------|------------------------------------|
| 8    | D8     | UART1 TX             | Bridge RP2350 GPIO 25 (UART0 RX)  |
| 9    | D9     | UART1 RX             | Bridge RP2350 GPIO 20 (UART0 TX)  |
| 10   | D10    | PIO2 SPI MOSI (in)  | PMOD A Top IO1 / iCE40 pin 4      |
| 11   | D11    | PIO2 SPI SCK (in)   | PMOD A Top IO2 / iCE40 pin 2      |
| 22   | D22    | GPIO CS_N (in)       | PMOD A Top IO4 / iCE40 pin 47     |
| 23   | D23    | PIO2 SPI MISO (out) | PMOD A Top IO3 / iCE40 pin 45     |
| 7    | —      | Button               | Mode cycle (active low, on-board)  |
| 23   | —      | Status LED           | Red LED (active-high)              |
| 25   | —      | NeoPixel             | WS2812B                            |
| 32   | —      | USB Host D+          | Physical mouse/keyboard            |
| 33   | —      | USB Host D−          | Physical mouse/keyboard            |
