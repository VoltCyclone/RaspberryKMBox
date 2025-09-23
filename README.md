# PIOKMbox — USB HID passthrough for Raspberry Pi Pico

A dual-role USB HID firmware for Raspberry Pi Pico boards that forwards mouse and keyboard input from a USB device (host port) to a PC (device port) and accepts KMBox-compatible serial commands for precise control and automation.

## Highlights

- Dual USB roles: native TinyUSB device + PIO-USB host running concurrently
- Dynamic identity mirroring: adopts VID/PID and string descriptors (manufacturer, product, serial) from the attached HID device
- Full HID passthrough: mouse and keyboard, including side buttons and scroll wheel
- KMBox serial protocol: inject movement, clicks, timed actions, and axis locks over UART
- Dual-core design: Core 0 handles device + main loop, Core 1 runs the USB host stack
- Watchdog: software and hardware watchdog with inter-core heartbeats
- Visual status: onboard LED plus WS2812 NeoPixel with rich status feedback
- Button control: long-press to reset stacks (debounced, with cooldown)

## Hardware

### Supported boards

- Default target: Adafruit Feather RP2040 USB Host (configurable)
- RP2040 (Pico) fully supported; RP2350 (Pico 2) is possible with appropriate board support and SDK, but is less tested

You can change pins and board type via macros in `defines.h` or CMake cache options.

### Pinout (defaults)

- USB host DP/DM: GPIO 16 / 17
- USB host 5V enable: GPIO 18
- Status LED: GPIO 13
- Reset button: GPIO 7 (active-low)
- NeoPixel data: GPIO 21
- NeoPixel power: GPIO 20
- KMBox UART (UART1): TX=GPIO 5, RX=GPIO 6 @ 115200
- Debug UART (UART0): TX=GPIO 0, RX=GPIO 1 @ 115200

## Build

### Prerequisites

1. Raspberry Pi Pico SDK 2.2+ (set PICO_SDK_PATH)
2. CMake 3.13+
3. Arm GNU toolchain (arm-none-eabi-gcc, etc.)
4. Optional: Ninja
5. Optional: picotool (for quick flashing)

### Quick start

```bash
./build.sh
```

This script creates a fresh `build/` directory, configures CMake, builds the project, and, if `picotool` is available, attempts to load `PIOKMbox.uf2` automatically.

### Manual build

```bash
mkdir build
cd build
cmake ..
make -j4   # or: ninja
```

### Build outputs

- `PIOKMbox.uf2` — UF2 firmware for drag-and-drop flashing
- `PIOKMbox.elf` — ELF for debugging
- `PIOKMbox.bin`, `PIOKMbox.hex` — alternative images

### Flashing

- Automatic (if picotool is installed): run `./build.sh`
- Manual UF2: hold BOOTSEL while plugging USB, mount RPI-RP2, copy `PIOKMbox.uf2`

Note: To target a different board, set `PICO_BOARD` via CMake cache or edit `CMakeLists.txt` (default is `adafruit_feather_rp2040_usb_host`).

## How it works

1. Core 1 runs TinyUSB host on PIO-USB to talk to your keyboard/mouse.
2. The firmware reads the HID report descriptor and caches VID/PID and strings from the attached device.
3. Core 0 exposes a TinyUSB HID device to the PC and mirrors the attached device’s identity and report behavior.
4. When VID/PID changes, the device disconnects and re-enumerates to reflect the new identity. String descriptors are mirrored when available.
5. Physical HID input and KMBox serial commands are combined so scripted actions and real input can coexist (with axis locks and timing).

Fallbacks: If no device is attached or descriptors aren’t available, defaults are used — VID:PID `0x9981:0x4001`, Manufacturer `"Hurricane"`, Product `"PIOKM Box"`, and a serial derived from the Pico’s unique ID.

## Using KMBox serial

- Port: UART1 (GPIO 5/6), 115200 8N1
- Capabilities: movement injection, button press/release, timed clicks, wheel, axis locks

Examples:

```text
km.move(100, 50)
km.left(1)       # press
km.left(0)       # release
km.click(0, 100) # left-click for 100 ms
km.lock.mx(1)    # lock X axis
km.lock.my(1)    # lock Y axis
```

## Status indicators

### LED (GPIO 13)

- Fast blink (~250 ms): device mounted/resumed
- Medium blink (~1000 ms): device unmounted
- Slow blink (~2500 ms): device suspended

### NeoPixel (GPIO 21)

- Blue: booting
- Green: USB device only
- Orange: USB host only
- Cyan: both stacks active
- Magenta: mouse connected
- Yellow: keyboard connected
- Pink: mouse and keyboard connected
- Red: error
- Purple: suspended

## Button behavior

- Long press (≈3 s): reset USB stacks (with cooldown to avoid loops)
- Short press: reserved

## Development notes

### Project layout

```text
PIOKMbox/
├── PIOKMbox.c               # Main firmware (core orchestration)
├── usb_hid.*                # HID device/host, VID/PID & string mirroring
├── led_control.*            # LED & WS2812 control
├── watchdog.*               # HW/SW watchdog + inter-core heartbeats
├── init_state_machine.*     # Startup/initialization sequencing
├── state_management.*       # Shared system state
├── kmbox_serial_handler.*   # KMBox UART integration
├── ws2812.pio               # PIO program for NeoPixel
├── defines.h, timing_config.h, config.h
├── lib/
│   ├── Pico-PIO-USB/        # PIO USB library
│   └── kmbox-commands/      # KMBox command parser
└── CMakeLists.txt, build.sh
```

### Configuration

- Build configuration presets (see `defines.h`):
    - `BUILD_CONFIG_DEVELOPMENT` (default)
    - `BUILD_CONFIG_PRODUCTION`
    - `BUILD_CONFIG_TESTING`
    - `BUILD_CONFIG_DEBUG`
- Pins, LED timings, watchdog intervals, and colors are centralized in `defines.h`.
- CPU clock and USB timing are tuned for PIO-USB (RP2040 default 240 MHz).

## Troubleshooting

1. Ensure Pico SDK is installed and `PICO_SDK_PATH` is set.
2. For USB host, verify 5V power and DP/DM wiring to GPIO 16/17.
3. If passthrough seems inactive, open the debug UART (GPIO 0/1 @ 115200) for logs.
4. If re-enumeration loops occur, check that the attached device is stable; the firmware only re-enumerates when VID/PID changes.
5. Some devices present 16-bit mouse deltas; these are scaled to 8-bit—adjust sensitivity in code if needed.

## Contributing

PRs and issues are welcome. Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test on hardware
5. Open a pull request with details

## License

Libraries under `lib/` retain their own licenses. If you need clarification, please open an issue.
