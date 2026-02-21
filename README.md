# PIOKMbox — USB HID Passthrough & KMBox Serial Interface

A high-performance dual-role USB HID firmware for RP2350 boards that creates a **transparent USB passthrough device** while providing **KMBox-compatible serial control** for mouse and keyboard automation with advanced humanization and optional visual feedback via ILI9341 display.

## ⚠️ Warning

This firmware runs at high clock speeds (240MHz+) for optimal performance. Monitor temperature if using extended automation sessions, especially with the TFT display enabled.

## What It Does

**USB Passthrough**: Transparently connects between your mouse/keyboard and PC, forwarding all input while mirroring the connected device's identity (VID/PID, manufacturer, product name).

**Serial Control**: Accepts KMBox, Macku binary, and Ferrum-compatible commands over UART to inject precise mouse movements, clicks, and keyboard input alongside your physical devices.

**Advanced Humanization**: Built-in anti-detection algorithms with movement-aware jitter, velocity suppression, overshoot simulation, and configurable intensity levels.

**Visual Feedback**: Optional ILI9341 TFT display shows real-time latency tracking, connection status, and system metrics.

## Key Features

- **Transparent USB Passthrough**: Your PC sees the original device, not the RP2350
- **KMBox Protocol Compatibility**: Works with existing KMBox B+, Ferrum, and Macku tools
- **Dual-Core Architecture**: Dedicated cores for USB host and device operations
- **Movement Humanization**: 4 configurable modes (OFF/LOW/MEDIUM/HIGH) with adaptive jitter
- **Hardware Watchdog**: Automatic recovery from USB stack failures
- **Visual Status**: NeoPixel RGB feedback and optional ILI9341 TFT display
- **Low Latency**: <50µs binary protocol, <100µs text commands

## Hardware Requirements

### Target Hardware Configuration

**Recommended Setup**: Dual Adafruit Metro RP2350 + ILI9341 Display

#### Board 1: USB Proxy (Main KMBox)
- **Board**: Adafruit Metro RP2350
- **Role**: USB HID passthrough + KMBox command execution
- **Connections**:
  - USB-A port → Mouse/keyboard
  - USB-C port → PC
  - GPIO TX → Board 2 RX (crossed)
  - GPIO RX → Board 2 TX (crossed)
  - GND → Board 2 GND

#### Board 2: Bridge/Autopilot (Optional)
- **Board**: Adafruit Metro RP2350
- **Role**: Computer vision tracking + ILI9341 display driver
- **Connections**:
  - USB-C port → PC (for serial input commands)
  - GPIO TX → Board 1 RX (crossed)
  - GPIO RX → Board 1 TX (crossed)
  - GND → Board 1 GND
  - SPI + control pins → ILI9341 display

#### ILI9341 TFT Display (Optional)
- **Resolution**: 320x240 pixels
- **Interface**: SPI (hardware accelerated)
- **Features**: Real-time latency graphs, status display, touch support
- **Connection**: SPI pins on Board 2 (Bridge)

### 🔴 Important: Serial Wire Configuration

**UART wires must be crossed between the two boards:**
- Board 1 TX → Board 2 RX
- Board 1 RX → Board 2 TX
- Common GND

This allows bidirectional communication for KMBox commands and display data.

### Power Requirements
- USB bus powered (5V)
- Typical consumption: 150-300mA (varies with display usage)
- TFT display adds ~80-150mA
- No external power supply required

## Quick Start

### 1. Clone and Build

```bash
git clone --recursive https://github.com/ramseymcgrath/RaspberryKMBox.git
cd RaspberryKMBox

# If you already cloned without --recursive:
git submodule update --init --recursive

# Build options:
./build.sh metro          # Main KMBox for Metro RP2350
./build.sh bridge-metro   # Bridge for Metro RP2350 with display
./build.sh dual-metro     # Build both targets
./build.sh all            # Build all configurations
```

### 2. Flash Firmware

**Board 1 (USB Proxy)**:
- Hold **BOOTSEL** while connecting USB-C to PC
- Drag `build-metro/PIOKMbox.uf2` to mounted **RP2350** drive
- Board reboots automatically

**Board 2 (Bridge - Optional)**:
- Hold **BOOTSEL** while connecting USB-C to PC
- Drag `bridge/build-metro/kmbox_bridge.uf2` to mounted **RP2350** drive
- Board reboots automatically

### 3. Wire the Boards

**UART Connection (crossed)**:
```
Board 1 (Proxy)     Board 2 (Bridge)
GPIO TX      ────────────→ GPIO RX
GPIO RX      ←──────────── GPIO TX
GND          ────────────── GND
```

**Display Connection** (on Board 2):
- See `bridge/README.md` for ILI9341 wiring details

### 4. Connect Devices

1. **Mouse/Keyboard** → Board 1 USB-A port
2. **Board 1 USB-C** → PC (appears as passthrough device)
3. **Board 2 USB-C** → PC (for serial input commands)

### 5. Verify Operation

- **NeoPixel**: Changes color based on connected devices (see Status Indicators below)
- **Mouse/Keyboard**: Should work normally through PC
- **Display**: Shows connection status and latency (if bridge installed)
- **Serial**: Board 1 accepts KMBox commands via UART

## Using KMBox Commands

### Serial Connection
- **Interface**: UART (hardware serial, crossed between boards)
- **Baud Rate**: 115200 (configurable, speeds are uncapped in USB CDC mode)
- **Protocol**: KMBox-compatible text and binary commands

### Basic Text Commands
```text
km.move(100, 50)      # Move mouse relative (+X right, +Y down)
km.left(1)            # Press left button  
km.left(0)            # Release left button
km.click(0)           # Left click (0=left, 1=right, 2=middle)
km.wheel(5)           # Scroll up (negative=down)
km.lock_mx(1)         # Lock X axis (ignore physical mouse X)
km.lock_my(1)         # Lock Y axis (ignore physical mouse Y)
km.unlock_mx()        # Unlock X axis
km.unlock_my()        # Unlock Y axis
```

### Fast Binary Protocol

For ultra-low latency (<50µs), use 8-byte binary packets:

```python
# Fast mouse move (0x01 command)
packet = bytes([0x01, x_lo, x_hi, y_lo, y_hi, buttons, wheel, 0x00])
serial.write(packet)
```

**Advantages**:
- Bypasses text parsing for minimal latency
- Fixed 8-byte packets for predictable timing
- Ideal for high-frequency automation (1000+ commands/sec)

### Monitor Mode

Real-time button state queries for automation:

```text
km.monitor(1)         # Enable monitoring
km.isdown_left()      # Query left button (returns 0 or 1)
km.isdown_right()     # Query right button
km.isdown_middle()    # Query middle button
km.isdown_side1()     # Query side button 1
km.isdown_side2()     # Query side button 2
km.monitor(0)         # Disable monitoring
```

## KMBox Compatibility

**Fully Compatible** with KMBox B+, Ferrum and Macku protocols

✅ **Mouse Control**: Movement, all buttons, scroll wheel  
✅ **Axis Locking**: X/Y movement and button masking  
✅ **Monitor Mode**: Real-time button state queries  
✅ **Fast Binary Protocol**: <50µs latency for automation  
✅ **Smooth Injection**: Velocity matching and timing control  
✅ **Movement Humanization**: Anti-detection with Bezier easing

## Movement Humanization

### Overview

Advanced anti-detection system that simulates natural human mouse movement patterns through adaptive jitter, velocity suppression, and overshoot simulation. All features are hardware-accelerated with <10 cycle overhead.

### How It Works

#### **Movement-Aware Scaling**
Humanization intensity adapts based on movement distance:
- **Small movements (0-20px)**: Higher jitter (0.7-0.8x) simulates hand tremor during precise positioning
- **Medium movements (20-60px)**: Moderate jitter (0.3-0.7x) balances precision and speed
- **Large movements (60-110px)**: Reduced jitter (0.1-0.3x) for intentional movements
- **Very large movements (110+px)**: Minimal jitter (0.05-0.09x) keeps fast flicks snappy

#### **Velocity-Based Suppression**
Jitter automatically fades as movement slows:
- At full speed: 1.0x jitter intensity
- As velocity approaches zero: jitter reduces to 0.1x
- Prevents "shaky" cursor after movement completes
- Mimics natural hand settling behavior

#### **Physical Input Protection**
- **Physical mouse/keyboard**: Zero humanization (untouched)
- **Synthetic injections**: Humanization applied with 0-1ms per sub-step
- Separate processing paths ensure real input feels native

### Humanization Modes

Control via button press (GPIO 7) or serial command. Each mode is optimized for specific use cases:

| Mode | Jitter | Overshoot | Onset Delay | Use Case |
|------|--------|-----------|-------------|----------|
| **OFF** | None | Disabled | None | Testing, maximum precision |
| **LOW** | ±0.06px | Disabled | 0-1 frames | Competitive gaming, fast reactions |
| **MEDIUM** | ±0.17px | 5% chance | 1-3 frames | **Default** - General use |
| **HIGH** | ±0.33px | 10% chance | 2-6 frames | Maximum stealth, anti-cheat |

#### Mode Details

**OFF Mode**:
- Linear movement, no variations
- Max 16px per frame (fixed)
- Best for: Testing, high-speed automation where detection isn't a concern

**LOW Mode**:
- Barely perceptible jitter (sensor noise floor)
- ±1% delivery error per movement
- Max 15-17px per frame (randomized per session)
- Accumulator clamp: ±4px
- Best for: FPS games where fast response matters

**MEDIUM Mode** (Default):
- Matches physical mouse sensor noise (~±0.17px)
- ±2% delivery error per movement
- Max 13-19px per frame (randomized per session)
- 5% overshoot chance on 15-120px moves (max 0.5px overshoot)
- Accumulator clamp: ±3px
- Best for: Most games and general automation

**HIGH Mode**:
- Upper bound of sensor noise (~±0.33px)
- ±3% delivery error per movement
- Max 10-22px per frame (randomized per session)
- 10% overshoot chance on 15-120px moves (max 1.0px overshoot)
- Accumulator clamp: ±2px (tightest)
- Best for: Maximum human-like behavior, stealth automation

### Additional Features

#### **Bezier Easing Curves**
- Cubic ease-in-out for large movements (natural acceleration/deceleration)
- Quadratic ease-out for quick corrections
- Automatic selection based on movement characteristics

#### **Micro-Jitter Injection**
- Simulates hand tremor (±1-2 pixels)
- Context-aware: More jitter during mid-movement
- Probabilistic: 40% chance per frame (not constant)
- Excluded from precision micro-adjustments

#### **Overshoot & Correction**
- 5-10% chance to overshoot target by 0.5-1.0px (mode dependent)
- Smooth correction over 2-4 frames
- Only triggered on medium/large movements (>15px)

#### **Per-Session Randomization**
- Base parameters vary on initialization
- ±1 pixel variation per movement
- ±1 frame variation for movements >3 frames
- Prevents statistical fingerprinting

### Security & Detection Resistance

**Resistant To:**
- ✅ Statistical analysis (no fixed patterns)
- ✅ Velocity profiling (non-linear curves with natural variation)
- ✅ Tremor detection (includes realistic hand micro-movements)
- ✅ Timing analysis (randomized frame counts and thresholds)
- ✅ ML fingerprinting (per-session parameter randomization)

**Detection Risk**: Low (requires extensive data collection + advanced ML models)

### Button Control (GPIO 7)

**Short press (< 3 seconds)**: Cycle humanization modes
- LED color feedback:
  - 🔴 **Red**: OFF
  - 🟡 **Yellow**: LOW
  - 🟢 **Green**: MEDIUM (default)
  - 🔵 **Cyan**: HIGH

**Long press (≥ 3 seconds)**: Reset USB stacks

For detailed technical documentation, see [HUMANIZATION.md](HUMANIZATION.md).

## Status Indicators

### LED Feedback  

- **Fast blink**: Device connected/active
- **Slow blink**: Device suspended or error
- **Solid**: Normal operation

### NeoPixel Colors

- **Blue**: Starting up
- **Green**: USB device only  
- **Orange**: USB host only
- **Cyan**: Both USB stacks active
- **Magenta**: Mouse connected
- **Yellow**: Keyboard connected  
- **Pink**: Mouse + keyboard connected
- **Red**: Error state
- **Purple**: Suspended

### Humanization Mode Colors (button press)

- 🔴 **Red**: OFF (no humanization)
- 🟡 **Yellow**: LOW (minimal)  
- 🟢 **Green**: MEDIUM (default)
- 🔵 **Cyan**: HIGH (maximum)

## KMBox Bridge - Visual Feedback & Autopilot

An optional companion firmware that runs on a separate **Adafruit Metro RP2350** with ILI9341 TFT display for real-time visual feedback and computer vision-based autopilot capabilities.

### Architecture
```
┌─────────────┐  USB CDC     ┌──────────────────┐  UART (crossed)  ┌───────────────┐
│   PC Tool   │  (commands)  │  Metro RP2350    │   115200 baud    │ KMBox Metro   │
│  (input)    │◄───────────►│  (Bridge/Display)│◄───────────────►│  (USB Proxy)  │
└─────────────┘              └──────────────────┘                   └───────────────┘
                                      │
                                      ├─ ILI9341 TFT (SPI)
                                      └─ Touch Controller (optional)
```

### Key Features
- **ILI9341 Display**: 320x240 real-time status, latency graphs, connection indicators
- **USB CDC Interface**: Receives serial input commands from PC for tracking and automation
- **Color Tracking**: Hardware-accelerated blob detection and centroid calculation
- **UART Communication**: Bidirectional serial between bridge and main KMBox
- **Touch Support**: Optional XPT2046/FT6206 for interactive controls
- **Temperature Monitoring**: Real-time thermal tracking with visual gauges

### Display Information
- **Connection Status**: USB proxy state, device types connected
- **Latency Graphs**: Real-time command processing times
- **Protocol Stats**: Command counts, error rates
- **Temperature**: Board thermal monitoring
- **Frame Rate**: CV processing performance

### Quick Setup
1. **Build**: `./build.sh dual-metro` (builds both boards)
2. **Wire**: Connect UART crossed (TX→RX, RX→TX) + GND between boards
3. **Display**: Connect ILI9341 to Bridge Metro's SPI pins (see `bridge/README.md`)
4. **Flash Board 1**: `build-metro/PIOKMbox.uf2` (USB proxy)
5. **Flash Board 2**: `bridge/build-metro/kmbox_bridge.uf2` (display/bridge)

### Status LEDs
- **Onboard LED**: Heartbeat (500ms blink)
- **NeoPixel**: 
  - RED = Idle/waiting
  - GREEN = Tracking active
  - YELLOW = Tracking disabled
  - BLUE = USB activity

For detailed wiring diagrams, Python client examples, and display configuration, see [`bridge/README.md`](bridge/README.md).

## How It Works

### Architecture Overview

1. **Core 1** runs TinyUSB host on PIO-USB to communicate with your physical mouse/keyboard
2. The firmware reads HID report descriptors and caches VID/PID/strings from attached devices
3. **Core 0** exposes a TinyUSB HID device to the PC, mirroring the attached device's identity
4. When VID/PID changes, the device re-enumerates to reflect the new identity
5. Physical HID input and KMBox serial commands are combined with intelligent axis locking

### USB Passthrough

- **Transparent Identity**: PC sees original mouse/keyboard, not the RP2350
- **Report Mirroring**: All HID reports forwarded with minimal latency (<1ms)
- **Dynamic Re-enumeration**: Automatically adapts when devices change
- **String Descriptors**: Manufacturer, product names mirrored when available

### Serial Command Injection

- **Dual Input**: Physical and synthetic input coexist seamlessly
- **Axis Locking**: Selective X/Y/button filtering for precise control
- **Smooth Injection**: Velocity-matched movement with frame-perfect timing
- **Priority Handling**: Physical input takes priority, synthetic fills gaps

## Development

### Project Layout

```text
PIOKMbox/
├── PIOKMbox.c               # Main firmware (core orchestration)
├── usb_hid.*                # HID device/host, VID/PID mirroring
├── led_control.*            # LED & WS2812 control
├── watchdog.*               # Hardware/software watchdog
├── kmbox_serial_handler.*   # KMBox UART integration
├── smooth_injection.*       # Humanized movement engine
├── humanization_lut.*       # Precomputed jitter lookup tables
├── bridge_handler.*         # Bridge communication protocol
├── ws2812.pio               # PIO program for NeoPixel
├── defines.h, config.h      # Configuration and pin definitions
├── lib/
│   ├── Pico-PIO-USB/        # PIO USB library
│   ├── kmbox-commands/      # KMBox command parser
│   └── bridge-protocol/     # Bridge communication
├── bridge/                  # Bridge firmware with display
│   ├── main.c               # Bridge entry point
│   ├── tft_display.*        # ILI9341 driver
│   ├── latency_tracker.*    # Performance monitoring
│   ├── core1_translator.*   # CV processing
│   └── protocol_luts.*      # Protocol translation
└── build.sh                 # Multi-target build script
```

### Build Targets

```bash
./build.sh pico2         # RP2350 (Pico 2)
./build.sh metro         # Metro RP2350 (main KMBox)
./build.sh bridge        # Bridge (XIAO RP2350)
./build.sh bridge-metro  # Bridge (Metro RP2350)
./build.sh dual-metro    # Both Metro boards
./build.sh all           # All configurations
```

Add `clean` argument to force rebuild: `./build.sh all clean`

### Configuration

Build configuration presets in `defines.h`:
- `BUILD_CONFIG_DEVELOPMENT` (default - verbose logging)
- `BUILD_CONFIG_PRODUCTION` (optimized, minimal logging)
- `BUILD_CONFIG_TESTING` (extended diagnostics)
- `BUILD_CONFIG_DEBUG` (full debug symbols)

Pin assignments, LED timings, watchdog intervals in `defines.h` and `config.h`.

### Clock Speeds

- **RP2350**: 300MHz (default, optimized for PIO-USB)
- **Bridge**: 280MHz (balanced for display + UART)
- Tuned for stable USB enumeration and TFT refresh rates


## Troubleshooting

### USB Issues

**Device not recognized**:
1. Verify 5V power to USB host port
2. Check D+/D- wiring (GPIO 16/17)
3. Try different USB cable (some are power-only)
4. Ensure physical device is supported (check debug UART)

**Re-enumeration loops**:
- Usually caused by unstable attached device
- Check USB cable quality
- Verify power supply stability
- Review debug logs on GPIO 0/1 @ 115200 baud

**Passthrough not working**:
- LED should show device connected state
- Open debug UART to verify device detection
- Some devices with complex HID descriptors may need adjustments

### Serial Communication

**No response to KMBox commands**:
1. Verify UART wires are crossed (TX→RX, RX→TX)
2. Check baud rate (115200 default)
3. Ensure common ground connection
4. Test with simple command: `km.move(10, 10)`

**Display not updating**:
1. Verify SPI connections to ILI9341
2. Check bridge firmware is flashed correctly
3. Review bridge debug output
4. Ensure TFT power (3.3V or 5V depending on module)

### Performance

**High latency**:
- Check CPU clock speeds in CMakeLists.txt
- Verify humanization mode (OFF = lowest latency)
- Monitor temperature (thermal throttling)
- Reduce display update rate if using bridge

**Movement feels sluggish**:
- Try OFF or LOW humanization mode
- Check mouse polling rate (1000Hz recommended)
- Verify physical mouse has high-quality sensor

### Build Issues

**CMake errors**:
```bash
# Ensure Pico SDK is installed and path is set
export PICO_SDK_PATH=/path/to/pico-sdk

# Clean and rebuild
./build.sh metro clean
```

**Flash failures**:
- Hold BOOTSEL button firmly during USB connect
- Try different USB port/cable
- Verify .uf2 file isn't corrupted (re-download if needed)

## Advanced Usage

### Custom Humanization Profiles

Edit `humanization_lut.c` to create custom jitter profiles. The LUT (lookup table) defines jitter multipliers based on movement distance. Regenerate with: `tools/generate_lut.py`

### Serial Protocol Extensions

Extend KMBox commands in `lib/kmbox-commands/`. Add new commands by:
1. Define command structure
2. Add parser in `kmbox_serial_handler.c`
3. Implement handler logic
4. Update protocol documentation

### Display Customization

Modify bridge display in `bridge/tft_display.c`:
- Change color schemes
- Add custom widgets
- Adjust refresh rates
- Implement touch controls

See `bridge/README.md` for detailed display API.

## Performance Metrics

Typical performance on Metro RP2350 @ 300MHz:

| Operation | Latency | Notes |
|-----------|---------|-------|
| USB passthrough | <1ms | Report forwarding |
| Text command | <100µs | Parsing + execution |
| Binary command | <50µs | Direct execution |
| Humanization overhead | <10 cycles | Per pixel calculation |
| Display update | 16-33ms | 30-60 FPS typical |
| UART transmission | 87µs | 8 bytes @ 115200 |

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Test thoroughly on hardware
4. Document changes in code and README
5. Submit a pull request with detailed description

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

Main project files follow standard open-source practices. Libraries under `lib/` retain their respective licenses:
- **Pico-PIO-USB**: See lib/Pico-PIO-USB/LICENSE
- **TinyUSB**: MIT License
- **Pico SDK**: BSD 3-Clause License

## Acknowledgments

- Raspberry Pi Foundation for Pico SDK and documentation
- TinyUSB project for USB stack
- Sekigon-gonnoc for Pico-PIO-USB
- KMBox community for protocol documentation
- Adafruit for excellent RP2350 hardware

## Support

- **Issues**: [GitHub Issues](https://github.com/ramseymcgrath/RaspberryKMBox/issues)
- **Discussions**: [GitHub Discussions](https://github.com/ramseymcgrath/RaspberryKMBox/discussions)
- **Documentation**: See individual .md files for detailed topics

---

**Note**: This project is for educational and accessibility purposes. Users are responsible for complying with applicable terms of service and regulations.
