#!/usr/bin/env python3
"""
CP2110 KMBox Client
High-performance HID-based serial communication with KMBox via CP2110 USB UART5 Click

The CP2110 uses USB HID for data transfer, providing:
- 1ms polling interval (1000 Hz)
- Low, predictable latency
- No CDC/ACM driver overhead

ADVANCED CP2110 FEATURES:
========================
Beyond basic UART bridge, the CP2110 offers:

1. GPIO Control (10 pins):
   - Read/write GPIO pins directly from PC
   - Use for status LEDs, buttons, external triggers
   
2. UART Status Monitoring:
   - Check TX/RX buffer fill levels
   - Detect parity/framing errors in real-time
   
3. FIFO Management:
   - Purge TX/RX buffers for clean state
   - Check buffer status before critical operations
   
4. Line Break Control:
   - Send UART break signals for protocol resets
   - Useful for bootloader entry or attention signals
   
5. Hardware Flow Control:
   - CTS/RTS pins for reliable high-speed transfers
   - Automatic flow control in hardware

Implements handshake protocol for reliable connection with LED indicators:
- WAITING:     Breathing light blue - firmware waiting for connection
- CONNECTING:  Yellow - handshake in progress
- CONNECTED:   Green - connected and ready
- ACTIVE:      Cyan - actively receiving commands
- DISCONNECTED: Breathing orange-red - connection lost

Requirements:
    pip install hidapi

Usage:
    from cp2110_kmbox import CP2110KMBox
    
    kmbox = CP2110KMBox()
    kmbox.connect()      # Performs handshake
    kmbox.move(10, 20)
    kmbox.click('left')
    
    # Advanced features:
    print(kmbox.get_uart_status())     # Check UART buffer status
    kmbox.purge_fifos()                # Clear TX/RX buffers
    gpio = kmbox.get_gpio()            # Read GPIO pin states
    kmbox.set_gpio(0x01, 0x01)         # Set GPIO pin 0 high
    kmbox.send_line_break(100)         # Send 100ms line break
    
    kmbox.disconnect()   # Graceful disconnect
"""

import hid
import time
import struct
import threading
from typing import Optional, Tuple, Callable, Dict, Any
from enum import Enum, IntEnum
from dataclasses import dataclass

# Silicon Labs CP2110 USB IDs
CP2110_VID = 0x10C4
CP2110_PID = 0xEA80

# ============================================================================
# CP2110 HID Report IDs (from AN434 Interface Specification)
# ============================================================================

class ReportID(IntEnum):
    """CP2110 HID Report IDs for feature reports and control transfers."""
    # Data Transfer (Interrupt Transfer) - Report IDs 0x00-0x3F
    # The report ID indicates the number of data bytes (1-63)
    
    # Device Configuration (Control Transfer)
    RESET_DEVICE = 0x40           # Set: Reset the device
    GET_SET_UART_ENABLE = 0x41    # Get/Set: Enable or disable UART
    GET_UART_STATUS = 0x42        # Get: UART transmit/receive status
    PURGE_FIFOS = 0x43            # Set: Purge TX and/or RX FIFOs
    GET_GPIO_VALUES = 0x44        # Get: Read GPIO pin states
    SET_GPIO_VALUES = 0x45        # Set: Write GPIO pin states
    GET_VERSION = 0x46            # Get: Part number and version
    GET_SET_LOCK_BYTE = 0x47      # Get/Set: OTP lock byte
    
    # UART Configuration (Control Transfer)
    GET_SET_UART_CONFIG = 0x50    # Get/Set: UART config (baud, parity, etc.)
    SET_TRANSMIT_LINE_BREAK = 0x51  # Set: Start transmitting line break
    SET_STOP_LINE_BREAK = 0x52    # Set: Stop transmitting line break
    
    # USB Customization (Control Transfer) - OTP programmable
    GET_SET_USB_CONFIG = 0x60     # Get/Set: USB configuration
    GET_SET_MFG_STRING_1 = 0x61   # Get/Set: Manufacturing string 1
    GET_SET_MFG_STRING_2 = 0x62   # Get/Set: Manufacturing string 2
    GET_SET_PRODUCT_STRING_1 = 0x63  # Get/Set: Product string 1
    GET_SET_PRODUCT_STRING_2 = 0x64  # Get/Set: Product string 2
    GET_SET_SERIAL_STRING = 0x65  # Get/Set: Serial number string
    GET_SET_PIN_CONFIG = 0x66     # Get/Set: Pin configuration


class Parity(IntEnum):
    """UART parity settings."""
    NONE = 0
    ODD = 1
    EVEN = 2
    MARK = 3
    SPACE = 4


class FlowControl(IntEnum):
    """UART flow control settings."""
    DISABLED = 0
    HARDWARE = 1  # RTS/CTS


class DataBits(IntEnum):
    """UART data bits settings."""
    FIVE = 0
    SIX = 1
    SEVEN = 2
    EIGHT = 3


class StopBits(IntEnum):
    """UART stop bits settings."""
    ONE = 0       # 1 stop bit (SHORT)
    ONE_FIVE = 1  # 1.5 stop bits (LONG, only valid with 5 data bits)
    TWO = 1       # 2 stop bits (LONG, valid with 6-8 data bits)


class FifoPurge(IntEnum):
    """FIFO purge options."""
    TX = 0x01     # Purge transmit FIFO
    RX = 0x02     # Purge receive FIFO
    BOTH = 0x03   # Purge both FIFOs


# GPIO pin definitions for CP2110 (directly controllable from PC!)
class GPIOPin(IntEnum):
    """CP2110 GPIO pins. These can be read/written directly from the PC."""
    GPIO_0 = 0   # Can be TX LED
    GPIO_1 = 1   # Can be RX LED
    GPIO_2 = 2   # Can be RS485 TX enable
    GPIO_3 = 3
    GPIO_4 = 4
    GPIO_5 = 5
    GPIO_6 = 6
    GPIO_7 = 7
    GPIO_8 = 8   # Can be clock output
    GPIO_9 = 9   # Can be TX toggle


@dataclass
class UARTStatus:
    """UART status information from CP2110."""
    tx_fifo_bytes: int      # Number of bytes in TX FIFO (0-480)
    rx_fifo_bytes: int      # Number of bytes in RX FIFO (0-480)
    parity_error: bool      # Parity error detected
    overrun_error: bool     # RX overrun error
    line_break: bool        # Line break detected
    
    @property
    def tx_full(self) -> bool:
        """Check if TX FIFO is nearly full."""
        return self.tx_fifo_bytes > 400
    
    @property
    def rx_empty(self) -> bool:
        """Check if RX FIFO is empty."""
        return self.rx_fifo_bytes == 0
    
    def __str__(self) -> str:
        errors = []
        if self.parity_error:
            errors.append("PARITY")
        if self.overrun_error:
            errors.append("OVERRUN")
        if self.line_break:
            errors.append("BREAK")
        error_str = ", ".join(errors) if errors else "none"
        return f"TX:{self.tx_fifo_bytes}/480 RX:{self.rx_fifo_bytes}/480 Errors:{error_str}"


@dataclass
class VersionInfo:
    """CP2110 version information."""
    part_number: int
    version: int
    
    def __str__(self) -> str:
        return f"CP{self.part_number} v{self.version}"


@dataclass
class UARTConfig:
    """UART configuration settings."""
    baud_rate: int
    data_bits: DataBits
    parity: Parity
    stop_bits: StopBits
    flow_control: FlowControl
    
    def __str__(self) -> str:
        parity_char = {Parity.NONE: 'N', Parity.ODD: 'O', Parity.EVEN: 'E', 
                       Parity.MARK: 'M', Parity.SPACE: 'S'}[self.parity]
        stop_char = '1' if self.stop_bits == StopBits.ONE else '2'
        flow_str = " HW-FC" if self.flow_control == FlowControl.HARDWARE else ""
        return f"{self.baud_rate} {self.data_bits.value + 5}{parity_char}{stop_char}{flow_str}"


# Legacy constants for backward compatibility
# Increase default to 2 Mbps to match KMBox firmware defaults
DEFAULT_BAUD_RATE = 115200  # Updated to high-speed (2 Mbps)
DEFAULT_DATA_BITS = 8
DEFAULT_PARITY = 0  # None
DEFAULT_STOP_BITS = 0  # 1 stop bit
DEFAULT_FLOW_CONTROL = 0  # None

# Protocol constants
PROTOCOL_VERSION = "1.0"
HEARTBEAT_INTERVAL_MS = 2000  # Send heartbeat every 2 seconds
CONNECTION_TIMEOUT_MS = 5000  # Connection timeout

# ============================================================================
# FAST BINARY COMMAND PROTOCOL
# ============================================================================
# Ultra-fast binary protocol for minimal latency mouse/keyboard control
# At 2 Mbps: 8 bytes takes only ~40µs vs ~700µs at 115200
#
# Command format (8 bytes fixed, no parsing overhead):
#   Byte 0: Command type (FAST_CMD_*)
#   Byte 1-7: Command-specific payload

class FastCmd(IntEnum):
    """Fast binary command types (8-byte fixed packets, 4-byte aligned)."""
    MOUSE_MOVE = 0x01      # [X_lo][X_hi][Y_lo][Y_hi][Buttons][Wheel][0x00]
    MOUSE_CLICK = 0x02     # [Button][Count][0x00][0x00][0x00][0x00][0x00]
    KEY_PRESS = 0x03       # [Keycode][Modifiers][0x00][0x00][0x00][0x00][0x00]
    KEY_COMBO = 0x04       # [Key1][Key2][Key3][Key4][Modifiers][0x00][0x00]
    MULTI_MOVE = 0x05      # [X1][Y1][X2][Y2][X3][Y3][Flags] - 3 moves in 1!
    MOUSE_ABS = 0x06       # [X_lo][X_hi][Y_lo][Y_hi][Buttons][0x00][0x00]
    SMOOTH_MOVE = 0x07     # [X_lo][X_hi][Y_lo][Y_hi][Mode][0x00][0x00]
    SMOOTH_CONFIG = 0x08   # [MaxPerFrame][VelMatch][0x00][0x00][0x00][0x00][0x00]
    SMOOTH_CLEAR = 0x09    # Clear smooth injection queue
    TIMED_MOVE = 0x0A      # [X_lo][X_hi][Y_lo][Y_hi][Time_lo][Time_hi][Mode]
    SYNC = 0x0B            # [SeqNum][Time0][Time1][Time2][Time3][0x00][0x00]
    PING = 0xFE            # Fast ping (response: 0xFF)
    RESPONSE = 0xFF        # Response/ACK


class SmoothMode(IntEnum):
    """Smooth injection modes."""
    IMMEDIATE = 0          # Add directly to accumulator
    SMOOTH = 1             # Spread across frames (rate-limited)
    VELOCITY_MATCHED = 2   # Match current mouse velocity
    MICRO = 3              # Sub-pixel micro-adjustments


class FastBtn(IntEnum):
    """Button bit flags for fast commands."""
    LEFT = 0x01
    RIGHT = 0x02
    MIDDLE = 0x04
    BACK = 0x08
    FORWARD = 0x10


# USB HID Keycodes (common ones)
class KeyCode(IntEnum):
    """USB HID keyboard keycodes."""
    A = 0x04
    B = 0x05
    C = 0x06
    D = 0x07
    E = 0x08
    F = 0x09
    G = 0x0A
    H = 0x0B
    I = 0x0C
    J = 0x0D
    K = 0x0E
    L = 0x0F
    M = 0x10
    N = 0x11
    O = 0x12
    P = 0x13
    Q = 0x14
    R = 0x15
    S = 0x16
    T = 0x17
    U = 0x18
    V = 0x19
    W = 0x1A
    X = 0x1B
    Y = 0x1C
    Z = 0x1D
    NUM_1 = 0x1E
    NUM_2 = 0x1F
    NUM_3 = 0x20
    NUM_4 = 0x21
    NUM_5 = 0x22
    NUM_6 = 0x23
    NUM_7 = 0x24
    NUM_8 = 0x25
    NUM_9 = 0x26
    NUM_0 = 0x27
    ENTER = 0x28
    ESCAPE = 0x29
    BACKSPACE = 0x2A
    TAB = 0x2B
    SPACE = 0x2C
    F1 = 0x3A
    F2 = 0x3B
    F3 = 0x3C
    F4 = 0x3D
    F5 = 0x3E
    F6 = 0x3F
    F7 = 0x40
    F8 = 0x41
    F9 = 0x42
    F10 = 0x43
    F11 = 0x44
    F12 = 0x45


class KeyMod(IntEnum):
    """Keyboard modifier bit flags."""
    NONE = 0x00
    CTRL_LEFT = 0x01
    SHIFT_LEFT = 0x02
    ALT_LEFT = 0x04
    GUI_LEFT = 0x08  # Windows/Command key
    CTRL_RIGHT = 0x10
    SHIFT_RIGHT = 0x20
    ALT_RIGHT = 0x40
    GUI_RIGHT = 0x80

FAST_CMD_PACKET_SIZE = 8


class ConnectionState(Enum):
    """Connection state matching firmware states."""
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    ACTIVE = 3


class CP2110KMBox:
    """High-performance KMBox client using CP2110 HID-to-UART bridge with handshake protocol."""
    
    def __init__(self, vid: int = CP2110_VID, pid: int = CP2110_PID):
        self.vid = vid
        self.pid = pid
        self.device: Optional[hid.device] = None
        self._state = ConnectionState.DISCONNECTED
        self._rx_buffer = bytearray()
        self._lock = threading.Lock()
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._heartbeat_stop = threading.Event()
        self._last_activity = 0
        self._firmware_version = "unknown"
        self._on_state_change: Optional[Callable[[ConnectionState], None]] = None
    
    @property
    def state(self) -> ConnectionState:
        """Get current connection state."""
        return self._state
    
    @property
    def is_connected(self) -> bool:
        """Check if connected and ready for commands."""
        return self._state in (ConnectionState.CONNECTED, ConnectionState.ACTIVE)
    
    @property
    def firmware_version(self) -> str:
        """Get firmware version (available after connect)."""
        return self._firmware_version
    
    def set_state_callback(self, callback: Callable[[ConnectionState], None]):
        """Set callback for state changes."""
        self._on_state_change = callback
    
    def _set_state(self, new_state: ConnectionState):
        """Update state and trigger callback."""
        if self._state != new_state:
            old_state = self._state
            self._state = new_state
            if self._on_state_change:
                try:
                    self._on_state_change(new_state)
                except Exception as e:
                    print(f"State callback error: {e}")
    
    def connect(self, baud_rate: int = DEFAULT_BAUD_RATE, 
                handshake: bool = True, timeout_ms: int = 3000) -> bool:
        """
        Connect to CP2110 and optionally perform handshake.
        
        Args:
            baud_rate: UART baud rate (default 115200)
            handshake: Whether to perform handshake protocol (default True)
            timeout_ms: Handshake timeout in milliseconds
            
        Returns:
            True if connection (and handshake if requested) successful
        """
        try:
            self.device = hid.device()
            self.device.open(self.vid, self.pid)
            self.device.set_nonblocking(1)
            
            # Configure UART
            self._configure_uart(baud_rate)
            self._enable_uart(True)
            
            print(f"CP2110 device opened (VID={self.vid:04X}, PID={self.pid:04X})")
            print(f"UART configured: {baud_rate} baud, 8N1")
            
            if handshake:
                self._set_state(ConnectionState.CONNECTING)
                if not self._perform_handshake(timeout_ms):
                    print("Handshake failed!")
                    self._set_state(ConnectionState.DISCONNECTED)
                    return False
            else:
                self._set_state(ConnectionState.CONNECTED)
            
            # Start heartbeat thread
            self._start_heartbeat()
            
            return True
            
        except Exception as e:
            print(f"Failed to connect to CP2110: {e}")
            self._set_state(ConnectionState.DISCONNECTED)
            return False
    
    def _perform_handshake(self, timeout_ms: int) -> bool:
        """Perform handshake protocol with firmware."""
        print("Performing handshake...")
        
        # Step 1: Send PING
        self._write_raw(b"KMBOX_PING\n")
        
        # Step 2: Wait for PONG
        start = time.monotonic()
        while (time.monotonic() - start) * 1000 < timeout_ms:
            response = self._read_line(100)
            if response:
                if response.startswith("KMBOX_PONG:"):
                    # Extract version
                    version_part = response[11:].strip()
                    if version_part.startswith("v"):
                        self._firmware_version = version_part[1:]
                    else:
                        self._firmware_version = version_part
                    print(f"Firmware responded: {response}")
                    break
        else:
            print("Timeout waiting for PONG")
            return False
        
        # Step 3: Send CONNECT
        self._write_raw(b"KMBOX_CONNECT\n")
        
        # Step 4: Wait for READY
        start = time.monotonic()
        while (time.monotonic() - start) * 1000 < timeout_ms:
            response = self._read_line(100)
            if response:
                if response == "KMBOX_READY":
                    print("Handshake complete - connected!")
                    self._set_state(ConnectionState.CONNECTED)
                    self._last_activity = time.monotonic()
                    return True
        
        print("Timeout waiting for READY")
        return False
    
    def _start_heartbeat(self):
        """Start background heartbeat thread."""
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()
    
    def _stop_heartbeat(self):
        """Stop heartbeat thread."""
        self._heartbeat_stop.set()
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=1.0)
            self._heartbeat_thread = None
    
    def _heartbeat_loop(self):
        """Background heartbeat to maintain connection."""
        while not self._heartbeat_stop.wait(HEARTBEAT_INTERVAL_MS / 1000):
            if self.is_connected:
                # Check if we need to send heartbeat
                elapsed = (time.monotonic() - self._last_activity) * 1000
                if elapsed > HEARTBEAT_INTERVAL_MS:
                    try:
                        with self._lock:
                            self._write_raw(b"KMBOX_PING\n")
                            # Read response (don't block too long)
                            response = self._read_line(100)
                            if response and response.startswith("KMBOX_PONG"):
                                self._last_activity = time.monotonic()
                            elif elapsed > CONNECTION_TIMEOUT_MS:
                                print("Connection timeout - no heartbeat response")
                                self._set_state(ConnectionState.DISCONNECTED)
                    except Exception as e:
                        print(f"Heartbeat error: {e}")
    
    def disconnect(self):
        """Gracefully disconnect from CP2110."""
        # Stop heartbeat first
        self._stop_heartbeat()
        
        if self.device and self.is_connected:
            try:
                # Send graceful disconnect
                self._write_raw(b"KMBOX_DISCONNECT\n")
                time.sleep(0.1)  # Give firmware time to process
            except:
                pass
        
        if self.device:
            try:
                self._enable_uart(False)
                self.device.close()
            except:
                pass
            self.device = None
        
        self._set_state(ConnectionState.DISCONNECTED)
        print("Disconnected from CP2110")
    
    def reconnect(self, timeout_ms: int = 3000) -> bool:
        """Attempt to reconnect after connection loss."""
        print("Attempting reconnection...")
        self.disconnect()
        time.sleep(0.5)
        return self.connect(timeout_ms=timeout_ms)
    
    # ========================================================================
    # ADVANCED CP2110 FEATURES - Direct hardware control from PC
    # ========================================================================
    
    def get_version(self) -> Optional[VersionInfo]:
        """
        Get CP2110 chip version information.
        
        Returns:
            VersionInfo object with part number and version, or None on error
        """
        if not self.device:
            return None
        
        try:
            report = self.device.get_feature_report(ReportID.GET_VERSION, 3)
            if report and len(report) >= 3:
                return VersionInfo(
                    part_number=report[1],
                    version=report[2]
                )
        except Exception as e:
            print(f"Error getting version: {e}")
        
        return None
    
    def get_uart_status(self) -> Optional[UARTStatus]:
        """
        Get UART status including FIFO fill levels and error flags.
        
        This is useful for:
        - Checking if TX buffer has room before sending
        - Checking if RX buffer has data waiting
        - Detecting communication errors
        
        Returns:
            UARTStatus object with buffer levels and error flags, or None on error
        """
        if not self.device:
            return None
        
        try:
            # Report format: [ReportID, TxFifoLow, TxFifoHigh, RxFifoLow, RxFifoHigh, ErrorStatus]
            report = self.device.get_feature_report(ReportID.GET_UART_STATUS, 7)
            if report and len(report) >= 6:
                tx_fifo = report[1] | (report[2] << 8)
                rx_fifo = report[3] | (report[4] << 8)
                errors = report[5] if len(report) > 5 else 0
                
                return UARTStatus(
                    tx_fifo_bytes=tx_fifo,
                    rx_fifo_bytes=rx_fifo,
                    parity_error=bool(errors & 0x01),
                    overrun_error=bool(errors & 0x02),
                    line_break=bool(errors & 0x04)
                )
        except Exception as e:
            print(f"Error getting UART status: {e}")
        
        return None
    
    def purge_fifos(self, which: FifoPurge = FifoPurge.BOTH) -> bool:
        """
        Purge (clear) the TX and/or RX FIFOs.
        
        Use this to:
        - Clear stale data before critical operations
        - Reset after communication errors
        - Ensure clean state for new commands
        
        Args:
            which: FifoPurge.TX, FifoPurge.RX, or FifoPurge.BOTH
            
        Returns:
            True if successful
        """
        if not self.device:
            return False
        
        try:
            self.device.send_feature_report([ReportID.PURGE_FIFOS, int(which)])
            return True
        except Exception as e:
            print(f"Error purging FIFOs: {e}")
            return False
    
    def get_gpio(self) -> Optional[Tuple[int, int]]:
        """
        Read GPIO pin states.
        
        The CP2110 has up to 10 GPIO pins that can be read/written directly
        from the PC, independent of UART data. Useful for:
        - Reading button states
        - Checking status signals
        - Triggering external events
        
        Returns:
            Tuple of (latch_value, gpio_value) where:
            - latch_value: Output latch states (what we're driving)
            - gpio_value: Actual pin states (what we're reading)
            Or None on error
        """
        if not self.device:
            return None
        
        try:
            # Report format: [ReportID, LatchLow, LatchHigh, GPIOLow, GPIOHigh]
            report = self.device.get_feature_report(ReportID.GET_GPIO_VALUES, 5)
            if report and len(report) >= 5:
                latch = report[1] | (report[2] << 8)
                gpio = report[3] | (report[4] << 8)
                return (latch, gpio)
        except Exception as e:
            print(f"Error getting GPIO: {e}")
        
        return None
    
    def set_gpio(self, latch_value: int, write_mask: int) -> bool:
        """
        Write GPIO pin states.
        
        Only pins configured as GPIO outputs will be affected. This allows
        direct control of output pins from the PC, useful for:
        - Controlling status LEDs
        - Triggering external devices
        - Sending handshake signals
        
        Args:
            latch_value: Bit values to write (1=high, 0=low)
            write_mask: Mask of which bits to update (1=update, 0=ignore)
            
        Returns:
            True if successful
            
        Example:
            # Set GPIO pin 0 high, pin 1 low, leave others unchanged
            set_gpio(latch_value=0x01, write_mask=0x03)
        """
        if not self.device:
            return False
        
        try:
            self.device.send_feature_report([
                ReportID.SET_GPIO_VALUES,
                latch_value & 0xFF,
                (latch_value >> 8) & 0xFF,
                write_mask & 0xFF,
                (write_mask >> 8) & 0xFF
            ])
            return True
        except Exception as e:
            print(f"Error setting GPIO: {e}")
            return False
    
    def send_line_break(self, duration_ms: int = 100) -> bool:
        """
        Send a UART line break signal.
        
        A line break is a sustained low signal on the TX line, useful for:
        - Entering bootloader mode on some devices
        - Sending attention/reset signals
        - Protocol-specific uses
        
        Args:
            duration_ms: Break duration in milliseconds
            
        Returns:
            True if successful
        """
        if not self.device:
            return False
        
        try:
            # Start break
            self.device.send_feature_report([ReportID.SET_TRANSMIT_LINE_BREAK])
            
            # Wait for duration
            time.sleep(duration_ms / 1000.0)
            
            # Stop break
            self.device.send_feature_report([ReportID.SET_STOP_LINE_BREAK])
            return True
        except Exception as e:
            print(f"Error sending line break: {e}")
            return False
    
    def reset_device(self) -> bool:
        """
        Reset the CP2110 device.
        
        This performs a full device reset. The USB device will disconnect
        and reconnect. You'll need to call connect() again after this.
        
        Returns:
            True if reset was initiated
        """
        if not self.device:
            return False
        
        try:
            self.device.send_feature_report([ReportID.RESET_DEVICE])
            self._set_state(ConnectionState.DISCONNECTED)
            self.device = None
            return True
        except Exception as e:
            print(f"Error resetting device: {e}")
            return False
    
    def get_uart_config(self) -> Optional[UARTConfig]:
        """
        Get current UART configuration.
        
        Returns:
            UARTConfig object with current settings, or None on error
        """
        if not self.device:
            return None
        
        try:
            report = self.device.get_feature_report(ReportID.GET_SET_UART_CONFIG, 9)
            if report and len(report) >= 9:
                baud = report[1] | (report[2] << 8) | (report[3] << 16) | (report[4] << 24)
                return UARTConfig(
                    baud_rate=baud,
                    parity=Parity(report[5]),
                    flow_control=FlowControl(report[6]),
                    data_bits=DataBits(report[7]),
                    stop_bits=StopBits(report[8])
                )
        except Exception as e:
            print(f"Error getting UART config: {e}")
        
        return None
    
    def set_uart_config(self, config: UARTConfig) -> bool:
        """
        Set UART configuration.
        
        Args:
            config: UARTConfig object with desired settings
            
        Returns:
            True if successful
        """
        if not self.device:
            return False
        
        try:
            self.device.send_feature_report([
                ReportID.GET_SET_UART_CONFIG,
                config.baud_rate & 0xFF,
                (config.baud_rate >> 8) & 0xFF,
                (config.baud_rate >> 16) & 0xFF,
                (config.baud_rate >> 24) & 0xFF,
                int(config.parity),
                int(config.flow_control),
                int(config.data_bits),
                int(config.stop_bits)
            ])
            return True
        except Exception as e:
            print(f"Error setting UART config: {e}")
            return False
    
    def is_uart_enabled(self) -> Optional[bool]:
        """
        Check if UART is currently enabled.
        
        Returns:
            True if enabled, False if disabled, None on error
        """
        if not self.device:
            return None
        
        try:
            report = self.device.get_feature_report(ReportID.GET_SET_UART_ENABLE, 2)
            if report and len(report) >= 2:
                return report[1] == 1
        except Exception as e:
            print(f"Error checking UART enabled: {e}")
        
        return None
    
    # ========================================================================
    # Internal UART configuration methods (updated to use new enums)
    # ========================================================================
    
    def _configure_uart(self, baud_rate: int):
        """Configure CP2110 UART settings."""
        if not self.device:
            return
            
        config = [
            ReportID.GET_SET_UART_CONFIG,
            baud_rate & 0xFF,
            (baud_rate >> 8) & 0xFF,
            (baud_rate >> 16) & 0xFF,
            (baud_rate >> 24) & 0xFF,
            int(Parity.NONE),
            int(FlowControl.DISABLED),
            int(DataBits.EIGHT),
            int(StopBits.ONE),
        ]
        
        self.device.send_feature_report(config)
    
    def _enable_uart(self, enable: bool):
        """Enable or disable CP2110 UART."""
        if not self.device:
            return
            
        report = [ReportID.GET_SET_UART_ENABLE, 1 if enable else 0]
        self.device.send_feature_report(report)
    
    def _write_raw(self, data: bytes) -> int:
        """Write raw bytes to UART."""
        if not self.device:
            return 0
        
        total_sent = 0
        remaining = data
        
        while remaining:
            chunk = remaining[:63]
            remaining = remaining[63:]
            report = bytes([len(chunk)]) + chunk
            
            try:
                self.device.write(report)
                total_sent += len(chunk)
            except Exception as e:
                print(f"Write error: {e}")
                break
        
        return total_sent
    
    def _read_line(self, timeout_ms: int = 100) -> Optional[str]:
        """Read a complete line from UART."""
        if not self.device:
            return None
        
        start = time.monotonic()
        line_buffer = bytearray()
        
        while (time.monotonic() - start) * 1000 < timeout_ms:
            try:
                data = self.device.read(64, 10)
                if data:
                    length = data[0]
                    if length > 0 and length <= 63:
                        chunk = bytes(data[1:1+length])
                        line_buffer.extend(chunk)
                        
                        # Check for newline
                        if b'\n' in line_buffer:
                            line, _, remainder = line_buffer.partition(b'\n')
                            self._rx_buffer = bytearray(remainder)
                            return line.decode('utf-8', errors='ignore').strip()
            except:
                pass
        
        return None
    
    def write(self, data: bytes) -> int:
        """Write data to UART via CP2110."""
        if not self.is_connected:
            return 0
        
        with self._lock:
            self._last_activity = time.monotonic()
            if self._state == ConnectionState.CONNECTED:
                self._set_state(ConnectionState.ACTIVE)
            return self._write_raw(data)
    
    def read(self, timeout_ms: int = 10) -> bytes:
        """Read data from UART via CP2110."""
        if not self.is_connected or not self.device:
            return b''
        
        try:
            data = self.device.read(64, timeout_ms)
            if data:
                length = data[0]
                if length > 0 and length <= 63:
                    self._last_activity = time.monotonic()
                    return bytes(data[1:1+length])
        except Exception as e:
            print(f"Read error: {e}")
        
        return b''
    
    def send_command(self, command: str, wait_response: bool = False, 
                     timeout_ms: int = 100) -> Optional[str]:
        """Send a KMBox command and optionally wait for response."""
        if not self.is_connected:
            return None
        
        if not command.endswith('\n'):
            command += '\n'
        
        with self._lock:
            self._last_activity = time.monotonic()
            if self._state == ConnectionState.CONNECTED:
                self._set_state(ConnectionState.ACTIVE)
            
            self._write_raw(command.encode('utf-8'))
            
            if wait_response:
                return self._read_line(timeout_ms)
        
        return None
    
    def get_status(self) -> Optional[str]:
        """Query firmware status."""
        return self.send_command("KMBOX_STATUS", wait_response=True, timeout_ms=200)
    
    # ========== KMBox Mouse Commands ==========
    
    def move(self, x: int, y: int):
        """Move mouse by relative offset."""
        self.send_command(f"km.move({x},{y})")
    
    def move_to(self, x: int, y: int):
        """Move mouse to absolute position (if supported)."""
        self.send_command(f"km.moveto({x},{y})")
    
    def click(self, button: str = 'left', count: int = 1):
        """Click mouse button."""
        button_map = {'left': 1, 'right': 2, 'middle': 3}
        btn = button_map.get(button.lower(), 1)
        
        if count == 2:
            self.send_command(f"km.dclick({btn})")
        else:
            self.send_command(f"km.click({btn})")
    
    def button_down(self, button: str = 'left'):
        """Press and hold mouse button."""
        button_map = {'left': 1, 'right': 2, 'middle': 3}
        btn = button_map.get(button.lower(), 1)
        self.send_command(f"km.down({btn})")
    
    def button_up(self, button: str = 'left'):
        """Release mouse button."""
        button_map = {'left': 1, 'right': 2, 'middle': 3}
        btn = button_map.get(button.lower(), 1)
        self.send_command(f"km.up({btn})")
    
    def scroll(self, delta: int):
        """Scroll mouse wheel."""
        self.send_command(f"km.scroll({delta})")
    
    # ========== KMBox Keyboard Commands ==========
    
    def key_press(self, key: str):
        """Press and release a key."""
        self.send_command(f"km.key({key})")
    
    def key_down(self, key: str):
        """Press and hold a key."""
        self.send_command(f"km.keydown({key})")
    
    def key_up(self, key: str):
        """Release a key."""
        self.send_command(f"km.keyup({key})")
    
    # ========================================================================
    # FAST BINARY COMMANDS - Ultra-low latency (~40µs per packet at 2 Mbps)
    # ========================================================================
    # These bypass text parsing for maximum speed. Use for:
    # - High-frequency mouse movement (gaming, drawing)
    # - Rapid key presses
    # - Time-critical automation
    
    def _send_fast_cmd(self, packet: bytes) -> bool:
        """Send an 8-byte fast command packet."""
        if not self.device or len(packet) != FAST_CMD_PACKET_SIZE:
            return False
        
        try:
            # CP2110: first byte is length
            report = bytes([FAST_CMD_PACKET_SIZE]) + packet
            self.device.write(report)
            self._last_activity = time.monotonic()
            return True
        except Exception as e:
            print(f"Fast command error: {e}")
            return False
    
    def fast_move(self, x: int, y: int, buttons: int = 0, wheel: int = 0) -> bool:
        """
        Ultra-fast mouse movement using binary protocol.
        
        This is significantly faster than text commands at lower baud rates.
        
        Args:
            x: Horizontal movement (-32768 to 32767, clamped to -127..127 on device)
            y: Vertical movement (-32768 to 32767, clamped to -127..127 on device)
            buttons: Button bits (FastBtn.LEFT | FastBtn.RIGHT | etc.)
            wheel: Scroll wheel (-128 to 127)
            
        Returns:
            True if sent successfully
            
        Example:
            # Move mouse right and down while holding left button
            kmbox.fast_move(10, 5, buttons=FastBtn.LEFT)
        """
        x = max(-32768, min(32767, x))
        y = max(-32768, min(32767, y))
        wheel = max(-128, min(127, wheel))
        
        packet = bytes([
            FastCmd.MOUSE_MOVE,
            x & 0xFF,          # X low byte
            (x >> 8) & 0xFF,   # X high byte
            y & 0xFF,          # Y low byte
            (y >> 8) & 0xFF,   # Y high byte
            buttons & 0xFF,
            wheel & 0xFF,
            0x00               # Reserved
        ])
        return self._send_fast_cmd(packet)
    
    def fast_click(self, button: int = FastBtn.LEFT, count: int = 1) -> bool:
        """
        Ultra-fast mouse click using binary protocol.
        
        Args:
            button: FastBtn.LEFT, RIGHT, MIDDLE, BACK, or FORWARD
            count: Number of clicks (1-10)
            
        Returns:
            True if sent successfully
        """
        # Map FastBtn enum to button number expected by firmware
        btn_num = {
            FastBtn.LEFT: 1,
            FastBtn.RIGHT: 2,
            FastBtn.MIDDLE: 3,
            FastBtn.BACK: 4,
            FastBtn.FORWARD: 5,
        }.get(button, 1)
        
        packet = bytes([
            FastCmd.MOUSE_CLICK,
            btn_num,
            min(10, max(1, count)),
            0, 0, 0, 0, 0
        ])
        return self._send_fast_cmd(packet)
    
    def fast_key(self, keycode: int, modifiers: int = 0) -> bool:
        """
        Ultra-fast key press using binary protocol.
        
        Args:
            keycode: USB HID keycode (see KeyCode enum)
            modifiers: Modifier bits (see KeyMod enum)
            
        Returns:
            True if sent successfully
            
        Example:
            # Press Ctrl+C
            kmbox.fast_key(KeyCode.C, KeyMod.CTRL_LEFT)
        """
        packet = bytes([
            FastCmd.KEY_PRESS,
            keycode & 0xFF,
            modifiers & 0xFF,
            0, 0, 0, 0, 0
        ])
        return self._send_fast_cmd(packet)
    
    def fast_key_combo(self, keys: list, modifiers: int = 0) -> bool:
        """
        Ultra-fast key combination (up to 4 keys simultaneously).
        
        Args:
            keys: List of up to 4 USB HID keycodes
            modifiers: Modifier bits (see KeyMod enum)
            
        Returns:
            True if sent successfully
            
        Example:
            # Press Ctrl+Shift+Escape (Task Manager)
            kmbox.fast_key_combo([KeyCode.ESCAPE], KeyMod.CTRL_LEFT | KeyMod.SHIFT_LEFT)
        """
        # Pad to 4 keys
        key_bytes = (list(keys) + [0, 0, 0, 0])[:4]
        
        packet = bytes([
            FastCmd.KEY_COMBO,
            key_bytes[0] & 0xFF,
            key_bytes[1] & 0xFF,
            key_bytes[2] & 0xFF,
            key_bytes[3] & 0xFF,
            modifiers & 0xFF,
            0, 0
        ])
        return self._send_fast_cmd(packet)
    
    def fast_multi_move(self, moves: list) -> bool:
        """
        Send up to 3 mouse movements in a single packet!
        
        This is the fastest way to send multiple small movements.
        Each move is limited to -127..127 range (8-bit signed).
        
        Args:
            moves: List of (x, y) tuples, up to 3 moves
            
        Returns:
            True if sent successfully
            
        Example:
            # Three quick movements in one packet
            kmbox.fast_multi_move([(10, 0), (0, 10), (-10, -10)])
        """
        # Pad to 3 moves
        padded = (list(moves) + [(0, 0), (0, 0), (0, 0)])[:3]
        
        packet = bytes([
            FastCmd.MULTI_MOVE,
            max(-127, min(127, padded[0][0])) & 0xFF,  # X1
            max(-127, min(127, padded[0][1])) & 0xFF,  # Y1
            max(-127, min(127, padded[1][0])) & 0xFF,  # X2
            max(-127, min(127, padded[1][1])) & 0xFF,  # Y2
            max(-127, min(127, padded[2][0])) & 0xFF,  # X3
            max(-127, min(127, padded[2][1])) & 0xFF,  # Y3
            0x00  # Flags (reserved)
        ])
        return self._send_fast_cmd(packet)
    
    def fast_ping(self) -> bool:
        """
        Send a fast binary ping (faster than text KMBOX_PING).
        
        Returns:
            True if ping response received
        """
        packet = bytes([FastCmd.PING] + [0] * 7)
        if not self._send_fast_cmd(packet):
            return False
        
        # Wait for response
        try:
            data = self.device.read(64, 50)  # 50ms timeout
            if data and len(data) > 1:
                # Check for RESPONSE command in data
                if data[1] == FastCmd.RESPONSE:
                    return True
        except:
            pass
        return False
    
    # ========== High-Performance Batch Operations ==========
    
    def fast_move_smooth(self, x: int, y: int, steps: int = 10, delay_ms: float = 0.5):
        """
        Smooth mouse movement using fast binary protocol.
        
        Much faster than move_smooth() - uses binary packets instead of text.
        At 2 Mbps, can achieve 2000+ updates per second depending on PC/bridge setup.
        
        Args:
            x: Total X movement
            y: Total Y movement  
            steps: Number of interpolation steps
            delay_ms: Delay between steps in milliseconds (can be < 1ms)
        """
        step_x = x / steps
        step_y = y / steps
        
        for i in range(steps):
            dx = int(step_x * (i + 1)) - int(step_x * i)
            dy = int(step_y * (i + 1)) - int(step_y * i)
            if dx != 0 or dy != 0:
                self.fast_move(dx, dy)
            if delay_ms > 0:
                time.sleep(delay_ms / 1000)
    
    def fast_move_batch(self, moves: list, delay_ms: float = 0):
        """
        Send a batch of mouse movements as fast as possible.
        
        Uses multi-move packets when possible for 3x efficiency.
        
        Args:
            moves: List of (x, y) tuples
            delay_ms: Optional delay between packets
        """
        # Group into sets of 3 for multi-move packets
        i = 0
        while i < len(moves):
            remaining = len(moves) - i
            
            if remaining >= 3:
                # Send 3 moves in one packet
                self.fast_multi_move(moves[i:i+3])
                i += 3
            else:
                # Send remaining moves individually
                for j in range(i, len(moves)):
                    self.fast_move(moves[j][0], moves[j][1])
                break
            
            if delay_ms > 0:
                time.sleep(delay_ms / 1000)

    # ========== Smooth Injection System ==========
    # 
    # The smooth injection system provides ultra-smooth mouse movement
    # that blends seamlessly with physical mouse passthrough. It features:
    # - Sub-pixel precision (16.16 fixed-point internally)
    # - Velocity tracking - injected moves match current mouse speed
    # - Temporal spreading - large moves spread across multiple frames
    # - Per-frame rate limiting - prevents jarring jumps
    #
    # Modes:
    # - IMMEDIATE: Direct injection (fastest, may cause jumps)
    # - SMOOTH: Rate-limited spreading (natural feel)
    # - VELOCITY_MATCHED: Adapts to current mouse speed
    # - MICRO: Sub-pixel adjustments (anti-recoil, aim correction)

    def smooth_move(self, x: int, y: int, mode: int = 1) -> bool:
        """
        Queue a smooth mouse movement for injection.
        
        The movement is blended with physical mouse passthrough
        automatically. Large movements are spread across frames
        for natural feel.
        
        Args:
            x: X movement in pixels (can be larger than 127)
            y: Y movement in pixels (can be larger than 127)
            mode: Injection mode (0=IMMEDIATE, 1=SMOOTH, 2=VELOCITY_MATCHED, 3=MICRO)
            
        Returns:
            True if command sent successfully
            
        Example:
            # Smooth 100 pixel move (will be spread across ~6 frames)
            kmbox.smooth_move(100, 50, mode=SmoothMode.SMOOTH)
            
            # Micro-adjustment (sub-pixel precision)
            kmbox.smooth_move(1, 0, mode=SmoothMode.MICRO)
        """
        # Clamp to int16 range
        x = max(-32768, min(32767, int(x)))
        y = max(-32768, min(32767, int(y)))
        
        # Pack as signed 16-bit little-endian
        x_lo = x & 0xFF
        x_hi = (x >> 8) & 0xFF
        y_lo = y & 0xFF
        y_hi = (y >> 8) & 0xFF
        
        packet = bytes([
            FastCmd.SMOOTH_MOVE,
            x_lo, x_hi,
            y_lo, y_hi,
            mode & 0xFF,
            0x00, 0x00
        ])
        return self._send_fast_cmd(packet)
    
    def smooth_config(self, max_per_frame: int = 16, velocity_matching: bool = True) -> bool:
        """
        Configure the smooth injection system.
        
        Args:
            max_per_frame: Maximum pixels to inject per frame (1-127)
                           Higher = faster but potentially jumpy
                           Lower = smoother but more latency
            velocity_matching: Enable velocity-aware injection
                              True = adapts to mouse speed
                              False = constant rate
                              
        Returns:
            True if command sent successfully
            
        Example:
            # High-speed configuration for fast games
            kmbox.smooth_config(max_per_frame=32, velocity_matching=True)
            
            # Ultra-smooth configuration for precision
            kmbox.smooth_config(max_per_frame=8, velocity_matching=False)
        """
        packet = bytes([
            FastCmd.SMOOTH_CONFIG,
            max_per_frame & 0x7F,  # Limit to 127 max
            1 if velocity_matching else 0,
            0x00, 0x00, 0x00, 0x00, 0x00
        ])
        return self._send_fast_cmd(packet)
    
    def smooth_clear(self) -> bool:
        """
        Clear the smooth injection queue.
        
        Use this to abort any pending smooth movements.
        
        Returns:
            True if command sent successfully
        """
        packet = bytes([FastCmd.SMOOTH_CLEAR] + [0x00] * 7)
        return self._send_fast_cmd(packet)
    
    def smooth_move_trajectory(self, points: list, mode: int = 1) -> int:
        """
        Queue a series of smooth movements forming a trajectory.
        
        Each point is queued as a smooth movement. The firmware
        will process them frame-by-frame for natural motion.
        
        Args:
            points: List of (x, y) tuples
            mode: Injection mode for all points
            
        Returns:
            Number of points successfully queued
            
        Example:
            # Draw a small circle
            import math
            points = [(int(10*math.cos(a)), int(10*math.sin(a))) 
                      for a in [i*0.1 for i in range(63)]]
            kmbox.smooth_move_trajectory(points)
        """
        count = 0
        for x, y in points:
            if self.smooth_move(x, y, mode):
                count += 1
        return count
    
    def smooth_aim_correct(self, x: float, y: float) -> bool:
        """
        Apply a micro-adjustment for aim correction.
        
        Uses MICRO mode for sub-pixel precision. Ideal for
        anti-recoil or fine aim adjustment scripts.
        
        Args:
            x: X adjustment (can be fractional, accumulated internally)
            y: Y adjustment (can be fractional, accumulated internally)
            
        Returns:
            True if command sent successfully
            
        Example:
            # Apply small recoil compensation
            kmbox.smooth_aim_correct(-0.5, -2.0)
        """
        # Convert to integer (sub-pixel is handled by firmware)
        return self.smooth_move(int(x), int(y), mode=SmoothMode.MICRO)

    # ========================================================================
    # Timed/Synchronized Commands (Clock-Aligned Execution)
    # ========================================================================
    # These methods support clock-aligned command execution for deterministic
    # timing. Commands can specify a target execution time relative to the
    # firmware's internal clock.
    
    def timed_move(self, x: int, y: int, delay_us: int = 0, 
                   mode: int = SmoothMode.IMMEDIATE) -> bool:
        """
        Queue a timed mouse movement for execution at a specific time offset.
        
        The firmware will hold the movement until the specified delay has
        elapsed from receipt, allowing precise timing coordination.
        
        Args:
            x: X movement in pixels
            y: Y movement in pixels
            delay_us: Delay in microseconds before execution (0-65535)
                      0 = immediate execution
            mode: Injection mode (0=IMMEDIATE, 1=SMOOTH, 2=VELOCITY_MATCHED, 3=MICRO)
            
        Returns:
            True if command sent successfully
            
        Example:
            # Schedule move for 5ms from now
            kmbox.timed_move(10, 5, delay_us=5000)
            
            # Immediate timed move (same as smooth_move with IMMEDIATE)
            kmbox.timed_move(10, 5, delay_us=0)
        """
        # Clamp to int16 range
        x = max(-32768, min(32767, int(x)))
        y = max(-32768, min(32767, int(y)))
        
        # Pack as signed 16-bit little-endian
        x_lo = x & 0xFF
        x_hi = (x >> 8) & 0xFF
        y_lo = y & 0xFF
        y_hi = (y >> 8) & 0xFF
        
        # Time in 16-bit microseconds
        delay_us = max(0, min(65535, int(delay_us)))
        time_lo = delay_us & 0xFF
        time_hi = (delay_us >> 8) & 0xFF
        
        packet = bytes([
            FastCmd.TIMED_MOVE,
            x_lo, x_hi,
            y_lo, y_hi,
            time_lo, time_hi,
            mode & 0xFF
        ])
        return self._send_fast_cmd(packet)
    
    def sync_clock(self, seq_num: int = 0) -> Optional[int]:
        """
        Synchronize clocks with firmware and measure round-trip time.
        
        Sends a sync request and waits for a response containing the
        firmware's timestamp. This can be used to:
        1. Measure communication latency
        2. Align PC clock with firmware clock
        3. Coordinate timed command sequences
        
        Args:
            seq_num: Sequence number (0-255) for matching request/response
            
        Returns:
            Round-trip time in microseconds, or None on timeout
            
        Example:
            # Measure latency
            rtt = kmbox.sync_clock()
            if rtt:
                print(f"Round-trip: {rtt}us, one-way: ~{rtt//2}us")
        """
        import time as _time
        
        # Capture send time
        send_time = _time.perf_counter_ns() // 1000  # Convert to microseconds
        
        # Pack timestamp into sync packet
        time_bytes = struct.pack('<I', send_time & 0xFFFFFFFF)
        
        packet = bytes([
            FastCmd.SYNC,
            seq_num & 0xFF,
            time_bytes[0], time_bytes[1], time_bytes[2], time_bytes[3],
            0x00, 0x00
        ])
        
        if not self._send_fast_cmd(packet):
            return None
        
        # Wait for response (fast ping style)
        # Response will be: [0xFF][seq_num][fw_time0-3][0x00][0x00]
        start = _time.perf_counter()
        timeout_sec = 0.1  # 100ms timeout
        
        while (_time.perf_counter() - start) < timeout_sec:
            data = self._read_raw(8, timeout_ms=10)
            if data and len(data) >= 6:
                if data[0] == FastCmd.RESPONSE and data[1] == seq_num:
                    recv_time = _time.perf_counter_ns() // 1000
                    rtt = recv_time - send_time
                    # Extract firmware timestamp if needed
                    # fw_time = struct.unpack('<I', bytes(data[2:6]))[0]
                    return rtt
            _time.sleep(0.0001)  # 100us
        
        return None
    
    def sync_batch_start(self) -> int:
        """
        Start a synchronized command batch.
        
        Returns a reference timestamp that can be used for scheduling
        multiple timed commands relative to a single point in time.
        
        Returns:
            Reference timestamp in microseconds
            
        Example:
            t0 = kmbox.sync_batch_start()
            kmbox.timed_move(10, 0, delay_us=0)      # Execute immediately
            kmbox.timed_move(10, 0, delay_us=8000)   # 8ms later (1 HID frame)
            kmbox.timed_move(10, 0, delay_us=16000)  # 16ms later
        """
        import time as _time
        return _time.perf_counter_ns() // 1000
    
    def schedule_moves(self, moves: list, interval_us: int = 1000) -> int:
        """
        Schedule a sequence of moves with precise timing intervals.
        
        Each move is scheduled to execute at a fixed interval from
        the previous one, creating smooth, timed motion.
        
        Args:
            moves: List of (x, y) tuples
            interval_us: Time between moves in microseconds (default 1ms)
            
        Returns:
            Number of moves successfully scheduled
            
        Example:
            # Schedule 10 moves at 1ms intervals
            moves = [(5, 0)] * 10  # 5 pixels right, 10 times
            kmbox.schedule_moves(moves, interval_us=1000)
        """
        count = 0
        delay = 0
        
        for x, y in moves:
            if self.timed_move(x, y, delay_us=delay, mode=SmoothMode.IMMEDIATE):
                count += 1
            delay += interval_us
            
            # Cap at max timing value
            if delay > 65535:
                break
        
        return count
    
    def fast_ping_rtt(self) -> Optional[int]:
        """
        Measure fast command round-trip time using ping/response.
        
        This is a lightweight alternative to sync_clock for
        just measuring latency without clock synchronization.
        
        Returns:
            Round-trip time in microseconds, or None on timeout
        """
        import time as _time
        
        send_time = _time.perf_counter_ns() // 1000
        
        packet = bytes([FastCmd.PING] + [0x00] * 7)
        
        if not self._send_fast_cmd(packet):
            return None
        
        # Wait for response
        start = _time.perf_counter()
        timeout_sec = 0.1
        
        while (_time.perf_counter() - start) < timeout_sec:
            data = self._read_raw(8, timeout_ms=10)
            if data and len(data) >= 1 and data[0] == FastCmd.RESPONSE:
                recv_time = _time.perf_counter_ns() // 1000
                return recv_time - send_time
            _time.sleep(0.0001)
        
        return None

    def move_smooth(self, x: int, y: int, steps: int = 10, delay_ms: float = 1):
        """Smooth mouse movement with interpolation."""
        step_x = x / steps
        step_y = y / steps
        
        for i in range(steps):
            dx = int(step_x * (i + 1)) - int(step_x * i)
            dy = int(step_y * (i + 1)) - int(step_y * i)
            if dx != 0 or dy != 0:
                self.move(dx, dy)
            time.sleep(delay_ms / 1000)
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


def list_cp2110_devices():
    """List all connected CP2110 devices."""
    print("Scanning for CP2110 devices...")
    devices = hid.enumerate(CP2110_VID, CP2110_PID)
    
    if not devices:
        print("No CP2110 devices found.")
        print("\nTroubleshooting:")
        print("  1. Ensure USB UART5 Click is connected")
        print("  2. Check USB connection to PC")
        print("  3. On Linux, you may need udev rules for HID access")
        return []
    
    for i, dev in enumerate(devices):
        print(f"\nDevice {i}:")
        print(f"  VID:PID = {dev['vendor_id']:04X}:{dev['product_id']:04X}")
        print(f"  Manufacturer: {dev.get('manufacturer_string', 'N/A')}")
        print(f"  Product: {dev.get('product_string', 'N/A')}")
        print(f"  Serial: {dev.get('serial_number', 'N/A')}")
        print(f"  Path: {dev.get('path', 'N/A')}")
    
    return devices


def main():
    """Example usage and device test."""
    import argparse
    
    parser = argparse.ArgumentParser(description='CP2110 KMBox Client')
    parser.add_argument('--list', action='store_true', help='List CP2110 devices')
    parser.add_argument('--test', action='store_true', help='Run connection test')
    parser.add_argument('--no-handshake', action='store_true', 
                        help='Skip handshake (for backwards compatibility)')
    parser.add_argument('--interactive', '-i', action='store_true', 
                        help='Interactive command mode')
    args = parser.parse_args()
    
    if args.list:
        list_cp2110_devices()
        return
    
    if args.test:
        print("Testing CP2110 connection with handshake...")
        
        def on_state_change(state):
            state_colors = {
                ConnectionState.DISCONNECTED: "🔴",
                ConnectionState.CONNECTING: "🟡",
                ConnectionState.CONNECTED: "🟢",
                ConnectionState.ACTIVE: "🔵",
            }
            print(f"  State: {state_colors.get(state, '⚪')} {state.name}")
        
        kmbox = CP2110KMBox()
        kmbox.set_state_callback(on_state_change)
        
        if kmbox.connect(handshake=not args.no_handshake):
            print(f"\nConnection test PASSED")
            print(f"Firmware version: {kmbox.firmware_version}")
            
            status = kmbox.get_status()
            if status:
                print(f"Status: {status}")
            
            print("\nSending test movement...")
            kmbox.move(1, 0)
            time.sleep(0.1)
            kmbox.move(-1, 0)
            print("Test complete!")
            
            kmbox.disconnect()
        else:
            print("Connection test FAILED")
        return
    
    if args.interactive:
        print("CP2110 KMBox Interactive Mode")
        print("=" * 50)
        print("Connection states shown as LED colors:")
        print("  🔵 Light Blue (breathing) = Waiting")
        print("  🟡 Yellow = Connecting")
        print("  🟢 Green = Connected")
        print("  🔵 Cyan = Active")
        print("  🟠 Orange-Red (breathing) = Disconnected")
        print("=" * 50)
        print("Commands: move X Y, click [left|right], scroll N, status, quit")
        print("-" * 50)
        
        def on_state_change(state):
            indicators = {
                ConnectionState.DISCONNECTED: "🟠 DISCONNECTED",
                ConnectionState.CONNECTING: "🟡 CONNECTING",
                ConnectionState.CONNECTED: "🟢 CONNECTED",
                ConnectionState.ACTIVE: "🔵 ACTIVE",
            }
            print(f"\n[{indicators.get(state, state.name)}]")
        
        kmbox = CP2110KMBox()
        kmbox.set_state_callback(on_state_change)
        
        if not kmbox.connect(handshake=not args.no_handshake):
            print("Failed to connect. Exiting.")
            return
        
        print(f"\nFirmware version: {kmbox.firmware_version}")
        
        while True:
            try:
                cmd = input("> ").strip().lower()
                if not cmd:
                    continue
                if cmd in ('quit', 'exit', 'q'):
                    break
                
                parts = cmd.split()
                if parts[0] == 'move' and len(parts) == 3:
                    kmbox.move(int(parts[1]), int(parts[2]))
                elif parts[0] == 'click':
                    button = parts[1] if len(parts) > 1 else 'left'
                    kmbox.click(button)
                elif parts[0] == 'scroll' and len(parts) == 2:
                    kmbox.scroll(int(parts[1]))
                elif parts[0] == 'status':
                    status = kmbox.get_status()
                    print(f"Firmware status: {status}")
                elif parts[0] == 'reconnect':
                    kmbox.reconnect()
                elif parts[0] == 'raw':
                    raw_cmd = ' '.join(parts[1:])
                    response = kmbox.send_command(raw_cmd, wait_response=True)
                    if response:
                        print(f"Response: {response}")
                else:
                    print(f"Unknown command: {cmd}")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except ValueError as e:
                print(f"Invalid value: {e}")
        
        kmbox.disconnect()
        return
    
    # Default: show help
    parser.print_help()


if __name__ == '__main__':
    main()
