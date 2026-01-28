#!/usr/bin/env python3
"""
MCP2210 KMBox Client
High-performance HID-based SPI communication with KMBox via MCP2210 USB-SPI Click

The MCP2210 uses USB HID for SPI transfers, providing:
- Up to 12 Mbps SPI speed
- Low, predictable latency  
- No CDC/ACM driver overhead
- 9 GPIO pins for additional control

Requirements:
    pip install hidapi

Usage:
    from mcp2210_kmbox import MCP2210KMBox
    
    with MCP2210KMBox() as kmbox:
        kmbox.move(10, 20)
        kmbox.click('left')
"""

import hid
import time
import struct
import threading
from typing import Optional, Tuple, List
from enum import IntEnum
from dataclasses import dataclass

# Microchip MCP2210 USB IDs
MCP2210_VID = 0x04D8
MCP2210_PID = 0x00DE

# ============================================================================
# MCP2210 HID Command Codes (from datasheet)
# ============================================================================

class CmdCode(IntEnum):
    """MCP2210 HID command codes."""
    # Status/Configuration
    GET_CHIP_STATUS = 0x10
    CANCEL_SPI_TRANSFER = 0x11
    
    # GPIO
    GET_GPIO_SETTING = 0x20
    SET_GPIO_SETTING = 0x21
    GET_GPIO_PIN_VAL = 0x30
    SET_GPIO_PIN_VAL = 0x31
    GET_GPIO_PIN_DIR = 0x32
    SET_GPIO_PIN_DIR = 0x33
    
    # SPI Transfer
    SPI_TRANSFER = 0x42
    
    # SPI Settings (Current/Power-up/NVRAM)
    GET_SPI_SETTING = 0x41
    SET_SPI_SETTING = 0x40
    
    # NVRAM Access
    GET_NVRAM_SETTING = 0x61
    SET_NVRAM_SETTING = 0x60
    
    # Chip Settings
    GET_CHIP_SETTING = 0x20
    SET_CHIP_SETTING = 0x21


class SPIMode(IntEnum):
    """SPI clock polarity and phase modes."""
    MODE_0 = 0x00  # CPOL=0, CPHA=0 (most common)
    MODE_1 = 0x01  # CPOL=0, CPHA=1
    MODE_2 = 0x02  # CPOL=1, CPHA=0
    MODE_3 = 0x03  # CPOL=1, CPHA=1


class GPIODesignation(IntEnum):
    """GPIO pin function designation."""
    GPIO = 0x00
    CHIP_SELECT = 0x01
    DEDICATED_FUNCTION = 0x02


class GPIODirection(IntEnum):
    """GPIO pin direction."""
    OUTPUT = 0x00
    INPUT = 0x01


class SPIStatus(IntEnum):
    """SPI transfer status codes."""
    FINISHED = 0x10
    STARTED = 0x20  
    NOT_FINISHED = 0x30
    
    # Errors
    SPI_NOT_OWNER = 0xF7
    SPI_BUSY = 0xF8


# ============================================================================
# Fast Binary Command Protocol (same as CP2110 version)
# ============================================================================

class FastCmd(IntEnum):
    """Fast binary command types (8-byte fixed packets)."""
    MOUSE_MOVE = 0x01
    MOUSE_CLICK = 0x02
    KEY_PRESS = 0x03
    KEY_COMBO = 0x04
    MULTI_MOVE = 0x05
    MOUSE_ABS = 0x06
    SMOOTH_MOVE = 0x07
    SMOOTH_CONFIG = 0x08
    SMOOTH_CLEAR = 0x09
    TIMED_MOVE = 0x0A
    SYNC = 0x0B
    PING = 0xFE
    RESPONSE = 0xFF


class FastBtn(IntEnum):
    """Button bit flags."""
    LEFT = 0x01
    RIGHT = 0x02
    MIDDLE = 0x04
    BACK = 0x08
    FORWARD = 0x10


class KeyCode(IntEnum):
    """USB HID keyboard keycodes (common ones)."""
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
    GUI_LEFT = 0x08
    CTRL_RIGHT = 0x10
    SHIFT_RIGHT = 0x20
    ALT_RIGHT = 0x40
    GUI_RIGHT = 0x80


FAST_CMD_PACKET_SIZE = 8


@dataclass
class SPISettings:
    """SPI transfer settings for MCP2210."""
    bit_rate: int = 1000000  # 1 MHz default
    idle_cs_value: int = 0x1FF  # All CS high when idle (9 pins)
    active_cs_value: int = 0x1FE  # CS0 low when active
    cs_to_data_delay: int = 0  # Delay in 100µs units
    data_to_cs_delay: int = 0
    inter_byte_delay: int = 0
    bytes_per_transaction: int = 8  # Match our packet size
    spi_mode: SPIMode = SPIMode.MODE_0
    
    def to_bytes(self) -> bytes:
        """Convert to MCP2210 command format."""
        return bytes([
            self.bit_rate & 0xFF,
            (self.bit_rate >> 8) & 0xFF,
            (self.bit_rate >> 16) & 0xFF,
            (self.bit_rate >> 24) & 0xFF,
            self.idle_cs_value & 0xFF,
            (self.idle_cs_value >> 8) & 0x01,
            self.active_cs_value & 0xFF,
            (self.active_cs_value >> 8) & 0x01,
            self.cs_to_data_delay & 0xFF,
            (self.cs_to_data_delay >> 8) & 0xFF,
            self.data_to_cs_delay & 0xFF,
            (self.data_to_cs_delay >> 8) & 0xFF,
            self.inter_byte_delay & 0xFF,
            (self.inter_byte_delay >> 8) & 0xFF,
            self.bytes_per_transaction & 0xFF,
            (self.bytes_per_transaction >> 8) & 0xFF,
            self.spi_mode,
        ])


@dataclass  
class ChipStatus:
    """MCP2210 chip status."""
    spi_bus_release_status: int
    spi_owner_status: int
    password_attempts: int
    password_guessed: bool


class MCP2210KMBox:
    """High-performance KMBox client using MCP2210 USB-to-SPI bridge."""
    
    def __init__(self, vid: int = MCP2210_VID, pid: int = MCP2210_PID):
        self.vid = vid
        self.pid = pid
        self.device: Optional[hid.device] = None
        self._lock = threading.Lock()
        self._connected = False
        self._spi_settings = SPISettings()
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False
    
    @property
    def is_connected(self) -> bool:
        return self._connected
    
    def connect(self, bit_rate: int = 1000000) -> bool:
        """
        Connect to MCP2210 and configure SPI.
        
        Args:
            bit_rate: SPI clock rate in Hz (max 12 MHz)
            
        Returns:
            True if connection successful
        """
        try:
            self.device = hid.device()
            self.device.open(self.vid, self.pid)
            self.device.set_nonblocking(1)
            
            print(f"MCP2210 opened (VID={self.vid:04X}, PID={self.pid:04X})")
            
            # Configure SPI settings
            self._spi_settings.bit_rate = bit_rate
            self._configure_spi()
            
            # Configure GPIO for CS0 as chip select
            self._configure_gpio()
            
            self._connected = True
            print(f"SPI configured: {bit_rate/1000000:.1f} MHz, Mode 0")
            
            return True
            
        except Exception as e:
            print(f"Failed to connect to MCP2210: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from MCP2210."""
        if self.device:
            try:
                self.device.close()
            except:
                pass
            self.device = None
        self._connected = False
        print("Disconnected from MCP2210")
    
    # ========================================================================
    # MCP2210 Low-Level Commands
    # ========================================================================
    
    def _send_command(self, cmd: bytes) -> Optional[bytes]:
        """Send command and get response."""
        if not self.device:
            return None
        
        # Pad to 64 bytes (MCP2210 HID report size)
        padded = bytes(cmd) + bytes(64 - len(cmd))
        
        try:
            # Write command
            self.device.write(bytes([0x00]) + padded)  # Report ID 0
            
            # Read response with timeout
            for _ in range(100):  # 100ms timeout
                response = self.device.read(64, 10)
                if response:
                    return bytes(response)
                time.sleep(0.001)
                
        except Exception as e:
            print(f"Command error: {e}")
        
        return None
    
    def _configure_spi(self):
        """Configure SPI transfer settings."""
        cmd = bytes([CmdCode.SET_SPI_SETTING, 0x00, 0x00, 0x00]) + self._spi_settings.to_bytes()
        response = self._send_command(cmd)
        if response and response[1] == 0x00:
            print("SPI settings configured")
        else:
            print(f"SPI config failed: {response[:4] if response else 'no response'}")
    
    def _configure_gpio(self):
        """Configure GPIO pins - GP0 as CS, others as GPIO."""
        # Set GP0 as chip select, GP1-GP8 as GPIO inputs
        # Format: [cmd, reserved, reserved, reserved, GP0-GP8 designations...]
        gpio_designations = [
            GPIODesignation.CHIP_SELECT,  # GP0 = CS
            GPIODesignation.GPIO,  # GP1
            GPIODesignation.GPIO,  # GP2  
            GPIODesignation.GPIO,  # GP3
            GPIODesignation.GPIO,  # GP4
            GPIODesignation.GPIO,  # GP5
            GPIODesignation.GPIO,  # GP6
            GPIODesignation.GPIO,  # GP7
            GPIODesignation.GPIO,  # GP8
        ]
        
        cmd = bytes([CmdCode.SET_GPIO_SETTING, 0x00, 0x00, 0x00]) + bytes(gpio_designations)
        response = self._send_command(cmd)
        if response and response[1] == 0x00:
            print("GPIO configured: GP0=CS, GP1-8=GPIO")
    
    def get_chip_status(self) -> Optional[ChipStatus]:
        """Get MCP2210 chip status."""
        response = self._send_command(bytes([CmdCode.GET_CHIP_STATUS]))
        if response and len(response) >= 6:
            return ChipStatus(
                spi_bus_release_status=response[2],
                spi_owner_status=response[3],
                password_attempts=response[4],
                password_guessed=bool(response[5])
            )
        return None
    
    def get_gpio(self) -> Optional[int]:
        """Read GPIO pin values (GP0-GP8 as 9-bit value)."""
        response = self._send_command(bytes([CmdCode.GET_GPIO_PIN_VAL]))
        if response and len(response) >= 6:
            return response[4] | (response[5] << 8)
        return None
    
    def set_gpio(self, value: int, mask: int = 0x1FF):
        """
        Set GPIO output values.
        
        Args:
            value: Bit values for GP0-GP8 (1=high, 0=low)
            mask: Which bits to update
        """
        # Read current value
        current = self.get_gpio() or 0
        new_value = (current & ~mask) | (value & mask)
        
        cmd = bytes([
            CmdCode.SET_GPIO_PIN_VAL, 0x00, 0x00, 0x00,
            new_value & 0xFF,
            (new_value >> 8) & 0x01
        ])
        self._send_command(cmd)
    
    # ========================================================================
    # SPI Transfer
    # ========================================================================
    
    def spi_transfer(self, data: bytes) -> Optional[bytes]:
        """
        Perform SPI transfer.
        
        Args:
            data: Bytes to send (max 60 per transfer)
            
        Returns:
            Received bytes, or None on error
        """
        if not self.device or len(data) > 60:
            return None
        
        with self._lock:
            # Build transfer command
            cmd = bytes([CmdCode.SPI_TRANSFER, len(data), 0x00, 0x00]) + data
            
            response = self._send_command(cmd)
            if not response:
                return None
            
            status = response[1]
            
            # Check for errors
            if status == SPIStatus.SPI_BUSY:
                print("SPI busy")
                return None
            elif status == SPIStatus.SPI_NOT_OWNER:
                print("SPI not owner")
                return None
            
            # If transfer started but not finished, keep reading
            rx_data = bytearray()
            
            if status in (SPIStatus.STARTED, SPIStatus.NOT_FINISHED):
                # Data in progress, get received bytes
                rx_len = response[2]
                if rx_len > 0:
                    rx_data.extend(response[4:4+rx_len])
                
                # Keep polling until finished
                while status != SPIStatus.FINISHED:
                    response = self._send_command(bytes([CmdCode.SPI_TRANSFER, 0, 0, 0]))
                    if not response:
                        break
                    status = response[1]
                    rx_len = response[2]
                    if rx_len > 0:
                        rx_data.extend(response[4:4+rx_len])
            
            elif status == SPIStatus.FINISHED:
                rx_len = response[2]
                if rx_len > 0:
                    rx_data.extend(response[4:4+rx_len])
            
            return bytes(rx_data) if rx_data else bytes(len(data))
    
    # ========================================================================
    # Fast Binary Commands (via SPI)
    # ========================================================================
    
    def _send_fast_cmd(self, cmd_type: int, *payload) -> bool:
        """Send 8-byte fast command over SPI."""
        packet = bytes([cmd_type] + list(payload))
        packet = packet.ljust(FAST_CMD_PACKET_SIZE, b'\x00')
        
        response = self.spi_transfer(packet)
        return response is not None
    
    def move(self, x: int, y: int, buttons: int = 0, wheel: int = 0) -> bool:
        """
        Send relative mouse movement.
        
        Args:
            x: X movement (-32768 to 32767)
            y: Y movement (-32768 to 32767)
            buttons: Button state (FastBtn flags)
            wheel: Scroll wheel delta (-128 to 127)
        """
        x = max(-32768, min(32767, x))
        y = max(-32768, min(32767, y))
        wheel = max(-128, min(127, wheel))
        
        return self._send_fast_cmd(
            FastCmd.MOUSE_MOVE,
            x & 0xFF, (x >> 8) & 0xFF,
            y & 0xFF, (y >> 8) & 0xFF,
            buttons & 0xFF,
            wheel & 0xFF,
            0x00
        )
    
    def click(self, button: str = 'left', count: int = 1) -> bool:
        """
        Perform mouse click.
        
        Args:
            button: 'left', 'right', or 'middle'
            count: Number of clicks
        """
        btn_map = {
            'left': FastBtn.LEFT,
            'right': FastBtn.RIGHT,
            'middle': FastBtn.MIDDLE,
            'back': FastBtn.BACK,
            'forward': FastBtn.FORWARD,
        }
        btn = btn_map.get(button.lower(), FastBtn.LEFT)
        
        return self._send_fast_cmd(
            FastCmd.MOUSE_CLICK,
            btn, count, 0, 0, 0, 0, 0
        )
    
    def button_down(self, button: str = 'left') -> bool:
        """Press and hold mouse button."""
        btn_map = {
            'left': FastBtn.LEFT,
            'right': FastBtn.RIGHT,
            'middle': FastBtn.MIDDLE,
        }
        btn = btn_map.get(button.lower(), FastBtn.LEFT)
        return self.move(0, 0, buttons=btn)
    
    def button_up(self) -> bool:
        """Release all mouse buttons."""
        return self.move(0, 0, buttons=0)
    
    def scroll(self, delta: int) -> bool:
        """Scroll wheel movement."""
        return self.move(0, 0, wheel=delta)
    
    def move_smooth(self, x: int, y: int, steps: int = 10, delay_ms: int = 1) -> bool:
        """
        Smooth interpolated mouse movement.
        
        Args:
            x: Total X movement
            y: Total Y movement
            steps: Number of steps to divide movement into
            delay_ms: Delay between steps in milliseconds
        """
        if steps < 1:
            steps = 1
        
        dx = x / steps
        dy = y / steps
        accum_x = 0.0
        accum_y = 0.0
        
        for i in range(steps):
            accum_x += dx
            accum_y += dy
            
            move_x = int(accum_x)
            move_y = int(accum_y)
            
            if move_x != 0 or move_y != 0:
                if not self.move(move_x, move_y):
                    return False
                accum_x -= move_x
                accum_y -= move_y
            
            if delay_ms > 0 and i < steps - 1:
                time.sleep(delay_ms / 1000.0)
        
        return True
    
    def key_press(self, key: str, modifiers: int = 0) -> bool:
        """
        Press and release a key.
        
        Args:
            key: Key character or KeyCode name
            modifiers: KeyMod flags
        """
        keycode = self._get_keycode(key)
        if keycode is None:
            return False
        
        return self._send_fast_cmd(
            FastCmd.KEY_PRESS,
            keycode, modifiers, 0, 0, 0, 0, 0
        )
    
    def key_combo(self, keys: List[str], modifiers: int = 0) -> bool:
        """
        Press a key combination (up to 4 keys).
        
        Args:
            keys: List of key characters or KeyCode names
            modifiers: KeyMod flags
        """
        keycodes = [self._get_keycode(k) or 0 for k in keys[:4]]
        while len(keycodes) < 4:
            keycodes.append(0)
        
        return self._send_fast_cmd(
            FastCmd.KEY_COMBO,
            keycodes[0], keycodes[1], keycodes[2], keycodes[3],
            modifiers, 0, 0
        )
    
    def _get_keycode(self, key: str) -> Optional[int]:
        """Convert key string to USB HID keycode."""
        if isinstance(key, int):
            return key
        
        key = key.upper()
        
        # Single character
        if len(key) == 1:
            if 'A' <= key <= 'Z':
                return KeyCode.A + (ord(key) - ord('A'))
            if '1' <= key <= '9':
                return KeyCode.NUM_1 + (ord(key) - ord('1'))
            if key == '0':
                return KeyCode.NUM_0
            if key == ' ':
                return KeyCode.SPACE
        
        # Named keys
        key_map = {
            'ENTER': KeyCode.ENTER,
            'RETURN': KeyCode.ENTER,
            'ESC': KeyCode.ESCAPE,
            'ESCAPE': KeyCode.ESCAPE,
            'BACKSPACE': KeyCode.BACKSPACE,
            'TAB': KeyCode.TAB,
            'SPACE': KeyCode.SPACE,
            'F1': KeyCode.F1,
            'F2': KeyCode.F2,
            'F3': KeyCode.F3,
            'F4': KeyCode.F4,
            'F5': KeyCode.F5,
            'F6': KeyCode.F6,
            'F7': KeyCode.F7,
            'F8': KeyCode.F8,
            'F9': KeyCode.F9,
            'F10': KeyCode.F10,
            'F11': KeyCode.F11,
            'F12': KeyCode.F12,
        }
        
        return key_map.get(key)
    
    def ping(self) -> Optional[float]:
        """
        Send ping and measure round-trip time.
        
        Returns:
            RTT in milliseconds, or None on failure
        """
        start = time.perf_counter()
        
        packet = bytes([FastCmd.PING, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        response = self.spi_transfer(packet)
        
        if response and len(response) >= 1 and response[0] == FastCmd.RESPONSE:
            return (time.perf_counter() - start) * 1000
        
        return None


# ============================================================================
# CLI Interface
# ============================================================================

def list_devices():
    """List all connected MCP2210 devices."""
    print("Scanning for MCP2210 devices...")
    
    try:
        for device_info in hid.enumerate(MCP2210_VID, MCP2210_PID):
            print(f"\nFound MCP2210:")
            print(f"  Manufacturer: {device_info.get('manufacturer_string', 'N/A')}")
            print(f"  Product: {device_info.get('product_string', 'N/A')}")
            print(f"  Serial: {device_info.get('serial_number', 'N/A')}")
            print(f"  Path: {device_info.get('path', 'N/A')}")
    except Exception as e:
        print(f"Error enumerating devices: {e}")


def test_connection():
    """Test MCP2210 connection and basic functionality."""
    print("Testing MCP2210 connection...")
    
    try:
        with MCP2210KMBox() as kmbox:
            print("\n--- Chip Status ---")
            status = kmbox.get_chip_status()
            if status:
                print(f"  SPI Bus: {status.spi_bus_release_status}")
                print(f"  SPI Owner: {status.spi_owner_status}")
            
            print("\n--- GPIO State ---")
            gpio = kmbox.get_gpio()
            if gpio is not None:
                print(f"  GPIO: {gpio:09b}")
            
            print("\n--- Ping Test ---")
            for i in range(5):
                rtt = kmbox.ping()
                if rtt:
                    print(f"  Ping {i+1}: {rtt:.2f} ms")
                else:
                    print(f"  Ping {i+1}: No response (firmware may need SPI slave support)")
                time.sleep(0.1)
            
            print("\nTest complete!")
            
    except Exception as e:
        print(f"Test failed: {e}")


def interactive_mode():
    """Interactive command mode."""
    print("MCP2210 KMBox Interactive Mode")
    print("Commands: move <x> <y>, click [left|right|middle], key <char>, ping, quit")
    print()
    
    try:
        with MCP2210KMBox() as kmbox:
            while True:
                try:
                    cmd = input("kmbox> ").strip().lower()
                    
                    if not cmd:
                        continue
                    
                    parts = cmd.split()
                    
                    if parts[0] == 'quit' or parts[0] == 'exit':
                        break
                    
                    elif parts[0] == 'move' and len(parts) >= 3:
                        x, y = int(parts[1]), int(parts[2])
                        if kmbox.move(x, y):
                            print(f"Moved ({x}, {y})")
                        else:
                            print("Move failed")
                    
                    elif parts[0] == 'click':
                        button = parts[1] if len(parts) > 1 else 'left'
                        if kmbox.click(button):
                            print(f"Clicked {button}")
                        else:
                            print("Click failed")
                    
                    elif parts[0] == 'key' and len(parts) >= 2:
                        key = parts[1]
                        if kmbox.key_press(key):
                            print(f"Pressed {key}")
                        else:
                            print("Key press failed")
                    
                    elif parts[0] == 'ping':
                        rtt = kmbox.ping()
                        if rtt:
                            print(f"RTT: {rtt:.2f} ms")
                        else:
                            print("No response")
                    
                    elif parts[0] == 'gpio':
                        gpio = kmbox.get_gpio()
                        if gpio is not None:
                            print(f"GPIO: {gpio:09b}")
                    
                    else:
                        print("Unknown command")
                
                except KeyboardInterrupt:
                    print("\nInterrupted")
                    break
                except ValueError as e:
                    print(f"Invalid value: {e}")
                except Exception as e:
                    print(f"Error: {e}")
    
    except Exception as e:
        print(f"Failed to connect: {e}")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='MCP2210 KMBox Client')
    parser.add_argument('--list', action='store_true', help='List connected devices')
    parser.add_argument('--test', action='store_true', help='Test connection')
    parser.add_argument('--interactive', '-i', action='store_true', help='Interactive mode')
    
    args = parser.parse_args()
    
    if args.list:
        list_devices()
    elif args.test:
        test_connection()
    elif args.interactive:
        interactive_mode()
    else:
        parser.print_help()
