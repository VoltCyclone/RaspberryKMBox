#!/usr/bin/env python3
"""
KMBox Bridge Client - Advanced Testing & Communication

Features:
- Binary fast command protocol (8-byte packets)
- Mouse movement testing with visual feedback
- Connection monitoring and diagnostics
- Frame streaming for autopilot testing
- Latency measurement

Usage:
    python bridge_client.py --list           # List serial ports
    python bridge_client.py --test           # Quick connection test
    python bridge_client.py --monitor        # Monitor bridge output
    python bridge_client.py --mouse-test     # Test mouse movement
    python bridge_client.py --latency        # Measure round-trip latency
    python bridge_client.py                  # Interactive mode
"""

import serial
import serial.tools.list_ports
import argparse
import time
import sys
import struct
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple
from enum import IntEnum


# =============================================================================
# Fast Binary Protocol Constants (from KMBox defines.h)
# =============================================================================

class FastCmd(IntEnum):
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


class MouseButton(IntEnum):
    LEFT = 0x01
    RIGHT = 0x02
    MIDDLE = 0x04
    BACK = 0x08
    FORWARD = 0x10


PACKET_SIZE = 8


# =============================================================================
# Packet Builders
# =============================================================================

def build_mouse_move(x: int, y: int, buttons: int = 0, wheel: int = 0) -> bytes:
    """Build 8-byte mouse move packet"""
    # Clamp values
    x = max(-32768, min(32767, x))
    y = max(-32768, min(32767, y))
    wheel = max(-128, min(127, wheel))
    # Pack: cmd(1) + x(2) + y(2) + buttons(1) + wheel(1) + pad(1)
    return struct.pack('<BhhBbB', FastCmd.MOUSE_MOVE, x, y, buttons, wheel, 0)


def build_mouse_click(button: int, count: int = 1) -> bytes:
    """Build 8-byte mouse click packet"""
    return struct.pack('<BBBBBBBB', FastCmd.MOUSE_CLICK, button, count, 0, 0, 0, 0, 0)


def build_key_press(keycode: int, modifiers: int = 0) -> bytes:
    """Build 8-byte key press packet"""
    return struct.pack('<BBBBBBBB', FastCmd.KEY_PRESS, keycode, modifiers, 0, 0, 0, 0, 0)


def build_ping() -> bytes:
    """Build 8-byte ping packet"""
    return struct.pack('<BBBBBBBB', FastCmd.PING, 0, 0, 0, 0, 0, 0, 0)


def build_smooth_move(x: int, y: int, mode: int = 1) -> bytes:
    """Build 8-byte smooth move packet"""
    x = max(-32768, min(32767, x))
    y = max(-32768, min(32767, y))
    return struct.pack('<BhhBBB', FastCmd.SMOOTH_MOVE, x, y, mode, 0, 0)


# =============================================================================
# Frame Protocol (for autopilot)
# =============================================================================

def build_frame_header(width: int, height: int) -> bytes:
    """Build frame header: 'FR' + width(2) + height(2) + reserved(2)"""
    return struct.pack('<2sHHH', b'FR', width, height, 0)


def build_test_frame(width: int, height: int, target_x: int, target_y: int, 
                     target_size: int = 5) -> bytes:
    """Build a test frame with a red target blob"""
    header = build_frame_header(width, height)
    
    # Create RGB pixel data
    pixels = bytearray(width * height * 3)
    
    # Fill with dark background
    for i in range(0, len(pixels), 3):
        pixels[i] = 30      # R
        pixels[i+1] = 30    # G
        pixels[i+2] = 30    # B
    
    # Draw red target blob
    for dy in range(-target_size, target_size + 1):
        for dx in range(-target_size, target_size + 1):
            px = target_x + dx
            py = target_y + dy
            if 0 <= px < width and 0 <= py < height:
                # Distance from center for soft edge
                dist = (dx*dx + dy*dy) ** 0.5
                if dist <= target_size:
                    idx = (py * width + px) * 3
                    intensity = int(255 * (1 - dist / target_size / 1.5))
                    pixels[idx] = max(200, intensity)  # R (bright red)
                    pixels[idx+1] = min(50, 255 - intensity)  # G (low)
                    pixels[idx+2] = min(50, 255 - intensity)  # B (low)
    
    return header + bytes(pixels)


# =============================================================================
# Bridge Client Class
# =============================================================================

class KMBoxBridge:
    """Advanced client for KMBox via UART bridge"""
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 0.1):
        self.port = port or self._find_bridge()
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._monitor_running = False
        self._rx_buffer = ""
        self._stats = {
            'tx_bytes': 0,
            'rx_bytes': 0,
            'tx_packets': 0,
            'pings_sent': 0,
            'pongs_received': 0,
        }
        
    @staticmethod
    def _find_bridge() -> Optional[str]:
        """Auto-detect bridge serial port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            desc_lower = port.description.lower()
            hwid_lower = port.hwid.lower()
            # Look for Pico/RP2040/RP2350 or Adafruit VID
            if any(x in desc_lower for x in ['pico', 'rp2040', 'rp2350', 'bridge', 'kmbox']):
                return port.device
            if '2e8a' in hwid_lower:  # Raspberry Pi VID
                return port.device
        return None
    
    @staticmethod
    def list_ports() -> List[Tuple[str, str, str]]:
        """List all available serial ports"""
        return [(p.device, p.description, p.hwid) for p in serial.tools.list_ports.comports()]
    
    def connect(self) -> bool:
        """Connect to the bridge"""
        if not self.port:
            print("ERROR: No bridge port found. Use --list to see available ports.")
            return False
        
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout
            )
            time.sleep(0.2)  # Let bridge initialize
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the bridge"""
        self._monitor_running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None
    
    # -------------------------------------------------------------------------
    # Low-level I/O
    # -------------------------------------------------------------------------
    
    def send_raw(self, data: bytes) -> int:
        """Send raw bytes"""
        if not self.serial:
            return 0
        n = self.serial.write(data)
        self._stats['tx_bytes'] += n
        return n
    
    def send_packet(self, packet: bytes) -> bool:
        """Send an 8-byte fast command packet"""
        if len(packet) != PACKET_SIZE:
            raise ValueError(f"Packet must be {PACKET_SIZE} bytes, got {len(packet)}")
        n = self.send_raw(packet)
        if n == PACKET_SIZE:
            self._stats['tx_packets'] += 1
            return True
        return False
    
    def read_available(self) -> bytes:
        """Read all available data"""
        if not self.serial:
            return b''
        data = b''
        while self.serial.in_waiting:
            chunk = self.serial.read(self.serial.in_waiting)
            data += chunk
            self._stats['rx_bytes'] += len(chunk)
        return data
    
    def read_line(self, timeout: float = 0.5) -> Optional[str]:
        """Read a line with timeout"""
        if not self.serial:
            return None
        start = time.time()
        line = ""
        while time.time() - start < timeout:
            if self.serial.in_waiting:
                c = self.serial.read(1).decode('utf-8', errors='replace')
                self._stats['rx_bytes'] += 1
                if c == '\n':
                    return line.strip()
                elif c != '\r':
                    line += c
            else:
                time.sleep(0.001)
        return line.strip() if line else None
    
    # -------------------------------------------------------------------------
    # Fast Binary Commands
    # -------------------------------------------------------------------------
    
    def ping(self) -> Tuple[bool, float]:
        """Send ping and measure round-trip time"""
        if not self.serial:
            return False, 0.0
        
        self.serial.reset_input_buffer()
        start = time.perf_counter()
        self.send_packet(build_ping())
        self._stats['pings_sent'] += 1
        
        # Wait for 0xFF response
        timeout = 0.1
        while time.perf_counter() - start < timeout:
            if self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                self._stats['rx_bytes'] += len(data)
                if 0xFF in data:
                    rtt = (time.perf_counter() - start) * 1000
                    self._stats['pongs_received'] += 1
                    return True, rtt
            time.sleep(0.0001)
        
        return False, 0.0
    
    def move(self, x: int, y: int, buttons: int = 0) -> bool:
        """Send relative mouse movement"""
        return self.send_packet(build_mouse_move(x, y, buttons))
    
    def click(self, button: int = MouseButton.LEFT, count: int = 1) -> bool:
        """Send mouse click"""
        return self.send_packet(build_mouse_click(button, count))
    
    def smooth_move(self, x: int, y: int, mode: int = 1) -> bool:
        """Send smooth mouse movement"""
        return self.send_packet(build_smooth_move(x, y, mode))
    
    def key(self, keycode: int, modifiers: int = 0) -> bool:
        """Send key press"""
        return self.send_packet(build_key_press(keycode, modifiers))
    
    # -------------------------------------------------------------------------
    # Frame Streaming (for autopilot)
    # -------------------------------------------------------------------------
    
    def send_frame(self, width: int, height: int, pixels: bytes) -> bool:
        """Send a frame to the autopilot"""
        if len(pixels) != width * height * 3:
            raise ValueError(f"Pixel data size mismatch: expected {width*height*3}, got {len(pixels)}")
        header = build_frame_header(width, height)
        return self.send_raw(header + pixels) == len(header) + len(pixels)
    
    def send_test_frame(self, width: int, height: int, target_x: int, target_y: int) -> bool:
        """Send a test frame with a target blob"""
        frame = build_test_frame(width, height, target_x, target_y)
        return self.send_raw(frame) == len(frame)
    
    # -------------------------------------------------------------------------
    # Monitoring
    # -------------------------------------------------------------------------
    
    def start_monitor(self, callback=None):
        """Start background monitoring thread"""
        if self._monitor_running:
            return
        
        self._monitor_running = True
        
        def monitor_loop():
            while self._monitor_running and self.serial:
                try:
                    if self.serial.in_waiting:
                        data = self.serial.read(self.serial.in_waiting)
                        self._stats['rx_bytes'] += len(data)
                        text = data.decode('utf-8', errors='replace')
                        if callback:
                            callback(text)
                        else:
                            print(text, end='', flush=True)
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    if self._monitor_running:
                        print(f"\nMonitor error: {e}")
                    break
        
        self._monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        self._monitor_thread.start()
    
    def stop_monitor(self):
        """Stop background monitoring"""
        self._monitor_running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
            self._monitor_thread = None
    
    def get_stats(self) -> dict:
        """Get communication statistics"""
        return self._stats.copy()
    
    # -------------------------------------------------------------------------
    # Context Manager
    # -------------------------------------------------------------------------
    
    def __enter__(self):
        if self.connect():
            return self
        raise RuntimeError("Failed to connect")
    
    def __exit__(self, *args):
        self.disconnect()


# =============================================================================
# Test Functions
# =============================================================================

def test_connection(port: Optional[str] = None, verbose: bool = True) -> bool:
    """Comprehensive connection test"""
    print("=" * 60)
    print("KMBox Bridge Connection Test")
    print("=" * 60)
    
    # Find port
    bridge = KMBoxBridge(port=port)
    if not bridge.port:
        print("\n✗ No bridge found!")
        print("\nAvailable ports:")
        for dev, desc, hwid in KMBoxBridge.list_ports():
            print(f"  {dev}: {desc}")
        return False
    
    print(f"\nUsing port: {bridge.port}")
    
    # Connect
    if not bridge.connect():
        return False
    
    try:
        # Read any startup messages
        print("\n--- Bridge Output ---")
        time.sleep(0.5)
        startup = bridge.read_available().decode('utf-8', errors='replace')
        if startup:
            print(startup)
        
        # Ping test
        print("\n--- Ping Test ---")
        successes = 0
        rtts = []
        for i in range(10):
            ok, rtt = bridge.ping()
            if ok:
                successes += 1
                rtts.append(rtt)
                if verbose:
                    print(f"  Ping {i+1}: {rtt:.2f} ms")
            else:
                if verbose:
                    print(f"  Ping {i+1}: timeout")
            time.sleep(0.05)
        
        if rtts:
            avg_rtt = sum(rtts) / len(rtts)
            min_rtt = min(rtts)
            max_rtt = max(rtts)
            print(f"\nPing results: {successes}/10 success")
            print(f"  RTT: avg={avg_rtt:.2f}ms, min={min_rtt:.2f}ms, max={max_rtt:.2f}ms")
        else:
            print("\n✗ No ping responses (KMBox may not be connected)")
        
        # Mouse movement test
        print("\n--- Mouse Command Test ---")
        print("Sending small test movements...")
        for i in range(5):
            bridge.move(1, 0)
            time.sleep(0.02)
        for i in range(5):
            bridge.move(-1, 0)
            time.sleep(0.02)
        print("  ✓ Movement commands sent")
        
        # Read any responses
        time.sleep(0.2)
        response = bridge.read_available().decode('utf-8', errors='replace')
        if response:
            print(f"\nBridge response:\n{response}")
        
        # Stats
        stats = bridge.get_stats()
        print("\n--- Statistics ---")
        print(f"  TX: {stats['tx_bytes']} bytes, {stats['tx_packets']} packets")
        print(f"  RX: {stats['rx_bytes']} bytes")
        print(f"  Pings: {stats['pongs_received']}/{stats['pings_sent']} responded")
        
        print("\n" + "=" * 60)
        print("✓ Connection test complete!")
        print("=" * 60)
        return True
        
    finally:
        bridge.disconnect()


def test_latency(port: Optional[str] = None, count: int = 100) -> bool:
    """Measure ping latency statistics"""
    print(f"Measuring latency over {count} pings...")
    
    try:
        with KMBoxBridge(port=port) as bridge:
            rtts = []
            for i in range(count):
                ok, rtt = bridge.ping()
                if ok:
                    rtts.append(rtt)
                    print(f"\r  {i+1}/{count}: {rtt:.2f} ms", end='', flush=True)
                else:
                    print(f"\r  {i+1}/{count}: timeout  ", end='', flush=True)
                time.sleep(0.01)
            
            print()
            if rtts:
                rtts.sort()
                avg = sum(rtts) / len(rtts)
                p50 = rtts[len(rtts) // 2]
                p95 = rtts[int(len(rtts) * 0.95)]
                p99 = rtts[int(len(rtts) * 0.99)] if len(rtts) >= 100 else rtts[-1]
                
                print(f"\nLatency Statistics ({len(rtts)}/{count} successful):")
                print(f"  Min:    {min(rtts):.2f} ms")
                print(f"  Avg:    {avg:.2f} ms")
                print(f"  Median: {p50:.2f} ms")
                print(f"  P95:    {p95:.2f} ms")
                print(f"  P99:    {p99:.2f} ms")
                print(f"  Max:    {max(rtts):.2f} ms")
                return True
            else:
                print("\n✗ No successful pings")
                return False
                
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False


def test_mouse_movement(port: Optional[str] = None) -> bool:
    """Interactive mouse movement test"""
    print("Mouse Movement Test")
    print("=" * 40)
    print("This will move your mouse cursor!")
    print("Press Ctrl+C to stop")
    print()
    
    try:
        with KMBoxBridge(port=port) as bridge:
            # Start monitoring in background
            bridge.start_monitor()
            
            time.sleep(0.5)  # Let connection stabilize
            
            # Test patterns
            patterns = [
                ("Small square", [(10, 0)] * 10 + [(0, 10)] * 10 + [(-10, 0)] * 10 + [(0, -10)] * 10),
                ("Circle-ish", [(3, -1), (2, -2), (1, -3), (0, -3), (-1, -3), (-2, -2), (-3, -1),
                               (-3, 0), (-3, 1), (-2, 2), (-1, 3), (0, 3), (1, 3), (2, 2), (3, 1), (3, 0)]),
                ("Zigzag", [(5, 5), (-5, 5)] * 5 + [(-5, -5), (5, -5)] * 100),
            ]
            
            for name, moves in patterns:
                print(f"\nRunning pattern: {name}")
                input("  Press Enter to start...")
                
                for x, y in moves:
                    bridge.move(x, y)
                    time.sleep(0.016)  # ~60fps
                
                print(f"  ✓ {name} complete")
                time.sleep(0.5)
            
            print("\n✓ Mouse test complete!")
            bridge.stop_monitor()
            return True
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
        return True
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False


def monitor_mode(port: Optional[str] = None):
    """Monitor bridge output continuously"""
    print("Monitor Mode - Press Ctrl+C to exit")
    print("=" * 60)
    
    try:
        with KMBoxBridge(port=port) as bridge:
            bridge.start_monitor()
            
            while True:
                time.sleep(1)
                stats = bridge.get_stats()
                # Print stats every 10 seconds
                if stats['rx_bytes'] > 0 and int(time.time()) % 10 == 0:
                    print(f"\n[Stats] RX: {stats['rx_bytes']} bytes | "
                          f"Pings: {stats['pongs_received']}/{stats['pings_sent']}")
                    
    except KeyboardInterrupt:
        print("\n\nMonitor stopped")


def interactive_mode(port: Optional[str] = None):
    """Interactive command mode"""
    print("KMBox Bridge Interactive Mode")
    print("=" * 60)
    print("Commands:")
    print("  move <x> <y>      - Relative mouse movement")
    print("  smooth <x> <y>    - Smooth mouse movement")
    print("  click [l|r|m]     - Mouse click (left/right/middle)")
    print("  key <code>        - Key press (USB HID keycode)")
    print("  ping              - Send ping")
    print("  ping <count>      - Multiple pings with stats")
    print("  frame <x> <y>     - Send test frame with target at x,y")
    print("  stats             - Show statistics")
    print("  monitor           - Toggle output monitoring")
    print("  quit              - Exit")
    print()
    
    try:
        with KMBoxBridge(port=port) as bridge:
            monitoring = False
            
            while True:
                try:
                    cmd = input("kmbox> ").strip()
                    if not cmd:
                        continue
                    
                    parts = cmd.lower().split()
                    
                    if parts[0] in ('quit', 'exit', 'q'):
                        break
                    
                    elif parts[0] == 'move' and len(parts) >= 3:
                        x, y = int(parts[1]), int(parts[2])
                        bridge.move(x, y)
                        print(f"  -> moved ({x}, {y})")
                    
                    elif parts[0] == 'smooth' and len(parts) >= 3:
                        x, y = int(parts[1]), int(parts[2])
                        bridge.smooth_move(x, y)
                        print(f"  -> smooth ({x}, {y})")
                    
                    elif parts[0] == 'click':
                        btn = MouseButton.LEFT
                        if len(parts) > 1:
                            btn = {'l': MouseButton.LEFT, 'r': MouseButton.RIGHT, 
                                   'm': MouseButton.MIDDLE}.get(parts[1][0], MouseButton.LEFT)
                        bridge.click(btn)
                        print(f"  -> clicked")
                    
                    elif parts[0] == 'key' and len(parts) >= 2:
                        code = int(parts[1], 0)  # Allow hex with 0x prefix
                        bridge.key(code)
                        print(f"  -> key {code}")
                    
                    elif parts[0] == 'ping':
                        count = int(parts[1]) if len(parts) > 1 else 1
                        rtts = []
                        for i in range(count):
                            ok, rtt = bridge.ping()
                            if ok:
                                rtts.append(rtt)
                                if count == 1:
                                    print(f"  -> pong! ({rtt:.2f} ms)")
                            else:
                                if count == 1:
                                    print(f"  -> timeout")
                            time.sleep(0.01)
                        if count > 1 and rtts:
                            print(f"  -> {len(rtts)}/{count} ok, "
                                  f"avg={sum(rtts)/len(rtts):.2f}ms")
                    
                    elif parts[0] == 'frame' and len(parts) >= 3:
                        x, y = int(parts[1]), int(parts[2])
                        bridge.send_test_frame(48, 48, x, y)
                        print(f"  -> sent 48x48 frame with target at ({x}, {y})")
                    
                    elif parts[0] == 'stats':
                        stats = bridge.get_stats()
                        print(f"  TX: {stats['tx_bytes']} bytes, {stats['tx_packets']} packets")
                        print(f"  RX: {stats['rx_bytes']} bytes")
                        print(f"  Pings: {stats['pongs_received']}/{stats['pings_sent']}")
                    
                    elif parts[0] == 'monitor':
                        if monitoring:
                            bridge.stop_monitor()
                            monitoring = False
                            print("  -> monitoring stopped")
                        else:
                            bridge.start_monitor()
                            monitoring = True
                            print("  -> monitoring started")
                    
                    else:
                        print(f"  Unknown command: {parts[0]}")
                    
                    # Check for any responses
                    if not monitoring:
                        time.sleep(0.05)
                        data = bridge.read_available()
                        if data:
                            print(f"  [RX] {data.decode('utf-8', errors='replace').strip()}")
                    
                except ValueError as e:
                    print(f"  Error: {e}")
                except KeyboardInterrupt:
                    print()
                    continue
                    
    except Exception as e:
        print(f"Error: {e}")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="KMBox Bridge Client",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --list              List available serial ports
  %(prog)s --test              Run comprehensive connection test
  %(prog)s --latency           Measure ping latency (100 samples)
  %(prog)s --mouse-test        Interactive mouse movement test
  %(prog)s --monitor           Monitor bridge output
  %(prog)s                     Interactive command mode
        """
    )
    parser.add_argument('--list', action='store_true', help='List available serial ports')
    parser.add_argument('--test', action='store_true', help='Run connection test')
    parser.add_argument('--latency', action='store_true', help='Measure ping latency')
    parser.add_argument('--mouse-test', action='store_true', help='Test mouse movement')
    parser.add_argument('--monitor', action='store_true', help='Monitor bridge output')
    parser.add_argument('--port', '-p', help='Serial port to use')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate (default: 115200)')
    args = parser.parse_args()
    
    if args.list:
        print("Available serial ports:")
        for dev, desc, hwid in KMBoxBridge.list_ports():
            print(f"  {dev}")
            print(f"    Description: {desc}")
            print(f"    Hardware ID: {hwid}")
        return
    
    if args.test:
        test_connection(args.port)
    elif args.latency:
        test_latency(args.port)
    elif args.mouse_test:
        test_mouse_movement(args.port)
    elif args.monitor:
        monitor_mode(args.port)
    else:
        interactive_mode(args.port)


if __name__ == "__main__":
    main()