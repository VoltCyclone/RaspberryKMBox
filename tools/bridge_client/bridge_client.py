#!/usr/bin/env python3
"""
KMBox Bridge Client

Communicates with KMBox via the SPI-CDC Bridge Pico.
Uses USB CDC serial for simple, fast communication.
"""

import serial
import serial.tools.list_ports
import struct
import time
from typing import Optional, Tuple

# Protocol constants
PROTO_SYNC = 0xAA
CMD_KMBOX = 0x01
CMD_BRIDGE_STATUS = 0x02
CMD_BRIDGE_RESET = 0x03
CMD_ECHO = 0xFF

# KMBox fast commands
FAST_CMD_MOUSE_MOVE = 0x01
FAST_CMD_MOUSE_CLICK = 0x02
FAST_CMD_KEY_PRESS = 0x03
FAST_CMD_PING = 0xFE


class BridgeClient:
    """Client for KMBox via SPI-CDC Bridge"""
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
    
    @staticmethod
    def find_bridge() -> Optional[str]:
        """Find the bridge serial port"""
        for port in serial.tools.list_ports.comports():
            # Look for Raspberry Pi Pico
            if "usbmodem" in port.device.lower() or "acm" in port.device.lower():
                return port.device
            if port.vid == 0x2E8A:  # Raspberry Pi VID
                return port.device
        return None
    
    def connect(self, port: Optional[str] = None) -> bool:
        """Connect to the bridge"""
        if port:
            self.port = port
        if not self.port:
            self.port = self.find_bridge()
        if not self.port:
            print("No bridge found!")
            return False
        
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(0.1)  # Allow USB to settle
            self.serial.reset_input_buffer()
            print(f"Connected to bridge at {self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the bridge"""
        if self.serial:
            self.serial.close()
            self.serial = None
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, *args):
        self.disconnect()
    
    def _checksum(self, data: bytes) -> int:
        """Calculate XOR checksum"""
        cs = 0
        for b in data:
            cs ^= b
        return cs
    
    def _send_frame(self, cmd: int, data: bytes = b'') -> Optional[bytes]:
        """Send a framed command and receive response"""
        if not self.serial:
            return None
        
        # Build frame: [SYNC][LEN][CMD][DATA...][CHECKSUM]
        payload = bytes([cmd]) + data
        frame = bytes([PROTO_SYNC, len(payload)]) + payload
        frame += bytes([self._checksum(frame)])
        
        self.serial.write(frame)
        self.serial.flush()
        
        # Read response
        return self._read_response()
    
    def _read_response(self) -> Optional[bytes]:
        """Read and parse a response frame"""
        if not self.serial:
            return None
        
        # Wait for sync byte
        start = time.time()
        while time.time() - start < 0.5:
            b = self.serial.read(1)
            if b and b[0] == PROTO_SYNC:
                break
        else:
            return None
        
        # Read length
        length_byte = self.serial.read(1)
        if not length_byte:
            return None
        length = length_byte[0]
        
        # Read payload + checksum
        remaining = self.serial.read(length + 1)
        if len(remaining) < length + 1:
            return None
        
        payload = remaining[:length]
        checksum = remaining[length]
        
        # Verify checksum
        frame = bytes([PROTO_SYNC, length]) + payload
        if self._checksum(frame) != checksum:
            return None
        
        return payload
    
    def send_kmbox_command(self, cmd_data: bytes) -> Optional[bytes]:
        """Send a command to KMBox through the bridge"""
        response = self._send_frame(CMD_KMBOX, cmd_data)
        if response and len(response) >= 1:
            status = response[0]
            if status == 0:  # RSP_OK
                return response[1:] if len(response) > 1 else b''
        return None
    
    def ping(self) -> bool:
        """Ping the KMBox through the bridge"""
        cmd = bytes([FAST_CMD_PING, 0, 0, 0, 0, 0, 0, 0])
        response = self.send_kmbox_command(cmd)
        return response is not None
    
    def move_mouse(self, x: int, y: int, buttons: int = 0, wheel: int = 0) -> bool:
        """Move the mouse relative"""
        x = max(-32768, min(32767, x))
        y = max(-32768, min(32767, y))
        wheel = max(-128, min(127, wheel))
        
        cmd = struct.pack('<BhhBbxx', FAST_CMD_MOUSE_MOVE, x, y, buttons, wheel)
        return self.send_kmbox_command(cmd) is not None
    
    def click(self, button: int = 1, count: int = 1) -> bool:
        """Click mouse button (1=left, 2=right, 4=middle)"""
        cmd = struct.pack('<BBBxxxxx', FAST_CMD_MOUSE_CLICK, button, count)
        return self.send_kmbox_command(cmd) is not None
    
    def key_press(self, keycode: int, modifier: int = 0) -> bool:
        """Press and release a key"""
        cmd = struct.pack('<BBBxxxxx', FAST_CMD_KEY_PRESS, keycode, modifier)
        return self.send_kmbox_command(cmd) is not None
    
    def get_bridge_status(self) -> Optional[Tuple[int, int, int]]:
        """Get bridge statistics: (sent, received, errors)"""
        response = self._send_frame(CMD_BRIDGE_STATUS)
        if response and len(response) >= 13:
            status = response[0]
            if status == 0:
                sent = struct.unpack('<I', response[1:5])[0]
                recv = struct.unpack('<I', response[5:9])[0]
                errs = struct.unpack('<I', response[9:13])[0]
                return (sent, recv, errs)
        return None
    
    def echo(self, data: bytes) -> Optional[bytes]:
        """Echo test"""
        response = self._send_frame(CMD_ECHO, data)
        if response and len(response) >= 1 and response[0] == 0:
            return response[1:]
        return None


def main():
    import argparse
    parser = argparse.ArgumentParser(description='KMBox Bridge Client')
    parser.add_argument('--port', '-p', help='Serial port')
    parser.add_argument('--test', '-t', action='store_true', help='Run connection test')
    parser.add_argument('--status', '-s', action='store_true', help='Get bridge status')
    parser.add_argument('--move', '-m', nargs=2, type=int, metavar=('X', 'Y'), help='Move mouse')
    parser.add_argument('--click', '-c', type=int, default=0, help='Click button (1=left, 2=right)')
    args = parser.parse_args()
    
    client = BridgeClient(args.port)
    if not client.connect():
        return 1
    
    try:
        if args.test:
            print("Testing connection...")
            
            # Echo test
            test_data = b'Hello Bridge!'
            echo_result = client.echo(test_data)
            if echo_result == test_data:
                print(f"✓ Echo test passed")
            else:
                print(f"✗ Echo test failed: got {echo_result}")
            
            # Ping KMBox
            if client.ping():
                print(f"✓ KMBox ping successful")
            else:
                print(f"✗ KMBox ping failed")
            
            # Status
            status = client.get_bridge_status()
            if status:
                sent, recv, errs = status
                print(f"✓ Bridge status: sent={sent}, recv={recv}, errors={errs}")
            else:
                print(f"✗ Failed to get bridge status")
        
        elif args.status:
            status = client.get_bridge_status()
            if status:
                sent, recv, errs = status
                print(f"Packets sent: {sent}")
                print(f"Packets received: {recv}")
                print(f"Errors: {errs}")
            else:
                print("Failed to get status")
        
        elif args.move:
            x, y = args.move
            if client.move_mouse(x, y):
                print(f"Moved mouse by ({x}, {y})")
            else:
                print("Failed to move mouse")
        
        elif args.click:
            if client.click(args.click):
                print(f"Clicked button {args.click}")
            else:
                print("Failed to click")
        
        else:
            # Interactive mode
            print("Interactive mode. Commands:")
            print("  m X Y    - Move mouse")
            print("  c [1|2]  - Click (1=left, 2=right)")
            print("  p        - Ping")
            print("  s        - Status")
            print("  q        - Quit")
            
            while True:
                try:
                    cmd = input("> ").strip().split()
                    if not cmd:
                        continue
                    
                    if cmd[0] == 'q':
                        break
                    elif cmd[0] == 'm' and len(cmd) >= 3:
                        x, y = int(cmd[1]), int(cmd[2])
                        if client.move_mouse(x, y):
                            print(f"OK: moved ({x}, {y})")
                        else:
                            print("Failed")
                    elif cmd[0] == 'c':
                        btn = int(cmd[1]) if len(cmd) > 1 else 1
                        if client.click(btn):
                            print(f"OK: clicked {btn}")
                        else:
                            print("Failed")
                    elif cmd[0] == 'p':
                        if client.ping():
                            print("OK: pong")
                        else:
                            print("Failed")
                    elif cmd[0] == 's':
                        status = client.get_bridge_status()
                        if status:
                            print(f"sent={status[0]} recv={status[1]} errors={status[2]}")
                        else:
                            print("Failed")
                    else:
                        print("Unknown command")
                
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"Error: {e}")
    
    finally:
        client.disconnect()
    
    return 0


if __name__ == '__main__':
    exit(main())
