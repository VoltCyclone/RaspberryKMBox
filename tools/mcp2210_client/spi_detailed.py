#!/usr/bin/env python3
"""
Detailed SPI Debug - Check what's happening on the SPI bus
"""

import hid
import time

MCP2210_VID = 0x04D8
MCP2210_PID = 0x00DE

def main():
    print("Opening MCP2210...")
    device = hid.device()
    device.open(MCP2210_VID, MCP2210_PID)
    print(f"Opened: {device.get_product_string()}")
    
    def send_command(cmd):
        while len(cmd) < 64:
            cmd = cmd + bytes([0x00])
        device.write(bytes([0x00]) + cmd)
        return bytes(device.read(64))
    
    # Cancel any pending transfers
    send_command(bytes([0x11]))
    time.sleep(0.1)
    
    # Configure SPI: 100 kHz, Mode 0, NO bytes per transfer limit
    print("Configuring SPI at 100 kHz...")
    spi_settings = bytearray(64)
    spi_settings[0] = 0x40
    # 100 kHz
    spi_settings[4:8] = [0xA0, 0x86, 0x01, 0x00]
    # Idle CS = all high
    spi_settings[8:10] = [0xFF, 0x01]
    # Active CS = GP0 low
    spi_settings[10:12] = [0xFE, 0x01]
    # CS delays (longer for debugging)
    spi_settings[12:14] = [0x64, 0x00]  # 100 * 100ns = 10us
    spi_settings[14:16] = [0x64, 0x00]
    spi_settings[16:18] = [0x00, 0x00]  # No inter-byte delay
    # Bytes per transfer = 0 (use count in transfer command)
    spi_settings[18:20] = [0x00, 0x00]
    # Mode 0
    spi_settings[20] = 0x00
    
    resp = send_command(bytes(spi_settings))
    print(f"SPI config status: {resp[1]} (0=success)")
    
    # Configure GPIO: GP0 as CS
    gpio_settings = bytearray(64)
    gpio_settings[0] = 0x21
    gpio_settings[4] = 0x01  # GP0 = CS
    gpio_settings[13:15] = [0xFF, 0x01]  # Default high
    gpio_settings[15:17] = [0xFE, 0x01]  # All output except GP0
    send_command(bytes(gpio_settings))
    
    # Read back settings
    resp = send_command(bytes([0x41]))
    bit_rate = resp[4] | (resp[5] << 8) | (resp[6] << 16) | (resp[7] << 24)
    print(f"Bit rate: {bit_rate} Hz")
    
    # Do transfer with detailed status checking
    print("\n=== 8-Byte SPI Transfer ===")
    tx_data = bytes([0xF0, 0xAA, 0x55, 0x12, 0x34, 0x56, 0x78, 0x9A])
    
    spi_cmd = bytearray(64)
    spi_cmd[0] = 0x42  # SPI_TRANSFER
    spi_cmd[1] = len(tx_data)  # Number of bytes to transfer
    spi_cmd[4:4+len(tx_data)] = tx_data
    
    print(f"TX data: {tx_data.hex()}")
    
    resp = send_command(bytes(spi_cmd))
    print(f"Response bytes [0:20]: {resp[:20].hex()}")
    print(f"Response bytes [20:40]: {resp[20:40].hex()}")
    print(f"Response bytes [40:60]: {resp[40:64].hex()}")
    
    # According to datasheet, if transfer is FINISHED immediately,
    # received data should be at byte 4 onwards
    # Let's explicitly look at bytes 4-11 (where 8 received bytes would be)
    print(f"Potential RX data at [4:12]: {resp[4:12].hex()}")
    
    # Parse response
    # Byte 0: Echo of command (0x42)
    # Byte 1: SPI engine status
    # Byte 2: Number of bytes received in this packet
    # Byte 3: SPI engine status (more detail)
    # Byte 4+: Received data
    
    cmd_echo = resp[0]
    spi_status = resp[1]
    rx_count = resp[2]
    engine_status = resp[3]
    
    status_names = {
        0x00: "FINISHED",
        0x10: "STARTED",
        0x20: "NOT_FINISHED",
        0x30: "BUS_NOT_AVAILABLE",
        0xF7: "WRITE_ERROR",
        0xF8: "BUS_BUSY",
    }
    
    print(f"Command echo: 0x{cmd_echo:02X}")
    print(f"SPI status: 0x{spi_status:02X} ({status_names.get(spi_status, 'UNKNOWN')})")
    print(f"RX count: {rx_count}")
    print(f"Engine status: 0x{engine_status:02X}")
    
    if rx_count > 0:
        print(f"RX data: {resp[4:4+rx_count].hex()}")
    
    # Keep polling if not finished
    total_rx = bytearray()
    if rx_count > 0:
        total_rx.extend(resp[4:4+rx_count])
    
    poll_count = 0
    while spi_status in (0x10, 0x20) and poll_count < 100:
        poll_count += 1
        resp = send_command(bytes([0x42, 0, 0, 0]))
        spi_status = resp[1]
        rx_count = resp[2]
        print(f"Poll {poll_count}: status=0x{spi_status:02X}, rx_count={rx_count}")
        if rx_count > 0:
            total_rx.extend(resp[4:4+rx_count])
            print(f"  RX: {resp[4:4+rx_count].hex()}")
    
    print(f"\nTotal received: {bytes(total_rx).hex() if total_rx else 'none'}")
    
    # Check chip status after transfer
    print("\n=== Post-transfer status ===")
    resp = send_command(bytes([0x10]))
    print(f"Chip status: {resp[:10].hex()}")
    print(f"  SPI bus release: {resp[1]}")
    print(f"  Bus owner: {resp[2]}")
    print(f"  Password attempts: {resp[3]}")
    print(f"  Password guessed: {resp[4]}")
    
    device.close()
    print("\nDone!")

if __name__ == "__main__":
    main()
