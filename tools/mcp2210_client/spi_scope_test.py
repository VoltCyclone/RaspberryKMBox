#!/usr/bin/env python3
"""
Test SPI with extensive debug output - designed to be watched with a scope
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
    
    # First, check CURRENT RUNTIME GPIO settings
    print("\n=== Current Runtime GPIO ===")
    resp = send_command(bytes([0x31]))  # GET_GPIO_PIN_VAL
    print(f"GPIO values: {resp[:10].hex()}")
    gpio_vals = resp[4] | (resp[5] << 8)
    for i in range(9):
        print(f"  GP{i}: {'HIGH' if gpio_vals & (1 << i) else 'LOW'}")
    
    # Read chip settings (NVRAM)
    print("\n=== NVRAM Chip Settings ===")
    resp = send_command(bytes([0x20]))
    print(f"Response: {resp[:25].hex()}")
    designations = resp[4:13]
    for i in range(9):
        d = designations[i]
        names = {0: "GPIO", 1: "CS", 2: "Dedicated"}
        print(f"  GP{i}: {names.get(d, f'Unknown({d})')}")
    
    # Set chip settings in VOLATILE RAM (runtime)
    print("\n=== Setting Runtime GPIO (GP0=CS) ===")
    gpio_cmd = bytearray(64)
    gpio_cmd[0] = 0x21  # SET_CHIP_SETTINGS (runtime)
    # GP0 = CS (1), rest = GPIO or Dedicated
    gpio_cmd[4] = 0x01  # GP0 = CS
    gpio_cmd[5] = 0x02  # GP1 = Dedicated (SPI functions)
    gpio_cmd[6] = 0x02  # GP2 = Dedicated
    gpio_cmd[7] = 0x02  # GP3 = Dedicated
    gpio_cmd[8] = 0x02  # GP4 = Dedicated
    gpio_cmd[9] = 0x02  # GP5 = Dedicated
    gpio_cmd[10] = 0x02 # GP6 = Dedicated
    gpio_cmd[11] = 0x00 # GP7 = GPIO
    gpio_cmd[12] = 0x00 # GP8 = GPIO
    # Default output (GP0 high = CS idle)
    gpio_cmd[13] = 0xFF
    gpio_cmd[14] = 0x01
    # Default direction (0 = output)
    gpio_cmd[15] = 0x00
    gpio_cmd[16] = 0x00
    resp = send_command(bytes(gpio_cmd))
    print(f"Response: cmd={resp[0]:02X} status={resp[1]:02X}")
    if resp[1] != 0:
        print("  ERROR setting GPIO!")
    
    # Verify chip settings
    print("\n=== Verify Runtime Chip Settings ===")
    resp = send_command(bytes([0x20]))
    designations = resp[4:13]
    for i in range(9):
        d = designations[i]
        names = {0: "GPIO", 1: "CS", 2: "Dedicated"}
        print(f"  GP{i}: {names.get(d, f'Unknown({d})')}")
    
    # Set SPI transfer settings
    print("\n=== Setting SPI Transfer Settings ===")
    spi_cmd = bytearray(64)
    spi_cmd[0] = 0x40  # SET_SPI_SETTING
    # Bit rate: 100000 Hz = 0x000186A0
    spi_cmd[4:8] = [0xA0, 0x86, 0x01, 0x00]
    # Idle CS value: GP0 HIGH (all bits high) = 0x01FF
    spi_cmd[8:10] = [0xFF, 0x01]
    # Active CS value: GP0 LOW = 0x01FE 
    spi_cmd[10:12] = [0xFE, 0x01]
    # CS-to-data delay: 10us = 0x0064 (in 100ns units... so 100*100ns = 10us)
    spi_cmd[12:14] = [0x64, 0x00]
    # Data-to-CS delay: 10us
    spi_cmd[14:16] = [0x64, 0x00]
    # Inter-byte delay: 0
    spi_cmd[16:18] = [0x00, 0x00]
    # Bytes per transfer
    spi_cmd[18:20] = [0x08, 0x00]  # 8 bytes
    # SPI Mode 0
    spi_cmd[20] = 0x00
    resp = send_command(bytes(spi_cmd))
    print(f"Response: cmd={resp[0]:02X} status={resp[1]:02X}")
    
    # Verify SPI settings
    resp = send_command(bytes([0x41]))  # GET_SPI_SETTING
    print(f"SPI Settings:")
    bitrate = resp[4]|(resp[5]<<8)|(resp[6]<<16)|(resp[7]<<24)
    print(f"  Bit rate: {bitrate} Hz")
    print(f"  Idle CS: 0x{resp[8]|(resp[9]<<8):04X}")
    print(f"  Active CS: 0x{resp[10]|(resp[11]<<8):04X}")
    print(f"  Bytes/xfer: {resp[18]|(resp[19]<<8)}")
    print(f"  Mode: {resp[20]}")
    
    # Check GPIO before transfer
    print("\n=== GPIO Before Transfer ===")
    resp = send_command(bytes([0x31]))
    gpio_vals = resp[4] | (resp[5] << 8)
    for i in range(9):
        print(f"  GP{i}: {'HIGH' if gpio_vals & (1 << i) else 'LOW'}")
    
    # Now do SPI transfer
    print("\n=== Starting SPI Transfer ===")
    print("(Watch CS/SCK/MOSI with scope)")
    time.sleep(1)
    
    tx_data = bytes([0xF0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07])
    xfer_cmd = bytearray(64)
    xfer_cmd[0] = 0x42  # SPI_TRANSFER
    xfer_cmd[1] = len(tx_data)
    xfer_cmd[4:4+len(tx_data)] = tx_data
    
    print(f"Sending: {tx_data.hex()}")
    resp = send_command(bytes(xfer_cmd))
    
    print(f"\nResponse hex dump:")
    for i in range(0, 64, 16):
        hex_str = ' '.join(f'{b:02X}' for b in resp[i:i+16])
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in resp[i:i+16])
        print(f"  {i:02X}: {hex_str}  |{ascii_str}|")
    
    status = resp[3]
    status_names = {
        0x00: "FINISHED",
        0x01: "STARTED, NOT PENDING",
        0x10: "NOT STARTED, PENDING",
        0x20: "STARTED, PENDING"
    }
    print(f"\nStatus: 0x{status:02X} ({status_names.get(status, 'UNKNOWN')})")
    
    # If transfer started but pending, poll for completion
    if status in [0x10, 0x20]:
        print("Polling for completion...")
        for attempt in range(10):
            time.sleep(0.1)
            # Send another transfer command with 0 bytes to check status
            check_cmd = bytearray(64)
            check_cmd[0] = 0x42
            check_cmd[1] = 0  # 0 bytes = just check status
            resp = send_command(bytes(check_cmd))
            status = resp[3]
            print(f"  Poll {attempt+1}: status=0x{status:02X}")
            if status == 0x00:
                print(f"  RX count: {resp[2]}")
                if resp[2] > 0:
                    rx_data = resp[4:4+resp[2]]
                    print(f"  RX data: {rx_data.hex()}")
                break
    
    rx_count = resp[2]
    print(f"\nRX count: {rx_count}")
    if rx_count > 0:
        rx_data = resp[4:4+rx_count]
        print(f"RX data: {rx_data.hex()}")
    else:
        print("No RX data!")
    
    # Check GPIO after transfer
    print("\n=== GPIO After Transfer ===")
    resp = send_command(bytes([0x31]))
    gpio_vals = resp[4] | (resp[5] << 8)
    for i in range(9):
        print(f"  GP{i}: {'HIGH' if gpio_vals & (1 << i) else 'LOW'}")
    
    device.close()
    print("\nDone!")

if __name__ == "__main__":
    main()
