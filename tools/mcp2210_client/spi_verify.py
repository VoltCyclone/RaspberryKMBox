#!/usr/bin/env python3
"""
Manually toggle CS and check if SPI works
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
    
    # First, check GPIO current settings
    print("\n=== Current GPIO Settings ===")
    resp = send_command(bytes([0x20]))  # GET_GPIO_SETTING (same as GET_CHIP_SETTING)
    print(f"Response: {resp[:25].hex()}")
    # Byte 4-12: GPIO designation
    # Byte 13-14: Default GPIO output
    # Byte 15-16: Default GPIO direction
    designations = resp[4:13]
    print(f"GPIO designations: {designations.hex()}")
    for i in range(9):
        d = designations[i]
        names = {0: "GPIO", 1: "CS", 2: "Dedicated"}
        print(f"  GP{i}: {names.get(d, f'Unknown({d})')}")
    
    # Configure all GPIOs (GP0 as CS)
    print("\n=== Configuring GPIO ===")
    gpio_cmd = bytearray(64)
    gpio_cmd[0] = 0x21  # SET_GPIO_SETTING
    # GP0 = CS, GP1-8 = GPIO
    gpio_cmd[4] = 0x01  # GP0 = CS
    for i in range(1, 9):
        gpio_cmd[4+i] = 0x00  # GPx = GPIO
    # Default output (all high)
    gpio_cmd[13] = 0xFF
    gpio_cmd[14] = 0x01
    # Default direction (all output)
    gpio_cmd[15] = 0x00
    gpio_cmd[16] = 0x00
    resp = send_command(bytes(gpio_cmd))
    print(f"GPIO config response: {resp[:5].hex()}")
    
    # Configure SPI
    print("\n=== Configuring SPI ===")
    spi_cmd = bytearray(64)
    spi_cmd[0] = 0x40  # SET_SPI_SETTING
    # 100 kHz
    spi_cmd[4:8] = [0xA0, 0x86, 0x01, 0x00]
    # Idle CS = all high (0x01FF)
    spi_cmd[8:10] = [0xFF, 0x01]
    # Active CS = GP0 low (0x01FE)
    spi_cmd[10:12] = [0xFE, 0x01]
    # CS-to-data delay
    spi_cmd[12:14] = [0x64, 0x00]  # 10us
    spi_cmd[14:16] = [0x64, 0x00]  # 10us
    # Inter-byte delay
    spi_cmd[16:18] = [0x00, 0x00]
    # Bytes per transfer (8)
    spi_cmd[18:20] = [0x08, 0x00]
    # Mode 0
    spi_cmd[20] = 0x00
    resp = send_command(bytes(spi_cmd))
    print(f"SPI config response: {resp[:5].hex()}")
    
    # Verify settings
    resp = send_command(bytes([0x41]))  # GET_SPI_SETTING
    print(f"SPI settings verification:")
    print(f"  Bit rate: {resp[4]|(resp[5]<<8)|(resp[6]<<16)|(resp[7]<<24)} Hz")
    print(f"  Idle CS: 0x{resp[8]|resp[9]<<8:04X}")
    print(f"  Active CS: 0x{resp[10]|resp[11]<<8:04X}")
    print(f"  Bytes/xfer: {resp[18]|resp[19]<<8}")
    print(f"  Mode: {resp[20]}")
    
    # Do the SPI transfer
    print("\n=== SPI Transfer ===")
    tx_data = bytes([0xF0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07])
    
    xfer_cmd = bytearray(64)
    xfer_cmd[0] = 0x42  # SPI_TRANSFER
    xfer_cmd[1] = len(tx_data)  # Byte count
    xfer_cmd[4:4+len(tx_data)] = tx_data
    
    print(f"Sending: {tx_data.hex()}")
    resp = send_command(bytes(xfer_cmd))
    
    print(f"Response hex dump:")
    for i in range(0, 64, 16):
        hex_str = ' '.join(f'{b:02x}' for b in resp[i:i+16])
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in resp[i:i+16])
        print(f"  {i:02x}: {hex_str}  |{ascii_str}|")
    
    status = resp[1]
    rx_count = resp[2]
    engine = resp[3]
    
    status_names = {0x00: "FINISHED", 0x10: "STARTED", 0x20: "NOT_FINISHED", 
                    0x30: "NOT_AVAILABLE", 0xF8: "BUS_BUSY"}
    
    print(f"Status: 0x{status:02X} ({status_names.get(status, 'UNKNOWN')})")
    print(f"RX count: {rx_count}")
    print(f"Engine: 0x{engine:02X}")
    
    if rx_count > 0:
        print(f"RX data: {resp[4:4+rx_count].hex()}")
    
    # Continue polling if needed
    total_rx = bytearray()
    if rx_count > 0:
        total_rx.extend(resp[4:4+rx_count])
        
    poll = 0
    while status in (0x10, 0x20) and poll < 50:
        poll += 1
        resp = send_command(bytes([0x42, 0, 0, 0]))
        status = resp[1]
        rx_count = resp[2]
        if rx_count > 0:
            total_rx.extend(resp[4:4+rx_count])
            print(f"Poll {poll}: {rx_count} bytes: {resp[4:4+rx_count].hex()}")
    
    print(f"\nTotal RX: {bytes(total_rx).hex() if total_rx else '(empty)'}")
    
    # Final status check
    resp = send_command(bytes([0x10]))  # GET_CHIP_STATUS
    print(f"\nFinal chip status: bus_release={resp[1]}, owner={resp[2]}")
    
    device.close()
    print("Done!")

if __name__ == "__main__":
    main()
