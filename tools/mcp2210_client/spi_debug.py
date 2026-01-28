#!/usr/bin/env python3
"""
Simple SPI Debug Script
Send raw bytes and observe responses
"""

import hid
import time

MCP2210_VID = 0x04D8
MCP2210_PID = 0x00DE

def main():
    print("Opening MCP2210...")
    device = hid.device()
    device.open(MCP2210_VID, MCP2210_PID)
    print(f"Opened: {device.get_manufacturer_string()} - {device.get_product_string()}")
    
    # Set non-blocking mode
    device.set_nonblocking(0)
    
    def send_command(cmd):
        """Send HID command and get response."""
        # Pad to 64 bytes
        while len(cmd) < 64:
            cmd = cmd + bytes([0x00])
        device.write(bytes([0x00]) + cmd)  # Report ID 0
        return bytes(device.read(64))
    
    # Get chip status
    print("\n--- Getting Chip Status ---")
    resp = send_command(bytes([0x10]))  # GET_CHIP_STATUS
    print(f"Status: {resp[:10].hex()}")
    print(f"  SPI Status: {resp[1]}")
    print(f"  Bus Owner: {resp[2]}")
    
    # Configure SPI settings for 1 MHz, Mode 0
    print("\n--- Configuring SPI Settings ---")
    spi_settings = bytes([
        0x40,  # SET_SPI_SETTING
        0x00, 0x00, 0x00, 0x00,  # Reserved
        # Bit rate: 1 MHz = 1000000 = 0x000F4240
        0x40, 0x42, 0x0F, 0x00,  # Little-endian
        # Idle CS: all high
        0xFF, 0x01,
        # Active CS: GP0 low
        0xFE, 0x01,
        # CS to data delay (10 * 100ns = 1us)
        0x0A, 0x00,
        # Last data to CS delay
        0x0A, 0x00,
        # Inter-byte delay
        0x00, 0x00,
        # Bytes per transaction
        0x08, 0x00,
        # SPI mode
        0x00,  # Mode 0
    ])
    resp = send_command(spi_settings)
    print(f"SPI config response: {resp[:10].hex()}")
    
    # Configure GPIO: GP0 as CS, rest as GPIO
    print("\n--- Configuring GPIO ---")
    gpio_settings = bytes([
        0x21,  # SET_GPIO_SETTING
        0x00, 0x00, 0x00, 0x00,  # Reserved
        # GP0-GP8 designation (0=GPIO, 1=CS)
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        # GP0-GP8 default output value
        0xFF, 0x01,
        # GP0-GP8 default direction (0=output, 1=input)
        0x00, 0x00,
    ])
    resp = send_command(gpio_settings)
    print(f"GPIO config response: {resp[:10].hex()}")
    
    # Do a simple SPI transfer
    print("\n--- SPI Transfer Test ---")
    
    # Data to send: PING command
    tx_data = bytes([0xF0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07])
    
    print(f"Sending: {tx_data.hex()}")
    
    spi_cmd = bytes([
        0x42,  # SPI_TRANSFER
        len(tx_data),  # Byte count
        0x00, 0x00,  # Reserved
    ]) + tx_data
    
    resp = send_command(spi_cmd)
    print(f"Response: {resp[:20].hex()}")
    print(f"  Status: {resp[1]} (0=finished, 0x10=started, 0x20=not finished, 0x30=busy)")
    print(f"  RX count: {resp[2]}")
    print(f"  Engine status: {resp[3]}")
    print(f"  RX data: {resp[4:4+resp[2]].hex() if resp[2] > 0 else 'none'}")
    
    # Continue reading if transfer not finished
    while resp[1] in (0x10, 0x20):  # STARTED or NOT_FINISHED
        print("  ... continuing transfer")
        resp = send_command(bytes([0x42, 0, 0, 0]))
        print(f"  Status: {resp[1]}, RX count: {resp[2]}")
        if resp[2] > 0:
            print(f"  RX data: {resp[4:4+resp[2]].hex()}")
    
    # Do several more transfers to see if anything changes
    print("\n--- Multiple transfers ---")
    for i in range(5):
        tx_data = bytes([0xF0, i, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        spi_cmd = bytes([0x42, len(tx_data), 0x00, 0x00]) + tx_data
        resp = send_command(spi_cmd)
        
        # Read all response bytes
        status = resp[1]
        all_rx = bytearray()
        if resp[2] > 0:
            all_rx.extend(resp[4:4+resp[2]])
        
        while status in (0x10, 0x20):
            resp = send_command(bytes([0x42, 0, 0, 0]))
            status = resp[1]
            if resp[2] > 0:
                all_rx.extend(resp[4:4+resp[2]])
        
        print(f"Transfer {i}: TX={tx_data.hex()} RX={bytes(all_rx).hex() if all_rx else 'zeros'}")
    
    device.close()
    print("\nDone!")

if __name__ == "__main__":
    main()
