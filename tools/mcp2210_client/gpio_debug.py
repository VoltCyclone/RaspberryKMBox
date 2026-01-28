#!/usr/bin/env python3
"""
GPIO Debug Script - Check pin states
"""

import hid
import time

MCP2210_VID = 0x04D8
MCP2210_PID = 0x00DE

def main():
    print("Opening MCP2210...")
    device = hid.device()
    device.open(MCP2210_VID, MCP2210_PID)
    print(f"Opened: {device.get_manufacturer_string()}")
    
    def send_command(cmd):
        while len(cmd) < 64:
            cmd = cmd + bytes([0x00])
        device.write(bytes([0x00]) + cmd)
        return bytes(device.read(64))
    
    # Get GPIO pin values
    print("\n--- GPIO Pin Values ---")
    resp = send_command(bytes([0x30]))  # GET_GPIO_PIN_VAL
    print(f"Response: {resp[:15].hex()}")
    pin_val = resp[4] | (resp[5] << 8)
    print(f"GPIO values: {bin(pin_val)} = 0x{pin_val:04X}")
    for i in range(9):
        val = (pin_val >> i) & 1
        print(f"  GP{i}: {val}")
    
    # Get GPIO pin directions
    print("\n--- GPIO Pin Directions ---")
    resp = send_command(bytes([0x32]))  # GET_GPIO_PIN_DIR
    print(f"Response: {resp[:15].hex()}")
    pin_dir = resp[4] | (resp[5] << 8)
    print(f"GPIO directions: {bin(pin_dir)} = 0x{pin_dir:04X}")
    for i in range(9):
        d = (pin_dir >> i) & 1
        print(f"  GP{i}: {'input' if d else 'output'}")
    
    # Get current SPI settings
    print("\n--- Current SPI Settings ---")
    resp = send_command(bytes([0x41]))  # GET_SPI_SETTING
    print(f"Response: {resp[:25].hex()}")
    bit_rate = resp[4] | (resp[5] << 8) | (resp[6] << 16) | (resp[7] << 24)
    print(f"Bit rate: {bit_rate} Hz")
    idle_cs = resp[8] | (resp[9] << 8)
    active_cs = resp[10] | (resp[11] << 8)
    print(f"Idle CS: 0x{idle_cs:04X}")
    print(f"Active CS: 0x{active_cs:04X}")
    cs_to_data = resp[12] | (resp[13] << 8)
    data_to_cs = resp[14] | (resp[15] << 8)
    inter_byte = resp[16] | (resp[17] << 8)
    print(f"CS-to-data delay: {cs_to_data * 100}ns")
    print(f"Data-to-CS delay: {data_to_cs * 100}ns")
    print(f"Inter-byte delay: {inter_byte * 100}ns")
    bytes_per_xfer = resp[18] | (resp[19] << 8)
    spi_mode = resp[20]
    print(f"Bytes per transfer: {bytes_per_xfer}")
    print(f"SPI mode: {spi_mode}")
    
    # Cancel any pending transfers
    print("\n--- Canceling pending transfers ---")
    resp = send_command(bytes([0x11]))  # CANCEL_SPI_TRANSFER
    print(f"Cancel response: {resp[:5].hex()}")
    
    # Release the bus
    time.sleep(0.1)
    
    # Get chip status
    print("\n--- Getting Chip Status ---")
    resp = send_command(bytes([0x10]))  # GET_CHIP_STATUS
    print(f"Status: {resp[:10].hex()}")
    print(f"  SPI bus release status: {resp[1]}")
    print(f"  Bus owner: {resp[2]}")
    
    # Configure properly
    print("\n--- Configuring SPI ---")
    spi_settings = bytearray(64)
    spi_settings[0] = 0x40  # SET_SPI_SETTING
    # Bit rate = 100 kHz = 100000 = 0x000186A0 (slow for debugging)
    spi_settings[4] = 0xA0
    spi_settings[5] = 0x86
    spi_settings[6] = 0x01
    spi_settings[7] = 0x00
    # Idle CS = all high (0x01FF for 9 GPIOs)
    spi_settings[8] = 0xFF
    spi_settings[9] = 0x01
    # Active CS = GP0 low (0x01FE)
    spi_settings[10] = 0xFE
    spi_settings[11] = 0x01
    # CS-to-data delay (100 = 10us)
    spi_settings[12] = 100
    spi_settings[13] = 0x00
    # Data-to-CS delay
    spi_settings[14] = 100
    spi_settings[15] = 0x00
    # Inter-byte delay
    spi_settings[16] = 0x00
    spi_settings[17] = 0x00
    # Bytes per transfer = 8
    spi_settings[18] = 8
    spi_settings[19] = 0x00
    # SPI mode 0
    spi_settings[20] = 0x00
    
    resp = send_command(bytes(spi_settings))
    status_code = resp[1]
    print(f"Config response: {resp[:5].hex()}, status={status_code} (0=success)")
    if status_code == 0xF8:
        print("  ERROR: SPI bus not available")
    elif status_code == 0xFB:
        print("  ERROR: SPI transfer in progress")
    
    # Configure GPIO for CS output
    print("\n--- Configuring GPIO ---")
    gpio_settings = bytearray(64)
    gpio_settings[0] = 0x21  # SET_GPIO_SETTING
    # Designation: GP0 = CS (1), rest = GPIO (0)
    gpio_settings[4] = 0x01  # GP0 = CS
    # Default output value (all high)
    gpio_settings[13] = 0xFF
    gpio_settings[14] = 0x01
    # Default direction (GP0 = output)
    gpio_settings[15] = 0xFE
    gpio_settings[16] = 0x01
    
    resp = send_command(bytes(gpio_settings))
    print(f"GPIO config response: {resp[:5].hex()}")
    
    # Read settings back
    print("\n--- Verify Settings ---")
    resp = send_command(bytes([0x41]))  # GET_SPI_SETTING
    bit_rate = resp[4] | (resp[5] << 8) | (resp[6] << 16) | (resp[7] << 24)
    spi_mode = resp[20]
    print(f"Bit rate: {bit_rate} Hz, Mode: {spi_mode}")
    
    # Now do an SPI transfer
    print("\n--- SPI Transfer with longer delays ---")
    tx_data = bytes([0xF0, 0xAA, 0x55, 0x12, 0x34, 0x56, 0x78, 0x9A])
    spi_cmd = bytearray(64)
    spi_cmd[0] = 0x42  # SPI_TRANSFER
    spi_cmd[1] = len(tx_data)
    spi_cmd[4:4+len(tx_data)] = tx_data
    
    print(f"Sending: {tx_data.hex()}")
    resp = send_command(bytes(spi_cmd))
    
    print(f"Full response: {resp[:32].hex()}")
    
    status = resp[1]
    rx_len = resp[2]
    engine_status = resp[3]
    
    print(f"Status: {status} (0=finished, 0x10=started, 0x20=not_finished)")
    print(f"RX length: {rx_len}")
    print(f"Engine status: {engine_status}")
    
    if rx_len > 0:
        print(f"RX data: {resp[4:4+rx_len].hex()}")
    
    # Continue if needed
    all_rx = bytearray()
    if rx_len > 0:
        all_rx.extend(resp[4:4+rx_len])
    
    while status in (0x10, 0x20):
        resp = send_command(bytes([0x42, 0, 0, 0]))
        status = resp[1]
        rx_len = resp[2]
        if rx_len > 0:
            all_rx.extend(resp[4:4+rx_len])
            print(f"  Got {rx_len} more bytes: {resp[4:4+rx_len].hex()}")
    
    print(f"Total RX: {bytes(all_rx).hex() if all_rx else 'none'}")
    
    device.close()
    print("\nDone!")

if __name__ == "__main__":
    main()
