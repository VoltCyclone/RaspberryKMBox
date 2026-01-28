#!/usr/bin/env python3
"""
GPIO toggle test - manually toggle each GPIO to see which ones are connected
to what on the Click Shield
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
    
    # Set ALL pins to GPIO mode (no dedicated functions)
    print("\n=== Setting ALL pins to GPIO mode ===")
    gpio_cmd = bytearray(64)
    gpio_cmd[0] = 0x21  # SET_CHIP_SETTINGS
    for i in range(9):
        gpio_cmd[4+i] = 0x00  # GPIO mode
    # Default output (all HIGH)
    gpio_cmd[13] = 0xFF
    gpio_cmd[14] = 0x01
    # All outputs (direction mask = 0 means output)
    gpio_cmd[15] = 0x00
    gpio_cmd[16] = 0x00
    resp = send_command(bytes(gpio_cmd))
    print(f"Response: {resp[1]:02X}")
    
    # Verify
    resp = send_command(bytes([0x20]))
    print("Designations:", [resp[4+i] for i in range(9)])
    
    # Now toggle each pin one at a time
    print("\n=== GPIO Toggle Test ===")
    print("Watch with scope/multimeter. Press Enter after each toggle.")
    
    for pin in range(9):
        print(f"\n--- GP{pin} ---")
        
        # Set this pin LOW, others HIGH
        gpio_out = bytearray(64)
        gpio_out[0] = 0x30  # SET_GPIO_PIN_VAL
        val = 0x01FF & ~(1 << pin)  # All high except this pin
        gpio_out[4] = val & 0xFF
        gpio_out[5] = (val >> 8) & 0xFF
        gpio_out[6] = 0xFF  # Direction mask - apply to all
        gpio_out[7] = 0x01
        resp = send_command(bytes(gpio_out))
        
        # Read back
        resp = send_command(bytes([0x31]))
        gpio_vals = resp[4] | (resp[5] << 8)
        print(f"  GP{pin} should be LOW: actual=0x{gpio_vals:04X}")
        for i in range(9):
            val = 'HIGH' if gpio_vals & (1 << i) else 'LOW'
            expected = 'LOW' if i == pin else 'HIGH'
            match = '✓' if val == expected else '✗'
            print(f"    GP{i}: {val} (expected {expected}) {match}")
        
        input(f"  Press Enter to continue...")
        
        # Set back to HIGH
        gpio_out[4] = 0xFF
        gpio_out[5] = 0x01
        send_command(bytes(gpio_out))
    
    # Now do a quick blink test on the SPI pins
    print("\n=== Quick SPI Pin Test ===")
    
    # These should be the SPI pins based on MCP2210 datasheet:
    # GP0 - can be CS
    # GP4 - SCK when dedicated
    # GP5 - MOSI (SDI to slave) when dedicated  
    # GP6 - MISO (SDO from slave) when dedicated
    
    # Actually let me check the datasheet again...
    # From MCP2210 datasheet:
    # GP0-GP8 can be GPIO or CS
    # GP0 is also used for SPI transfer complete
    # The dedicated function pins:
    #   GP5 = CS (if used for dedicated)
    #   Actually no - looking at page 27:
    #     GP5 = MISO (master in, slave out) 
    #     GP6 = MOSI (master out, slave in)
    #     GP7 = SCK
    # But wait, the MCP2210 is a USB-to-SPI MASTER
    # So MOSI = data from MCP2210 to slave (this is GP6)
    # And MISO = data from slave to MCP2210 (this is GP5)
    
    print("According to MCP2210 datasheet (when GP5-7 are Dedicated):")
    print("  GP5 = MISO (slave → master)")
    print("  GP6 = MOSI (master → slave)")  
    print("  GP7 = SCK")
    print("  GP0-GP4, GP8 can be CS")
    
    print("\nNow testing SPI with GP0 as CS...")
    
    # Configure for SPI
    gpio_cmd = bytearray(64)
    gpio_cmd[0] = 0x21
    gpio_cmd[4] = 0x01   # GP0 = CS
    gpio_cmd[5] = 0x00   # GP1 = GPIO
    gpio_cmd[6] = 0x00   # GP2 = GPIO
    gpio_cmd[7] = 0x00   # GP3 = GPIO
    gpio_cmd[8] = 0x00   # GP4 = GPIO
    gpio_cmd[9] = 0x02   # GP5 = Dedicated (MISO)
    gpio_cmd[10] = 0x02  # GP6 = Dedicated (MOSI)
    gpio_cmd[11] = 0x02  # GP7 = Dedicated (SCK)
    gpio_cmd[12] = 0x00  # GP8 = GPIO
    gpio_cmd[13] = 0xFF
    gpio_cmd[14] = 0x01
    gpio_cmd[15] = 0x00
    gpio_cmd[16] = 0x00
    resp = send_command(bytes(gpio_cmd))
    print(f"Set chip settings: {resp[1]:02X}")
    
    # Verify
    resp = send_command(bytes([0x20]))
    designations = resp[4:13]
    print("Designations after:", list(designations))
    
    # Set SPI settings
    spi_cmd = bytearray(64)
    spi_cmd[0] = 0x40
    # 100 kHz
    spi_cmd[4:8] = [0xA0, 0x86, 0x01, 0x00]
    # Idle CS (all high)
    spi_cmd[8:10] = [0xFF, 0x01]
    # Active CS (GP0 low)
    spi_cmd[10:12] = [0xFE, 0x01]
    # Delays (100us each)
    spi_cmd[12:14] = [0xE8, 0x03]
    spi_cmd[14:16] = [0xE8, 0x03]
    spi_cmd[16:18] = [0x00, 0x00]
    # 8 bytes
    spi_cmd[18:20] = [0x08, 0x00]
    # Mode 0
    spi_cmd[20] = 0x00
    resp = send_command(bytes(spi_cmd))
    print(f"Set SPI settings: {resp[1]:02X}")
    
    # Now try a transfer
    print("\nDoing SPI transfer...")
    tx_data = bytes([0xAA, 0x55, 0xAA, 0x55, 0x00, 0x00, 0x00, 0x00])
    xfer_cmd = bytearray(64)
    xfer_cmd[0] = 0x42
    xfer_cmd[1] = len(tx_data)
    xfer_cmd[4:4+len(tx_data)] = tx_data
    
    resp = send_command(bytes(xfer_cmd))
    status = resp[3]
    print(f"Transfer status: 0x{status:02X}")
    
    # Poll for completion
    for i in range(20):
        time.sleep(0.1)
        check = bytearray(64)
        check[0] = 0x42
        check[1] = 0
        resp = send_command(bytes(check))
        status = resp[3]
        rx_count = resp[2]
        print(f"  Poll {i+1}: status=0x{status:02X}, rx_count={rx_count}")
        if status == 0x00 and rx_count > 0:
            print(f"  RX: {resp[4:4+rx_count].hex()}")
            break
    
    device.close()
    print("\nDone!")

if __name__ == "__main__":
    main()
