#!/usr/bin/env python3
"""
Reset and clean test - fully reset MCP2210 before each transfer
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
    print("\n=== Cancel/Reset ===")
    for i in range(5):
        resp = send_command(bytes([0x11]))  # CANCEL_SPI_TRANSFER
        print(f"Cancel {i+1}: resp[1]={resp[1]:02X}")
        time.sleep(0.05)
    
    # Check engine status
    resp = send_command(bytes([0x10]))  # GET_SPI_STATUS
    print(f"SPI engine status: {resp[3]:02X}")
    
    # Check GPIO
    resp = send_command(bytes([0x31]))
    gpio_vals = resp[4] | (resp[5] << 8)
    print(f"GPIO values: 0x{gpio_vals:04X}")
    for i in range(9):
        print(f"  GP{i}: {'HIGH' if gpio_vals & (1 << i) else 'LOW'}")
    
    # Manually set GP0 HIGH (release CS)
    print("\n=== Manually release CS ===")
    gpio_out_cmd = bytearray(64)
    gpio_out_cmd[0] = 0x30  # SET_GPIO_PIN_VAL
    gpio_out_cmd[4] = 0xFF  # All pins HIGH
    gpio_out_cmd[5] = 0x01  # GP8 too
    gpio_out_cmd[6] = 0xFF  # Direction mask - apply to all
    gpio_out_cmd[7] = 0x01
    resp = send_command(bytes(gpio_out_cmd))
    print(f"Response: {resp[1]:02X}")
    
    # Check again
    resp = send_command(bytes([0x31]))
    gpio_vals = resp[4] | (resp[5] << 8)
    print(f"GPIO values after: 0x{gpio_vals:04X}")
    
    # Now close and re-open
    print("\n=== Re-opening device ===")
    device.close()
    time.sleep(0.5)
    device = hid.device()
    device.open(MCP2210_VID, MCP2210_PID)
    print("Reopened")
    
    # Cancel again
    for i in range(3):
        resp = send_command(bytes([0x11]))
        time.sleep(0.05)
    
    # Check GPIO
    resp = send_command(bytes([0x31]))
    gpio_vals = resp[4] | (resp[5] << 8)
    print(f"GPIO values: 0x{gpio_vals:04X}")
    for i in range(9):
        val = 'HIGH' if gpio_vals & (1 << i) else 'LOW'
        print(f"  GP{i}: {val}")
    
    # Read NVRAM chip settings to see what's there
    print("\n=== NVRAM Chip Settings ===")
    resp = send_command(bytes([0x20]))
    designations = resp[4:13]
    for i in range(9):
        d = designations[i]
        names = {0: "GPIO", 1: "CS", 2: "Dedicated"}
        print(f"  GP{i}: {names.get(d, f'Unknown({d})')}")
    default_out = resp[13] | (resp[14] << 8)
    print(f"  Default output: 0x{default_out:04X}")
    
    device.close()
    print("\nDone!")

if __name__ == "__main__":
    main()
