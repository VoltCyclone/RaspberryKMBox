#!/usr/bin/env python3
"""Monitor GP5 (MISO) while RP2040 toggles GPIO8 at boot"""

import hid
import time

device = hid.device()
device.open(0x04D8, 0x00DE)
print('Monitoring GP5 (MISO) - watch for toggling from RP2040...')

def send_command(cmd):
    while len(cmd) < 64:
        cmd = cmd + bytes([0x00])
    device.write(bytes([0x00]) + cmd)
    return bytes(device.read(64))

# Cancel any transfers
send_command(bytes([0x11]))

# Set all GPIO mode
gpio_cmd = bytearray(64)
gpio_cmd[0] = 0x21
for i in range(9):
    gpio_cmd[4+i] = 0x00
gpio_cmd[13] = 0xFF
gpio_cmd[14] = 0x01
send_command(bytes(gpio_cmd))

print('Reading GP5 every 50ms for 5 seconds...')
last_val = None
for i in range(100):
    resp = send_command(bytes([0x31]))
    gpio_vals = resp[4] | (resp[5] << 8)
    gp5 = (gpio_vals >> 5) & 1
    if gp5 != last_val:
        print(f't={i*50:4d}ms: GP5(MISO)={gp5} (changed)')
        last_val = gp5
    elif i % 20 == 0:
        print(f't={i*50:4d}ms: GP5(MISO)={gp5}')
    time.sleep(0.05)

device.close()
print('Done')
