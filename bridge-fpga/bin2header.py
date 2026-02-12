#!/usr/bin/env python3
"""
Convert an iCE40 binary bitstream to a C header file.

Usage:
    python bin2header.py bitstream.bin > bitstream.h

Generates a uint8_t array suitable for ice_cram_write().
"""

import sys

if len(sys.argv) < 2:
    print("Usage: bin2header.py <input.bin>", file=sys.stderr)
    sys.exit(1)

binary_data = open(sys.argv[1], 'rb').read()

# Output as a C array
print("// Auto-generated from iCE40 bitstream - DO NOT EDIT")
print(f"// Size: {len(binary_data)} bytes")
print("")
print("static const uint8_t bitstream[] = {")

# Print 16 bytes per line
for i in range(0, len(binary_data), 16):
    chunk = binary_data[i:i+16]
    hex_str = ", ".join(f"0x{b:02x}" for b in chunk)
    if i + 16 >= len(binary_data):
        print(f"    {hex_str}")
    else:
        print(f"    {hex_str},")

print("};")
