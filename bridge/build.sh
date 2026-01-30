#!/bin/bash
# Build script for Adafruit Feather RP2350 bridge firmware

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

# Clean if requested
if [ "$1" = "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
echo "Configuring..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
echo "Building..."
make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)

# Report
if [ -f "kmbox_bridge.uf2" ]; then
    echo ""
    echo "=========================================="
    echo "Build successful!"
    echo "Firmware: $BUILD_DIR/kmbox_bridge.uf2"
    echo ""
    echo "To flash:"
    echo "  1. Hold BOOTSEL button on Feather RP2350"
    echo "  2. Plug in USB"
    echo "  3. Copy kmbox_bridge.uf2 to RPI-RP2 drive"
    echo "=========================================="
else
    echo "Build failed!"
    exit 1
fi
