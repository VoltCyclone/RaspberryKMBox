#!/bin/bash

# PIO KMbox Build Script
# Builds main KMBox firmware and adafruit RP2350 bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PICO_SDK_PATH="${PICO_SDK_PATH:-$HOME/.pico-sdk/sdk/2.2.0-fresh}"
PICOTOOL="${PICOTOOL:-$HOME/.pico-sdk/picotool/2.1.1/picotool/picotool}"
NJOBS=$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

usage() {
    echo "Usage: $0 [target] [options]"
    echo ""
    echo "Targets:"
    echo "  pico      Build main KMBox for RP2040 (Pico)"
    echo "  pico2     Build main KMBox for RP2350 (Pico 2)"
    echo "  bridge    Build UART bridge for adafruit RP2350"
    echo "  both      Build main KMBox for both RP2040 and RP2350"
    echo "  all       Build and flash KMBox + Bridge (interactive)"
    echo ""
    echo "Options:"
    echo "  clean     Clean build directories before building"
    echo "  flash     Flash firmware after building"
    echo ""
    echo "Examples:"
    echo "  $0 pico              # Build for RP2040"
    echo "  $0 pico2 flash       # Build and flash for RP2350"
    echo "  $0 bridge            # Build UART bridge"
    echo "  $0 all               # Build & flash KMBox + Bridge (guided)"
    echo "  $0 both clean        # Clean build both KMBox variants"
}

build_kmbox() {
    local target=$1
    local build_dir="$SCRIPT_DIR/build-$target"
    local board=""
    local platform=""
    
    case $target in
        pico)
            board="pico"
            platform="rp2040"
            ;;
        pico2)
            board="pico2"
            platform="rp2350-arm-s"
            ;;
        *)
            echo -e "${RED}Unknown target: $target${NC}"
            return 1
            ;;
    esac
    
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Building KMBox for $target ($platform)${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ "$CLEAN" = "1" ]; then
        echo "Cleaning $build_dir..."
        rm -rf "$build_dir"
    fi
    
    mkdir -p "$build_dir"
    cd "$build_dir"
    
    echo "Configuring..."
    PICO_SDK_PATH="$PICO_SDK_PATH" cmake "$SCRIPT_DIR" \
        -DPICO_BOARD="$board" \
        -DPICO_PLATFORM="$platform" \
        -DCMAKE_BUILD_TYPE=Release
    
    echo "Building..."
    make -j"$NJOBS"
    
    if [ -f "PIOKMbox.uf2" ]; then
        local size=$(ls -la PIOKMbox.uf2 | awk '{print $5}')
        echo -e "${GREEN}✓ Built: $build_dir/PIOKMbox.uf2 ($size bytes)${NC}"
        return 0
    else
        echo -e "${RED}✗ Build failed for $target${NC}"
        return 1
    fi
}

build_bridge() {
    local build_dir="$SCRIPT_DIR/bridge/build"
    
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Building UART Bridge for adafruit RP2350${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ "$CLEAN" = "1" ]; then
        echo "Cleaning $build_dir..."
        rm -rf "$build_dir"
    fi
    
    mkdir -p "$build_dir"
    cd "$build_dir"
    
    echo "Configuring..."
    PICO_SDK_PATH="$PICO_SDK_PATH" cmake "$SCRIPT_DIR/bridge" \
        -DCMAKE_BUILD_TYPE=Release
    
    echo "Building..."
    make -j"$NJOBS"
    
    if [ -f "kmbox_bridge.uf2" ]; then
        local size=$(ls -la kmbox_bridge.uf2 | awk '{print $5}')
        echo -e "${GREEN}✓ Built: $build_dir/kmbox_bridge.uf2 ($size bytes)${NC}"
        return 0
    else
        echo -e "${RED}✗ Bridge build failed${NC}"
        return 1
    fi
}

wait_for_device() {
    local name=$1
    echo ""
    echo -e "${CYAN}────────────────────────────────────────${NC}"
    echo -e "${YELLOW}Ready to flash: $name${NC}"
    echo -e "${CYAN}────────────────────────────────────────${NC}"
    echo ""
    echo "  1. Hold BOOTSEL button on the device"
    echo "  2. Connect USB (or tap reset while holding BOOTSEL)"
    echo "  3. Release BOOTSEL - device should appear as RPI-RP2"
    echo ""
    echo -e "${YELLOW}Press ENTER when device is in BOOTSEL mode...${NC}"
    read -r
}

flash_firmware() {
    local uf2_file=$1
    local name=$2
    
    if [ ! -f "$uf2_file" ]; then
        echo -e "${RED}Firmware not found: $uf2_file${NC}"
        return 1
    fi
    
    echo -e "${BLUE}Flashing $name...${NC}"
    
    if [ -x "$PICOTOOL" ]; then
        if "$PICOTOOL" load "$uf2_file" -fx 2>/dev/null; then
            echo -e "${GREEN}✓ Flashed $name successfully!${NC}"
            sleep 1
            return 0
        fi
    fi
    
    if command -v picotool &>/dev/null; then
        if picotool load "$uf2_file" -fx 2>/dev/null; then
            echo -e "${GREEN}✓ Flashed $name successfully!${NC}"
            sleep 1
            return 0
        fi
    fi
    
    # Fallback: try to copy to mounted drive
    for mount in /Volumes/RPI-RP2 /media/*/RPI-RP2 /run/media/*/RPI-RP2; do
        if [ -d "$mount" ]; then
            echo "Copying to $mount..."
            cp "$uf2_file" "$mount/"
            echo -e "${GREEN}✓ Copied to $mount - device will reboot${NC}"
            sleep 2
            return 0
        fi
    done
    
    echo -e "${RED}✗ Could not flash. Please manually copy:${NC}"
    echo "  $uf2_file"
    echo "  to the RPI-RP2 drive"
    return 1
}

# Parse arguments
TARGET=""
CLEAN=0
FLASH=0

for arg in "$@"; do
    case $arg in
        pico|pico2|bridge|both|all)
            TARGET="$arg"
            ;;
        clean)
            CLEAN=1
            ;;
        flash)
            FLASH=1
            ;;
        -h|--help|help)
            usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown argument: $arg${NC}"
            usage
            exit 1
            ;;
    esac
done

# Default target
if [ -z "$TARGET" ]; then
    TARGET="all"
fi

# Execute build
cd "$SCRIPT_DIR"

case $TARGET in
    pico)
        build_kmbox pico
        if [ "$FLASH" = "1" ]; then
            wait_for_device "KMBox (RP2040)"
            flash_firmware "$SCRIPT_DIR/build-pico/PIOKMbox.uf2" "KMBox (RP2040)"
        fi
        ;;
    pico2)
        build_kmbox pico2
        if [ "$FLASH" = "1" ]; then
            wait_for_device "KMBox (RP2350)"
            flash_firmware "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" "KMBox (RP2350)"
        fi
        ;;
    bridge)
        build_bridge
        if [ "$FLASH" = "1" ]; then
            wait_for_device "UART Bridge (adafruit RP2350)"
            flash_firmware "$SCRIPT_DIR/bridge/build/kmbox_bridge.uf2" "UART Bridge"
        fi
        ;;
    both)
        build_kmbox pico
        build_kmbox pico2
        if [ "$FLASH" = "1" ]; then
            echo ""
            echo -e "${YELLOW}Which KMBox device to flash?${NC}"
            echo "1) RP2040 (Pico)"
            echo "2) RP2350 (Pico 2)"
            echo "3) Both (one at a time)"
            echo "4) Skip flashing"
            read -rp "Choice [1-4]: " choice
            case $choice in
                1)
                    wait_for_device "KMBox (RP2040)"
                    flash_firmware "$SCRIPT_DIR/build-pico/PIOKMbox.uf2" "KMBox (RP2040)"
                    ;;
                2)
                    wait_for_device "KMBox (RP2350)"
                    flash_firmware "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" "KMBox (RP2350)"
                    ;;
                3)
                    wait_for_device "KMBox (RP2040)"
                    flash_firmware "$SCRIPT_DIR/build-pico/PIOKMbox.uf2" "KMBox (RP2040)"
                    wait_for_device "KMBox (RP2350)"
                    flash_firmware "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" "KMBox (RP2350)"
                    ;;
            esac
        fi
        ;;
    all)
        # Build KMBox first (RP2040 USB Host)
        build_kmbox pico
        
        # Wait and flash KMBox
        wait_for_device "KMBox (RP2040 / Feather USB Host)"
        flash_firmware "$SCRIPT_DIR/build-pico/PIOKMbox.uf2" "KMBox Firmware"
        
        echo ""
        echo -e "${GREEN}KMBox flashed! Now building bridge...${NC}"
        echo ""
        
        # Build bridge
        build_bridge
        
        # Wait and flash bridge
        wait_for_device "UART Bridge (adafruit RP2350)"
        flash_firmware "$SCRIPT_DIR/bridge/build/kmbox_bridge.uf2" "UART Bridge"
        
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}Both devices flashed successfully!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo "Wiring reminder (adafruit to KMBox):"
        echo "  adafruit GPIO0 (TX) ────► KMBox GPIO0 (RX)"
        echo "  adafruit GPIO1 (RX) ◄──── KMBox GPIO1 (TX)"
        echo "  adafruit GND        ────  KMBox GND"
        echo ""
        echo "Test with: python bridge/bridge_client.py --test"
        exit 0
        ;;
esac

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Done!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Firmware locations:"
[ -f "$SCRIPT_DIR/build-pico/PIOKMbox.uf2" ] && echo "  KMBox (RP2040):  build-pico/PIOKMbox.uf2"
[ -f "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" ] && echo "  KMBox (RP2350):  build-pico2/PIOKMbox.uf2"
[ -f "$SCRIPT_DIR/bridge/build/kmbox_bridge.uf2" ] && echo "  UART Bridge:     bridge/build/kmbox_bridge.uf2"