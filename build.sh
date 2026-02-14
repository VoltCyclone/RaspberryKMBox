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
    echo "Targets (FPGA setup):"
    echo "  (default)       Build & flash FPGA bridge + Metro broken [default]"
    echo "  bridge-fpga     Build FPGA bridge for pico2-ice (iCE40 + RP2350)"
    echo "  metro-broken    Build KMBox for Metro RP2350 (broken D0-D7, SPI input)"
    echo ""
    echo "Targets (other boards):"
    echo "  metro           Build main KMBox for Adafruit Metro RP2350"
    echo "  pico2           Build main KMBox for RP2350 (Pico 2)"
    echo "  xiao            Build main KMBox for Seeed XIAO RP2350"
    echo "  bridge          Build UART bridge for Metro RP2350 + ILI9341"
    echo "  bridge-feather  Build UART bridge for Feather RP2350 + ST7735"
    echo "  both            Build KMBox for Metro RP2350 and Pico 2"
    echo "  flash-metros    Build and flash both Metros (UART bridge setup)"
    echo ""
    echo "Options:"
    echo "  clean     Clean build directories before building"
    echo "  flash     Flash firmware after building"
    echo ""
    echo "Examples:"
    echo "  $0                   # Build & flash FPGA bridge + Metro broken (default)"
    echo "  $0 bridge-fpga       # Build FPGA bridge only"
    echo "  $0 metro-broken      # Build Metro broken KMBox only"
    echo "  $0 bridge-fpga flash # Build and flash FPGA bridge"
    echo "  $0 pico2 flash       # Build and flash for RP2350 (Pico 2)"
    echo "  $0 both clean        # Clean build Metro + Pico 2"
}

build_kmbox() {
    local target=$1
    local build_dir="$SCRIPT_DIR/build-$target"
    local board=""
    local platform=""
    local extra_cmake=""
    
    case $target in
        pico2)
            board="pico2"
            platform="rp2350-arm-s"
            ;;
        metro)
            board="adafruit_metro_rp2350"
            platform="rp2350-arm-s"
            ;;
        metro-broken)
            board="adafruit_metro_rp2350"
            platform="rp2350-arm-s"
            extra_cmake="-DMETRO_BROKEN_PINS=ON"
            ;;
        xiao)
            board="seeed_xiao_rp2350"
            platform="rp2350-arm-s"
            ;;
        *)
            echo -e "${RED}Unknown target: $target${NC}"
            return 1
            ;;
    esac
    
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Building KMBox for $target ($board / $platform)${NC}"
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
        -DCMAKE_BUILD_TYPE=Release \
        ${extra_cmake:-}
    
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
    local target=${1:-feather}
    local build_dir=""
    local board=""
    
    case $target in
        feather)
            build_dir="$SCRIPT_DIR/bridge/build"
            board="adafruit_feather_rp2350"
            ;;
        metro)
            build_dir="$SCRIPT_DIR/bridge/build-metro"
            board="adafruit_metro_rp2350"
            ;;
        *)
            echo -e "${RED}Unknown bridge target: $target${NC}"
            return 1
            ;;
    esac
    
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Building UART Bridge for $board${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ "$CLEAN" = "1" ]; then
        echo "Cleaning $build_dir..."
        rm -rf "$build_dir"
    fi
    
    mkdir -p "$build_dir"
    cd "$build_dir"
    
    echo "Configuring..."
    PICO_SDK_PATH="$PICO_SDK_PATH" cmake "$SCRIPT_DIR/bridge" \
        -DPICO_BOARD="$board" \
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

build_bridge_fpga() {
    local build_dir="$SCRIPT_DIR/bridge-fpga/build"
    
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Building FPGA Bridge for pico2-ice${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    # Check for FPGA toolchain
    local missing_tools=0
    for tool in yosys nextpnr-ice40 icepack; do
        if ! command -v "$tool" &>/dev/null; then
            echo -e "${RED}Missing required tool: $tool${NC}"
            missing_tools=1
        fi
    done
    if [ "$missing_tools" = "1" ]; then
        echo -e "${YELLOW}Install OSS FPGA toolchain: brew install yosys nextpnr icestorm${NC}"
        return 1
    fi
    
    if [ "$CLEAN" = "1" ]; then
        echo "Cleaning $build_dir..."
        rm -rf "$build_dir"
    fi
    
    mkdir -p "$build_dir"
    cd "$build_dir"
    
    echo "Configuring..."
    PICO_SDK_PATH="$PICO_SDK_PATH" cmake "$SCRIPT_DIR/bridge-fpga" \
        -DPICO_BOARD="pico2_ice" \
        -DCMAKE_BUILD_TYPE=Release
    
    echo "Building (includes Verilog synthesis)..."
    make -j"$NJOBS"
    
    if [ -f "kmbox_fpga_bridge.uf2" ]; then
        local size=$(ls -la kmbox_fpga_bridge.uf2 | awk '{print $5}')
        echo -e "${GREEN}✓ Built: $build_dir/kmbox_fpga_bridge.uf2 ($size bytes)${NC}"
        return 0
    else
        echo -e "${RED}✗ FPGA bridge build failed${NC}"
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
    
    # Fallback: try to copy to mounted drive (check both default and white-labeled names)
    for mount in /Volumes/RPI-RP2 /Volumes/KMBOX-BRDG /media/*/RPI-RP2 /media/*/KMBOX-BRDG /run/media/*/RPI-RP2 /run/media/*/KMBOX-BRDG; do
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
        pico2|metro|metro-broken|xiao|bridge|bridge-metro|bridge-feather|bridge-fpga|both|all|dual-metro|flash-metros|flash-fpga|white-label|white-label-verify)
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

# Default target: build & flash FPGA bridge + metro-broken
if [ -z "$TARGET" ]; then
    TARGET="flash-fpga"
fi

# Execute build
cd "$SCRIPT_DIR"

case $TARGET in
    pico2)
        build_kmbox pico2
        if [ "$FLASH" = "1" ]; then
            wait_for_device "KMBox (RP2350 Pico 2)"
            flash_firmware "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" "KMBox (RP2350 Pico 2)"
        fi
        ;;
    metro)
        build_kmbox metro
        if [ "$FLASH" = "1" ]; then
            wait_for_device "KMBox (Metro RP2350)"
            flash_firmware "$SCRIPT_DIR/build-metro/PIOKMbox.uf2" "KMBox (Metro RP2350)"
        fi
        ;;
    metro-broken)
        build_kmbox metro-broken
        if [ "$FLASH" = "1" ]; then
            wait_for_device "KMBox (Metro RP2350 broken-pins)"
            flash_firmware "$SCRIPT_DIR/build-metro-broken/PIOKMbox.uf2" "KMBox (Metro RP2350 broken-pins)"
        fi
        ;;
    xiao)
        build_kmbox xiao
        if [ "$FLASH" = "1" ]; then
            wait_for_device "KMBox (XIAO RP2350)"
            flash_firmware "$SCRIPT_DIR/build-xiao/PIOKMbox.uf2" "KMBox (XIAO RP2350)"
        fi
        ;;
    bridge)
        build_bridge metro
        if [ "$FLASH" = "1" ]; then
            wait_for_device "UART Bridge (Metro RP2350)"
            flash_firmware "$SCRIPT_DIR/bridge/build-metro/kmbox_bridge.uf2" "UART Bridge (Metro)"
        fi
        ;;
    bridge-metro)
        # Alias for bridge (metro is now the default)
        build_bridge metro
        if [ "$FLASH" = "1" ]; then
            wait_for_device "UART Bridge (Metro RP2350)"
            flash_firmware "$SCRIPT_DIR/bridge/build-metro/kmbox_bridge.uf2" "UART Bridge (Metro)"
        fi
        ;;
    bridge-feather)
        build_bridge feather
        if [ "$FLASH" = "1" ]; then
            wait_for_device "UART Bridge (Feather RP2350)"
            flash_firmware "$SCRIPT_DIR/bridge/build/kmbox_bridge.uf2" "UART Bridge (Feather)"
        fi
        ;;
    bridge-fpga)
        build_bridge_fpga
        if [ "$FLASH" = "1" ]; then
            wait_for_device "FPGA Bridge (pico2-ice)"
            flash_firmware "$SCRIPT_DIR/bridge-fpga/build/kmbox_fpga_bridge.uf2" "FPGA Bridge (pico2-ice)"
        fi
        ;;
    dual-metro)
        # Build both Metro RP2350 firmwares: KMBox + Bridge
        build_kmbox metro
        build_bridge metro
        if [ "$FLASH" = "1" ]; then
            echo ""
            echo -e "${YELLOW}Dual-Metro flash sequence:${NC}"
            echo "  1) KMBox Metro (USB Host device)"
            echo "  2) Bridge Metro (USB CDC + ILI9341 TFT)"
            echo ""
            
            wait_for_device "KMBox (Metro RP2350)"
            flash_firmware "$SCRIPT_DIR/build-metro/PIOKMbox.uf2" "KMBox (Metro RP2350)"
            
            echo ""
            echo -e "${GREEN}KMBox flashed! Now flashing bridge...${NC}"
            echo ""
            
            wait_for_device "Bridge (Metro RP2350)"
            flash_firmware "$SCRIPT_DIR/bridge/build-metro/kmbox_bridge.uf2" "Bridge (Metro RP2350)"
            
            echo ""
            echo -e "${GREEN}========================================${NC}"
            echo -e "${GREEN}Dual-Metro setup flashed successfully!${NC}"
            echo -e "${GREEN}========================================${NC}"
            echo ""
            echo "Wiring (Metro KMBox <-> Metro Bridge):"
            echo "  KMBox TX/GPIO0  ────► Bridge RX/GPIO1"
            echo "  KMBox RX/GPIO1  ◄──── Bridge TX/GPIO0"
            echo "  KMBox GND       ────  Bridge GND"
            echo ""
            echo "Note: Set the RX/TX switches on both Metros"
            echo "      so that TX=GPIO0 and RX=GPIO1."
            echo ""
            echo "Test with: python bridge/bridge_client.py --test"
            exit 0
        fi
        ;;
    flash-fpga)
        # Build + flash FPGA bridge (pico2-ice) + Metro broken KMBox
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  FPGA Bridge + Metro KMBox Build & Flash${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""

        build_bridge_fpga
        build_kmbox metro-broken

        echo ""
        echo -e "${YELLOW}Flash sequence:${NC}"
        echo "  1) FPGA Bridge  (pico2-ice, USB CDC)"
        echo "  2) KMBox Metro  (Metro RP2350, USB Host)"
        echo ""

        wait_for_device "FPGA Bridge (pico2-ice)"
        flash_firmware "$SCRIPT_DIR/bridge-fpga/build/kmbox_fpga_bridge.uf2" "FPGA Bridge (pico2-ice)"

        echo ""
        echo -e "${GREEN}✓ FPGA bridge flashed! Now flash the KMBox Metro...${NC}"
        echo ""

        wait_for_device "KMBox (Metro RP2350 broken-pins)"
        flash_firmware "$SCRIPT_DIR/build-metro-broken/PIOKMbox.uf2" "KMBox (Metro RP2350)"

        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}  FPGA Bridge setup complete!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo "Wiring (pico2-ice PMOD A → Metro KMBox):"
        echo "  SPI:  MOSI(iCE40 pin 4) → D10,  SCK(pin 2) → D11"
        echo "        CS_N(pin 47) → D22,  MISO(pin 45) ← D23"
        echo "  UART: Bridge GPIO 20 → D9,  Bridge GPIO 25 ← D8"
        echo "  GND:  Common ground"
        echo ""
        exit 0
        ;;
    flash-metros)
        # Legacy: Build + flash both Metro RP2350s (UART bridge setup)
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  Dual-Metro Build & Flash (UART bridge)${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""

        build_kmbox metro
        build_bridge metro

        echo ""
        echo -e "${YELLOW}Flash sequence:${NC}"
        echo "  1) KMBox Metro  (USB Host device)"
        echo "  2) Bridge Metro (USB CDC + ILI9341 TFT)"
        echo ""

        wait_for_device "KMBox (Metro RP2350)"
        flash_firmware "$SCRIPT_DIR/build-metro/PIOKMbox.uf2" "KMBox (Metro RP2350)"

        echo ""
        echo -e "${GREEN}✓ KMBox Metro flashed! Now flash the Bridge Metro...${NC}"
        echo ""

        wait_for_device "Bridge (Metro RP2350)"
        flash_firmware "$SCRIPT_DIR/bridge/build-metro/kmbox_bridge.uf2" "Bridge (Metro RP2350)"

        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}  Dual-Metro setup complete!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo "Wiring (Metro KMBox <-> Metro Bridge):"
        echo "  KMBox TX/GPIO0  ────► Bridge RX/GPIO1"
        echo "  KMBox RX/GPIO1  ◄──── Bridge TX/GPIO0"
        echo "  KMBox GND       ────  Bridge GND"
        echo ""
        exit 0
        ;;
    white-label)
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  Bridge RP2350 OTP White-Labelling${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""
        "$SCRIPT_DIR/bridge/white_label.sh"
        exit 0
        ;;
    white-label-verify)
        "$SCRIPT_DIR/bridge/white_label.sh" --verify
        exit 0
        ;;
    both)
        build_kmbox metro
        build_kmbox pico2
        if [ "$FLASH" = "1" ]; then
            echo ""
            echo -e "${YELLOW}Which KMBox device to flash?${NC}"
            echo "1) Metro RP2350"
            echo "2) Pico 2 (RP2350)"
            echo "3) Both (one at a time)"
            echo "4) Skip flashing"
            read -rp "Choice [1-4]: " choice
            case $choice in
                1)
                    wait_for_device "KMBox (Metro RP2350)"
                    flash_firmware "$SCRIPT_DIR/build-metro/PIOKMbox.uf2" "KMBox (Metro RP2350)"
                    ;;
                2)
                    wait_for_device "KMBox (Pico 2 RP2350)"
                    flash_firmware "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" "KMBox (Pico 2 RP2350)"
                    ;;
                3)
                    wait_for_device "KMBox (Metro RP2350)"
                    flash_firmware "$SCRIPT_DIR/build-metro/PIOKMbox.uf2" "KMBox (Metro RP2350)"
                    wait_for_device "KMBox (Pico 2 RP2350)"
                    flash_firmware "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" "KMBox (Pico 2 RP2350)"
                    ;;
            esac
        fi
        ;;
    all)
        # Build KMBox first (Metro RP2350 USB Host)
        build_kmbox metro
        
        # Wait and flash KMBox
        wait_for_device "KMBox (Metro RP2350)"
        flash_firmware "$SCRIPT_DIR/build-metro/PIOKMbox.uf2" "KMBox Firmware"
        
        echo ""
        echo -e "${GREEN}KMBox flashed! Now building bridge...${NC}"
        echo ""
        
        # Build bridge (Metro)
        build_bridge metro
        
        # Wait and flash bridge
        wait_for_device "UART Bridge (Metro RP2350)"
        flash_firmware "$SCRIPT_DIR/bridge/build-metro/kmbox_bridge.uf2" "UART Bridge (Metro)"
        
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}Both devices flashed successfully!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo "Wiring reminder (Metro KMBox <-> Metro Bridge):"
        echo "  KMBox TX/GPIO0  ────► Bridge RX/GPIO1"
        echo "  KMBox RX/GPIO1  ◄──── Bridge TX/GPIO0"
        echo "  KMBox GND       ────  Bridge GND"
        echo ""
        echo "Note: Set the RX/TX switches on both Metros"
        echo "      so that TX=GPIO0 and RX=GPIO1."
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
[ -f "$SCRIPT_DIR/build-pico2/PIOKMbox.uf2" ] && echo "  KMBox (Pico 2):            build-pico2/PIOKMbox.uf2"
[ -f "$SCRIPT_DIR/build-metro/PIOKMbox.uf2" ] && echo "  KMBox (Metro RP2350):      build-metro/PIOKMbox.uf2"
[ -f "$SCRIPT_DIR/build-metro-broken/PIOKMbox.uf2" ] && echo "  KMBox (Metro broken):      build-metro-broken/PIOKMbox.uf2"
[ -f "$SCRIPT_DIR/build-xiao/PIOKMbox.uf2" ] && echo "  KMBox (XIAO RP2350):       build-xiao/PIOKMbox.uf2"
[ -f "$SCRIPT_DIR/bridge/build/kmbox_bridge.uf2" ] && echo "  Bridge (Feather):          bridge/build/kmbox_bridge.uf2"
[ -f "$SCRIPT_DIR/bridge/build-metro/kmbox_bridge.uf2" ] && echo "  Bridge (Metro RP2350):     bridge/build-metro/kmbox_bridge.uf2"
[ -f "$SCRIPT_DIR/bridge-fpga/build/kmbox_fpga_bridge.uf2" ] && echo "  Bridge (FPGA pico2-ice):   bridge-fpga/build/kmbox_fpga_bridge.uf2"