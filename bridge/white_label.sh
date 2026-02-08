#!/bin/bash

# White-label OTP writer for KMBox Bridge (RP2350)
#
# Burns custom USB branding into RP2350 OTP (One-Time-Programmable) memory.
# This replaces the default "Raspberry Pi" / "RP2350" identity that shows
# in BOOTSEL mode with Hurricane KMBox branding.
#
# ⚠️  WARNING: OTP writes are PERMANENT and IRREVERSIBLE!
#     Once written, these values cannot be changed.
#     The device will always identify as "Hurricane KMBox Bridge"
#     in BOOTSEL mode after this operation.
#
# What this changes (BOOTSEL mode only):
#   - USB Manufacturer:  "Raspberry Pi"  → "Hurricane"
#   - USB Product:       "RP2350 Boot"   → "KMBox Bridge"
#   - Volume Label:      "RP2350"        → "KMBOX-BRDG"
#   - UF2 Model:         "Raspberry Pi RP2350" → "Hurricane KMBox Bridge"
#   - UF2 Board ID:      "RP2350"        → "KMBOX-BRIDGE"
#   - SCSI Vendor:       "RPI"           → "HURRCNE"
#   - SCSI Product:      "RP2350"        → "KMBox Bridge"
#   - INDEX.HTM redirect → github.com/ramseymcgrath/RaspberryKMBox
#
# Normal firmware USB descriptors (CDC mode) are NOT affected — those
# are set at compile time via CMakeLists.txt (already "Hurricane").
#
# Usage:
#   ./white_label.sh              # Interactive — prompts before writing
#   ./white_label.sh --force      # Skip confirmation (CI/automation)
#   ./white_label.sh --dry-run    # Show what would be written, don't write
#   ./white_label.sh --verify     # Read back current OTP white-label state

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JSON_FILE="$SCRIPT_DIR/white_label.json"
OTP_START_ROW="0x400"

# Find picotool
PICOTOOL="${PICOTOOL:-$(command -v picotool 2>/dev/null || echo "$HOME/.pico-sdk/picotool/2.1.1/picotool/picotool")}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Burns KMBox Bridge white-label branding into RP2350 OTP memory."
    echo ""
    echo "Options:"
    echo "  --force       Skip confirmation prompt"
    echo "  --dry-run     Show JSON config without writing"
    echo "  --verify      Read current OTP white-label fields"
    echo "  --json FILE   Use custom JSON file (default: white_label.json)"
    echo "  -h, --help    Show this help"
    echo ""
    echo "⚠️  OTP writes are PERMANENT and IRREVERSIBLE!"
}

verify_otp() {
    echo -e "${BLUE}Reading current OTP white-label state...${NC}"
    echo ""
    
    echo -e "${CYAN}USB_WHITE_LABEL_ADDR:${NC}"
    "$PICOTOOL" otp get OTP_DATA_USB_WHITE_LABEL_ADDR 2>&1 || echo "  (not set)"
    
    echo ""
    echo -e "${CYAN}USB_BOOT_FLAGS:${NC}"
    "$PICOTOOL" otp get OTP_DATA_USB_BOOT_FLAGS 2>&1 || echo "  (not set)"
    
    echo ""
    echo -e "${CYAN}OTP rows ${OTP_START_ROW}+:${NC}"
    "$PICOTOOL" otp dump 2>&1 | grep -A 20 "^0x0400:" || echo "  (empty)"
}

# Parse arguments
FORCE=0
DRY_RUN=0
VERIFY=0

for arg in "$@"; do
    case $arg in
        --force)   FORCE=1 ;;
        --dry-run) DRY_RUN=1 ;;
        --verify)  VERIFY=1 ;;
        --json)    shift; JSON_FILE="$1" ;;
        -h|--help) usage; exit 0 ;;
        *)
            echo -e "${RED}Unknown option: $arg${NC}"
            usage
            exit 1
            ;;
    esac
done

# Check picotool
if [ ! -x "$PICOTOOL" ]; then
    echo -e "${RED}Error: picotool not found${NC}"
    echo "Install via: brew install picotool"
    echo "Or set PICOTOOL=/path/to/picotool"
    exit 1
fi

echo -e "${CYAN}Using picotool: $PICOTOOL${NC}"
"$PICOTOOL" version 2>&1 || true
echo ""

# Verify mode
if [ "$VERIFY" = "1" ]; then
    verify_otp
    exit 0
fi

# Check JSON file
if [ ! -f "$JSON_FILE" ]; then
    echo -e "${RED}Error: White-label JSON not found: $JSON_FILE${NC}"
    exit 1
fi

# Show what will be written
echo -e "${BOLD}White-label configuration:${NC}"
echo -e "${CYAN}─────────────────────────────────────────${NC}"
cat "$JSON_FILE"
echo ""
echo -e "${CYAN}─────────────────────────────────────────${NC}"
echo ""
echo -e "OTP start row: ${BOLD}${OTP_START_ROW}${NC}"
echo ""

if [ "$DRY_RUN" = "1" ]; then
    echo -e "${YELLOW}Dry run — nothing will be written.${NC}"
    exit 0
fi

# Confirm
if [ "$FORCE" != "1" ]; then
    echo -e "${RED}${BOLD}⚠️  WARNING: OTP writes are PERMANENT and IRREVERSIBLE!${NC}"
    echo ""
    echo "This will burn the following into the RP2350's OTP memory:"
    echo "  • Custom USB device descriptor (manufacturer, product, serial)"
    echo "  • Custom volume label (\"KMBOX-BRDG\" instead of \"RP2350\")"
    echo "  • Custom UF2 info (model, board ID)"
    echo "  • Custom SCSI inquiry response"
    echo "  • Custom INDEX.HTM redirect URL"
    echo ""
    echo "The device must be in BOOTSEL mode (hold BOOTSEL + reset)."
    echo ""
    echo -ne "${YELLOW}Type 'BURN' to proceed: ${NC}"
    read -r confirm
    if [ "$confirm" != "BURN" ]; then
        echo -e "${RED}Aborted.${NC}"
        exit 1
    fi
fi

echo ""
echo -e "${BLUE}Writing white-label OTP data...${NC}"
echo ""

"$PICOTOOL" otp white-label -s "$OTP_START_ROW" "$JSON_FILE"

echo ""
echo -e "${GREEN}${BOLD}✓ White-label OTP written successfully!${NC}"
echo ""
echo -e "${BLUE}Rebooting device into BOOTSEL to verify...${NC}"
"$PICOTOOL" reboot -u 2>/dev/null || true
sleep 2

echo ""
echo -e "${GREEN}Done! The bridge device will now show as:${NC}"
echo "  USB Manufacturer:  Hurricane"
echo "  USB Product:       KMBox Bridge"
echo "  Volume Label:      KMBOX-BRDG"
echo "  UF2 Model:         Hurricane KMBox Bridge"
echo "  Board ID:          KMBOX-BRIDGE"
echo ""
echo "When flashing firmware, the drive will appear as 'KMBOX-BRDG'"
echo "instead of 'RPI-RP2'."
