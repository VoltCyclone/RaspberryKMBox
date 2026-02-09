/**
 * TFT Configuration for KMBox Bridge
 * 
 * Board-aware configuration:
 *   - Feather RP2350: ST7735 1.8" 128x160 on SPI0
 *   - Metro RP2350:   ILI9341 2.8" 240x320 Arduino shield on SPI1 (ICSP header)
 * 
 * Pin assignments come from CMake compile definitions (BRIDGE_TFT_*).
 * Fallback defaults match the Feather RP2350 wiring.
 */

#ifndef TFT_CONFIG_H
#define TFT_CONFIG_H

#include "pico/stdlib.h"

// ============================================================================
// Driver Selection (set by CMake: 1=ST7735, 3=ILI9341)
// ============================================================================

#ifdef BRIDGE_TFT_DRIVER
#define TFT_DRIVER      BRIDGE_TFT_DRIVER
#else
#define TFT_DRIVER      TFT_DRIVER_ST7735
#endif

// ============================================================================
// Display Settings
// ============================================================================

#define TFT_SCALE       1       // 1:1 pixel mapping
#define TFT_SWAP_XY     0       // No axis swap

#if defined(BRIDGE_TFT_DRIVER) && BRIDGE_TFT_DRIVER == 3
// ILI9341 Arduino shield is mirrored by default — flip X to correct
#define TFT_FLIP_X      1
#define TFT_FLIP_Y      0
#else
#define TFT_FLIP_X      0
#define TFT_FLIP_Y      0
#endif
#define TFT_VSYNC       0       // No vsync
#define TFT_HW_ACCEL    0       // Software rendering (simpler, reliable)

// ============================================================================
// SPI Configuration (pins from CMake or defaults for Feather)
// ============================================================================

#if defined(BRIDGE_TFT_DRIVER) && BRIDGE_TFT_DRIVER == 3
// Metro RP2350: ILI9341 on SPI1 via Arduino ICSP header
#define TFT_SPI_DEV     spi1
#define TFT_BAUDRATE    40000000                    // 40 MHz (ILI9341 max; SDK rounds to nearest achievable)
#else
// Feather RP2350: ST7735 on SPI0
#define TFT_SPI_DEV     spi0
#define TFT_BAUDRATE    8000000                     // 8 MHz
#endif

#ifdef BRIDGE_TFT_SCK_PIN
#define TFT_SCK_PIN     BRIDGE_TFT_SCK_PIN
#else
#define TFT_SCK_PIN     PICO_DEFAULT_SPI_SCK_PIN    // SPI SCK
#endif

#ifdef BRIDGE_TFT_MOSI_PIN
#define TFT_MOSI_PIN    BRIDGE_TFT_MOSI_PIN
#else
#define TFT_MOSI_PIN    PICO_DEFAULT_SPI_TX_PIN     // SPI TX
#endif

// ============================================================================
// Control Pins (from CMake or defaults for Feather)
// ============================================================================

#ifdef BRIDGE_TFT_CS_PIN
#define TFT_CS_PIN      BRIDGE_TFT_CS_PIN
#else
#define TFT_CS_PIN      9       // Chip Select
#endif

#ifdef BRIDGE_TFT_DC_PIN
#define TFT_RS_PIN      BRIDGE_TFT_DC_PIN
#else
#define TFT_RS_PIN      10      // Data/Command
#endif

#ifdef BRIDGE_TFT_RST_PIN
#define TFT_RST_PIN     BRIDGE_TFT_RST_PIN
#else
#define TFT_RST_PIN     6       // Reset
#endif

#ifdef BRIDGE_TFT_BL_PIN
#define TFT_BL_PIN      BRIDGE_TFT_BL_PIN
#else
#define TFT_BL_PIN      7       // Backlight (PWM)
#endif

// ============================================================================
// Timing
// ============================================================================

#define TFT_RST_DELAY   50      // Reset delay (ms)

#endif // TFT_CONFIG_H