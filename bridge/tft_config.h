/**
 * TFT Configuration for KMBox Bridge
 * 
 * ST7735 1.8" 128x160 display on SPI0
 */

#ifndef TFT_CONFIG_H
#define TFT_CONFIG_H

#include "pico/stdlib.h"

// ============================================================================
// Driver
// ============================================================================

#define TFT_DRIVER      TFT_DRIVER_ST7735

// ============================================================================
// Display Settings
// ============================================================================

#define TFT_SCALE       1       // 1:1 pixel mapping
#define TFT_SWAP_XY     0       // No axis swap
#define TFT_FLIP_X      0       // No horizontal flip
#define TFT_FLIP_Y      0       // No vertical flip
#define TFT_VSYNC       0       // No vsync
#define TFT_HW_ACCEL    0       // Software rendering (simpler, reliable)

// ============================================================================
// SPI Configuration
// ============================================================================

#define TFT_SPI_DEV     spi0
#define TFT_BAUDRATE    8000000                     // 8 MHz (increase if stable)
#define TFT_SCK_PIN     PICO_DEFAULT_SPI_SCK_PIN    // SPI0 SCK
#define TFT_MOSI_PIN    PICO_DEFAULT_SPI_TX_PIN     // SPI0 TX

// ============================================================================
// Control Pins
// ============================================================================

#define TFT_CS_PIN      9       // Chip Select
#define TFT_RS_PIN      10      // Data/Command
#define TFT_RST_PIN     6       // Reset
#define TFT_BL_PIN      7       // Backlight (PWM)

// ============================================================================
// Timing
// ============================================================================

#define TFT_RST_DELAY   50      // Reset delay (ms)

#endif // TFT_CONFIG_H