/**
 * TFT Configuration for pico-tft library
 * 
 * This file configures the pico-tft library for use with the KMBox Bridge
 * using the ST7735 1.8" display on the Adafruit Feather RP2350.
 * 
 * Pin assignments use PICO_DEFAULT_SPI_* definitions where possible.
 */

#ifndef TFT_CONFIG_H
#define TFT_CONFIG_H

#include "pico/stdlib.h"

// ============================================================================
// Driver Selection
// ============================================================================
#define TFT_DRIVER TFT_DRIVER_ST7735  // ST7735 1.8" display

// ============================================================================
// Display Scaling and Orientation
// ============================================================================
#define TFT_SCALE 1        // No scaling (1:1 pixel mapping)
#define TFT_SWAP_XY 0      // No axis swap
#define TFT_FLIP_X 0       // No horizontal flip
#define TFT_FLIP_Y 0       // No vertical flip
#define TFT_VSYNC 0        // Disable vsync

// ============================================================================
// Hardware Acceleration
// ============================================================================
#define TFT_HW_ACCEL 0     // Disable PIO hardware acceleration (simpler, more reliable)
#define TFT_PIO pio0       // PIO instance (not used if HW_ACCEL is 0)

// ============================================================================
// SPI Configuration using Pico SDK defaults
// ============================================================================
#define TFT_SPI_DEV      spi0
#define TFT_BAUDRATE     4000000  // 4 MHz for signal integrity on flying leads

// Pin definitions using PICO_DEFAULT_SPI_* where possible
#define TFT_SCK_PIN      PICO_DEFAULT_SPI_SCK_PIN   // SPI0 SCK
#define TFT_MOSI_PIN     PICO_DEFAULT_SPI_TX_PIN    // SPI0 TX (MOSI)
// No MISO needed for write-only display

// Control pins (directly driven GPIOs)
#define TFT_CS_PIN       9    // Chip Select
#define TFT_RS_PIN       10   // Register Select (Data/Command)
#define TFT_RST_PIN      6    // Reset
#define TFT_BL_PIN       7    // Backlight (PWM capable)

// ============================================================================
// Timing Configuration
// ============================================================================
#define TFT_RST_DELAY    50   // Reset delay in milliseconds

#endif // TFT_CONFIG_H
