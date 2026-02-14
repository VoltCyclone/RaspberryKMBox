/**
 * TFT Configuration for KMBox FPGA Bridge (pico2-ice)
 *
 * ILI9341 2.8" 240x320 TFT on SPI0
 * Pins are on RP2350B high-numbered GPIOs (33-39)
 *
 * Pin mapping:
 *   GPIO 33: Backlight (PWM)
 *   GPIO 34: DC (Data/Command)
 *   GPIO 35: SPI0 MOSI
 *   GPIO 36: SPI0 MISO (reserved for touch reads)
 *   GPIO 37: TFT CS (GPIO, active low)
 *   GPIO 38: SPI0 SCK
 *   GPIO 39: RST (Reset)
 */

#ifndef TFT_CONFIG_H
#define TFT_CONFIG_H

#include "pico/stdlib.h"

// ============================================================================
// Driver Selection: ILI9341
// ============================================================================
#define TFT_DRIVER      TFT_DRIVER_ILI9341

// ============================================================================
// Display Settings
// ============================================================================
#define TFT_SCALE       1       // 1:1 pixel mapping
#define TFT_SWAP_XY     0       // No axis swap
#define TFT_FLIP_X      1       // ILI9341 typically needs X flip
#define TFT_FLIP_Y      0
#define TFT_VSYNC       0       // No vsync
#define TFT_HW_ACCEL    0       // Software rendering

// ============================================================================
// SPI Configuration
// ============================================================================
#define TFT_SPI_DEV     spi0
#define TFT_BAUDRATE    40000000    // 40 MHz (ILI9341 max)

// SPI pins
#define TFT_SCK_PIN     38      // SPI0 SCK
#define TFT_MOSI_PIN    35      // SPI0 TX
#define TFT_MISO_PIN    36      // SPI0 RX (for touch reads)

// ============================================================================
// Control Pins
// ============================================================================
#define TFT_CS_PIN      37      // Chip Select (GPIO, active low)
#define TFT_RS_PIN      34      // Data/Command (DC)
#define TFT_RST_PIN     39      // Reset
#define TFT_BL_PIN      33      // Backlight (PWM)

// ============================================================================
// Timing
// ============================================================================
#define TFT_RST_DELAY   50      // Reset delay (ms)

#endif // TFT_CONFIG_H
