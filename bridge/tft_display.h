/**
 * TFT Display Driver for KMBox Bridge
 * 
 * Supports 1.8" ST7735-based SPI TFT displays (128x160 or 160x128)
 * Uses hardware SPI with DMA for non-blocking operation.
 * 
 * Display shows:
 * - Connection status (Bridge <-> KMBox)
 * - Mouse statistics (movements, latency)
 * - API mode indicator
 * - Real-time data rates
 */

#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Pin Configuration for Adafruit Feather RP2350
// ============================================================================
// Using SPI1 pins on the RIGHT side of the Feather header
// All pins accessible from main headers (no HSTX connector needed)
//
// Wiring (TFT -> Feather silkscreen label):
//   VCC  -> 3V
//   GND  -> GND  
//   CS   -> D9   (GPIO9)
//   DC   -> D6   (GPIO6)
//   RST  -> D5   (GPIO5) - or tie to 3V for always-on
//   MOSI -> D11  (GPIO11, SPI1 TX)
//   SCK  -> D10  (GPIO10, SPI1 SCK)
//   LED  -> D13  (GPIO7) - or tie to 3V for always-on backlight

#define TFT_SPI_PORT    spi1
#define TFT_PIN_CS      9    // D9
#define TFT_PIN_DC      6    // D6
#define TFT_PIN_RST     5    // D5
#define TFT_PIN_MOSI    11   // D11
#define TFT_PIN_SCK     10   // D10
#define TFT_PIN_BL      7    // D13

// SPI clock speed (40 MHz for fast updates)
#define TFT_SPI_FREQ    40000000

// Display dimensions (ST7735 1.8")
#define TFT_WIDTH       128
#define TFT_HEIGHT      160

// ============================================================================
// Color Definitions (RGB565 format)
// ============================================================================
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_CYAN        0x07FF
#define TFT_MAGENTA     0xF81F
#define TFT_YELLOW      0xFFE0
#define TFT_ORANGE      0xFD20
#define TFT_GRAY        0x8410
#define TFT_DARKGRAY    0x4208
#define TFT_LIGHTGRAY   0xC618

// Status colors
#define TFT_COLOR_OK        TFT_GREEN
#define TFT_COLOR_WARN      TFT_YELLOW
#define TFT_COLOR_ERROR     TFT_RED
#define TFT_COLOR_INACTIVE  TFT_GRAY

// ============================================================================
// Display Statistics Structure
// ============================================================================
typedef struct {
    // Connection status
    bool kmbox_connected;
    bool cdc_connected;
    
    // API mode
    uint8_t api_mode;  // 0=KMBox, 1=Makcu, 2=Ferrum
    
    // Data statistics
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_rate_bps;  // Bytes per second
    uint32_t rx_rate_bps;
    
    // Attached device info (from KMBox)
    uint16_t device_vid;
    uint16_t device_pid;
    char device_manufacturer[24];
    char device_product[24];
    
    // Mouse activity
    uint32_t mouse_moves;
    uint32_t mouse_clicks;
    
    // System info
    uint32_t uptime_sec;
    uint32_t cpu_freq_mhz;
} tft_stats_t;

// ============================================================================
// Public API
// ============================================================================

/**
 * Initialize the TFT display
 * Sets up SPI with DMA for non-blocking operation
 * 
 * @return true if successful
 */
bool tft_init(void);

/**
 * Update display with current statistics (non-blocking)
 * Call this periodically from main loop
 * Internally rate-limits updates to ~10 FPS
 * 
 * @param stats Current statistics to display
 */
void tft_update(const tft_stats_t *stats);

/**
 * Force immediate display refresh (blocking)
 * Use sparingly, mainly for boot/error screens
 * 
 * @param stats Current statistics
 */
void tft_refresh_now(const tft_stats_t *stats);

/**
 * Show boot splash screen
 */
void tft_show_splash(void);

/**
 * Show error message
 * 
 * @param msg Error message (max ~20 chars)
 */
void tft_show_error(const char *msg);

/**
 * Set backlight brightness
 * 
 * @param brightness 0-255 (0=off, 255=full)
 */
void tft_set_backlight(uint8_t brightness);

/**
 * Check if display is busy with DMA transfer
 * 
 * @return true if DMA transfer in progress
 */
bool tft_is_busy(void);

/**
 * Background task for display updates
 * Call frequently from main loop, returns immediately if nothing to do
 */
void tft_task(void);

#endif // TFT_DISPLAY_H
