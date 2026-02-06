/**
 * TFT Display API for KMBox Bridge
 * 
 * Status display for ST7735 128x160 TFT via pico-tft library.
 */

#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Stats Structure
// ============================================================================

typedef struct {
    // Connection status
    bool cdc_connected;
    bool kmbox_connected;
    
    // API mode (0=KMBox, 1=Makcu, 2=Ferrum)
    uint8_t api_mode;
    
    // Humanization mode (0=Off, 1=Low, 2=Med, 3=High)
    uint8_t humanization_mode;
    bool humanization_valid;
    
    // UART stats
    uint32_t uart_baud;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_rate_bps;
    uint32_t rx_rate_bps;
    uint32_t rx_buffer_level;
    
    // Activity counters
    uint32_t mouse_moves;
    
    // Device info (from KMBox)
    uint16_t device_vid;
    uint16_t device_pid;
    char device_product[32];
    
    // Uptime (seconds)
    uint32_t uptime_sec;
    
    // Temperatures (Celsius, use <-50 or >150 to indicate invalid)
    float bridge_temperature_c;
    float kmbox_temperature_c;
} tft_stats_t;

// ============================================================================
// Public API
// ============================================================================

/** Initialize TFT display. Returns true on success. */
bool tft_display_init(void);

/** Update display (rate-limited to 10 FPS). */
void tft_display_update(const tft_stats_t *stats);

/** Force immediate refresh. */
void tft_display_refresh(const tft_stats_t *stats);

/** Show boot splash screen. */
void tft_display_splash(void);

/** Show error message. */
void tft_display_error(const char *msg);

/** Set backlight brightness (0-255). */
void tft_display_backlight(uint8_t brightness);

#endif // TFT_DISPLAY_H