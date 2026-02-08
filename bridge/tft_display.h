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
// Display View Modes
// ============================================================================

typedef enum {
    TFT_VIEW_DETAILED,   // Full detailed statistics view
    TFT_VIEW_GAUGES      // Large gauge/visualization view
} tft_view_mode_t;

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
    uint32_t button_presses;
    uint32_t commands_per_sec;
    
    // Device info (from KMBox)
    uint16_t device_vid;
    uint16_t device_pid;
    char device_product[32];
    
    // Uptime (seconds)
    uint32_t uptime_sec;
    
    // System info
    uint32_t cpu_freq_mhz;
    
    // Error counters
    uint32_t uart_errors;
    uint32_t frame_errors;
    
    // Peak rates (for showing max throughput achieved)
    uint32_t tx_peak_bps;
    uint32_t rx_peak_bps;
    
    // Latency stats (microseconds)
    uint32_t latency_min_us;
    uint32_t latency_avg_us;
    uint32_t latency_max_us;
    uint32_t latency_jitter_us;
    uint32_t latency_samples;
    
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

/** Get current view mode. */
tft_view_mode_t tft_display_get_view(void);

/** Set view mode. */
void tft_display_set_view(tft_view_mode_t mode);

/** Toggle between view modes. */
void tft_display_toggle_view(void);

/** Process touch input (call regularly if touch enabled). */
void tft_display_handle_touch(void);

#endif // TFT_DISPLAY_H