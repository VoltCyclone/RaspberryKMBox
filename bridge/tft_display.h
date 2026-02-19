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
    TFT_VIEW_GAUGES,     // Large gauge/visualization view
    TFT_VIEW_MENU        // Humanization settings menu
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
    
    // Extended humanization info (from 0x0C byte 7 flags + 0x0E ext packet)
    uint8_t inject_mode;       // 0=Immediate, 1=Smooth, 2=VelMatch, 3=Micro
    uint8_t max_per_frame;     // Max pixels per HID frame
    uint8_t queue_depth;       // Current smooth injection queue entries
    uint8_t queue_capacity;    // Max queue size (typically 32)
    bool jitter_enabled;       // Whether jitter/tremor is active
    bool velocity_matching;    // Whether velocity matching is on
    uint16_t total_injected;   // Total movements injected (wrapping 16-bit)
    uint16_t queue_overflows;  // Queue overflow count (wrapping 16-bit)
    
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

    // Console mode (Xbox passthrough)
    bool console_mode;
    bool console_auth_complete;
    uint16_t gamepad_buttons;
    int16_t gamepad_sticks[4];     // LX, LY, RX, RY
    uint16_t gamepad_triggers[2];  // Left, Right
} tft_stats_t;

// ============================================================================
// Menu Command Requests (set by TFT menu, consumed by main loop)
// ============================================================================

typedef struct {
    bool set_humanization_mode;      // Request to set humanization mode
    uint8_t humanization_mode_val;   // 0=Off, 1=Micro, 2=Full

    bool set_inject_mode;            // Request to set injection mode
    uint8_t inject_mode_val;         // 0=Immediate, 1=Smooth, 2=VelMatch, 3=Micro

    bool set_max_per_frame;          // Request to set max pixels per frame
    uint8_t max_per_frame_val;       // 1-32

    bool set_velocity_matching;      // Request to toggle velocity matching
    uint8_t velocity_matching_val;   // 0=Off, 1=On
} tft_menu_request_t;

// Shared request structure — menu writes, main loop reads & clears
extern volatile tft_menu_request_t tft_menu_request;

// ============================================================================
// Public API
// ============================================================================

/** Initialize TFT display and start background render timer. Returns true on success. */
bool tft_display_init(void);

/**
 * Submit latest stats for background rendering.
 * Lightweight memcpy — safe to call every main-loop iteration.
 * The background timer (100ms / 10 FPS) will pick these up and
 * render into the framebuffer without blocking the caller.
 */
void tft_display_submit_stats(const tft_stats_t *stats);

/**
 * Flush a rendered frame to the display via SPI DMA.
 * Returns true if a frame was actually sent, false if nothing new.
 * Must be called from the main loop (not ISR) because it waits on DMA.
 */
bool tft_display_flush(void);

/**
 * Convenience: submit + flush in one call (legacy API).
 * For best latency, prefer calling submit_stats() early in the loop
 * and flush() later so other work can overlap with rendering.
 */
void tft_display_update(const tft_stats_t *stats);

/** Force immediate synchronous refresh (bypasses timer). */
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