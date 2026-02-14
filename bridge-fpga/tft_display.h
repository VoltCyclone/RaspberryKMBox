/**
 * TFT Display for KMBox FPGA Bridge
 *
 * Simplified status display for the pico2-ice FPGA bridge.
 * Renders connection status, tracking stats, and system info
 * to an ILI9341 240x320 TFT at 10 FPS via hardware timer.
 */

#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Stats Structure (simplified for FPGA bridge)
// ============================================================================

typedef struct {
    // Connection
    bool cdc_connected;         // USB CDC to PC
    bool kmbox_connected;       // FPGA reports KMBox responding

    // Color tracking
    bool tracker_enabled;
    uint32_t frames_processed;
    int16_t last_dx;            // Last tracking delta X
    int16_t last_dy;            // Last tracking delta Y
    uint16_t blob_size;         // Last detected blob pixel count

    // Commands
    uint32_t commands_sent;
    uint32_t commands_per_sec;

    // System
    uint32_t uptime_sec;
    uint32_t cpu_freq_mhz;
    float temperature_c;

    // KMBox status (from side channel UART)
    uint8_t km_humanization_mode;   // 0=OFF, 1=LOW, 2=MED, 3=HIGH
    uint8_t km_inject_mode;         // inject_mode_t enum value
    float km_temperature_c;         // KMBox board temperature
    uint8_t km_queue_count;         // Injection queue depth
    bool km_mouse_connected;        // Mouse plugged into KMBox USB host
    uint16_t km_vid;                // Connected device VID
    uint16_t km_pid;                // Connected device PID
    char km_manufacturer[65];       // Connected device manufacturer string
    char km_product[65];            // Connected device product string
} tft_stats_t;

// ============================================================================
// Public API
// ============================================================================

/** Initialize TFT display, touch, and start 10 FPS render timer. */
bool tft_display_init(void);

/** Submit latest stats for background rendering (cheap memcpy). */
void tft_display_submit_stats(const tft_stats_t *stats);

/** Flush rendered frame to display via SPI DMA. Returns true if frame sent. */
bool tft_display_flush(void);

/** Show boot splash screen. */
void tft_display_splash(void);

/** Show error message on display. */
void tft_display_error(const char *msg);

/** Set backlight brightness (0-255). */
void tft_display_backlight(uint8_t brightness);

/** Process touch input (call regularly from main loop). */
void tft_display_handle_touch(void);

// Touch event flags (set by handle_touch, consumed by main loop)
extern volatile bool touch_tracker_toggle_requested;

#endif // TFT_DISPLAY_H
