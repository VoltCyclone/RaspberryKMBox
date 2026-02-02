/**
 * TFT Display Wrapper for KMBox Bridge
 * 
 * Adapts the pico-tft library to provide the tft_display.h API expected
 * by the bridge application. Uses ST7735 1.8" display on SPI with DMA.
 */

#include "tft.h"  // pico-tft library
#include "tft_display.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>

// Declare the renamed pico-tft init function
extern void picotft_init(void);

// ============================================================================
// Internal State
// ============================================================================
static bool initialized = false;
static tft_stats_t last_stats = {0};
static uint32_t last_update_time = 0;
#define UPDATE_INTERVAL_MS 100  // 10 FPS

// Color definitions mapped to 8-bit palette indices
// We'll use the default palette and map our colors
#define COLOR_BG        0x00  // Black background
#define COLOR_TEXT      0xFF  // White text
#define COLOR_OK        0x1C  // Green
#define COLOR_WARN      0xE0  // Yellow  
#define COLOR_ERROR     0xE0  // Red
#define COLOR_INACTIVE  0x92  // Gray

// Layout constants for 128x160 display
#define LINE_HEIGHT     16
#define MARGIN_X        2
#define MARGIN_Y        2

// Simple 5x7 font bitmap (we'll use the one from pico-tft's font.c)
extern uint8_t tft_font[256 * 16];

// ============================================================================
// Helper Functions
// ============================================================================

static void draw_status_bar(const tft_stats_t *stats) {
    int y = MARGIN_Y;
    
    // Title
    tft_draw_string(MARGIN_X, y, COLOR_TEXT, "KMBox Bridge");
    y += LINE_HEIGHT;
    
    // Connection status
    char line[32];
    uint8_t conn_color = (stats->kmbox_connected && stats->cdc_connected) ? 
                         COLOR_OK : COLOR_ERROR;
    snprintf(line, sizeof(line), "CDC:%c KM:%c", 
             stats->cdc_connected ? 'Y' : 'N',
             stats->kmbox_connected ? 'Y' : 'N');
    tft_draw_string(MARGIN_X, y, conn_color, line);
    y += LINE_HEIGHT;
    
    // API mode
    const char *mode_str;
    switch(stats->api_mode) {
        case 0: mode_str = "KMBox"; break;
        case 1: mode_str = "Makcu"; break;
        case 2: mode_str = "Ferrum"; break;
        default: mode_str = "???"; break;
    }
    snprintf(line, sizeof(line), "Mode: %s", mode_str);
    tft_draw_string(MARGIN_X, y, COLOR_TEXT, line);
    y += LINE_HEIGHT;
    
    // UART baud rate
    snprintf(line, sizeof(line), "UART: %luK", stats->uart_baud / 1000);
    tft_draw_string(MARGIN_X, y, COLOR_TEXT, line);
    y += LINE_HEIGHT;
    
    // Data rates
    snprintf(line, sizeof(line), "TX: %lu B/s", stats->tx_rate_bps);
    tft_draw_string(MARGIN_X, y, COLOR_TEXT, line);
    y += LINE_HEIGHT;
    
    snprintf(line, sizeof(line), "RX: %lu B/s", stats->rx_rate_bps);
    tft_draw_string(MARGIN_X, y, COLOR_TEXT, line);
    y += LINE_HEIGHT;
    
    // Mouse activity
    snprintf(line, sizeof(line), "Mouse: %lu", stats->mouse_moves);
    tft_draw_string(MARGIN_X, y, COLOR_TEXT, line);
    y += LINE_HEIGHT;
    
    // Device info (if available)
    if (stats->device_vid != 0) {
        snprintf(line, sizeof(line), "Dev: %04X:%04X", 
                 stats->device_vid, stats->device_pid);
        tft_draw_string(MARGIN_X, y, COLOR_TEXT, line);
        y += LINE_HEIGHT;
        
        if (stats->device_product[0]) {
            // Truncate if too long
            char prod[17];
            strncpy(prod, stats->device_product, 16);
            prod[16] = '\0';
            tft_draw_string(MARGIN_X, y, COLOR_TEXT, prod);
            y += LINE_HEIGHT;
        }
    }
    
    // Uptime
    uint32_t uptime_min = stats->uptime_sec / 60;
    uint32_t uptime_sec = stats->uptime_sec % 60;
    snprintf(line, sizeof(line), "Up: %lum%lus", uptime_min, uptime_sec);
    tft_draw_string(MARGIN_X, y, COLOR_TEXT, line);
}

// ============================================================================
// Public API Implementation
// ============================================================================

bool tft_init(void) {
    if (initialized) {
        return true;
    }
    
    printf("[TFT] Initializing pico-tft library...\n");
    
    // Initialize the pico-tft library
    picotft_init();
    
    printf("[TFT] pico-tft init complete, clearing display...\n");
    
    // Clear the display
    tft_fill(COLOR_BG);
    
    printf("[TFT] Display cleared, swapping buffers...\n");
    
    tft_swap_sync();
    
    printf("[TFT] Buffer swap complete, setting up backlight...\n");
    
    // Set backlight to full brightness via PWM
    gpio_set_function(TFT_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), 255);
    pwm_set_enabled(slice, true);
    
    initialized = true;
    printf("[TFT] Initialization complete\n");
    return true;
}

void tft_update(const tft_stats_t *stats) {
    if (!initialized) {
        return;
    }
    
    // Rate limiting
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_update_time < UPDATE_INTERVAL_MS) {
        return;
    }
    last_update_time = now;
    
    // Check if we need to redraw (stats changed)
    if (memcmp(&last_stats, stats, sizeof(tft_stats_t)) == 0) {
        return;
    }
    
    // Clear screen
    tft_fill(COLOR_BG);
    
    // Draw all stats
    draw_status_bar(stats);
    
    // Swap and sync (blocking)
    tft_swap_sync();
    
    // Remember stats
    memcpy(&last_stats, stats, sizeof(tft_stats_t));
}

void tft_refresh_now(const tft_stats_t *stats) {
    if (!initialized) {
        return;
    }
    
    // Clear screen
    tft_fill(COLOR_BG);
    
    // Draw all stats
    draw_status_bar(stats);
    
    // Swap and sync (blocking)
    tft_swap_sync();
    
    // Remember stats
    memcpy(&last_stats, stats, sizeof(tft_stats_t));
    last_update_time = to_ms_since_boot(get_absolute_time());
}

void tft_show_splash(void) {
    if (!initialized) {
        return;
    }
    
    tft_fill(COLOR_BG);
    
    int y = TFT_HEIGHT / 2 - LINE_HEIGHT * 2;
    tft_draw_string_center(TFT_WIDTH / 2, y, COLOR_TEXT, "KMBox Bridge");
    y += LINE_HEIGHT;
    tft_draw_string_center(TFT_WIDTH / 2, y, COLOR_TEXT, "Autopilot");
    y += LINE_HEIGHT * 2;
    tft_draw_string_center(TFT_WIDTH / 2, y, COLOR_OK, "Initializing...");
    
    tft_swap_sync();
    
    printf("[TFT] Splash screen displayed\n");
}

void tft_show_error(const char *msg) {
    if (!initialized) {
        return;
    }
    
    tft_fill(COLOR_BG);
    
    int y = TFT_HEIGHT / 2 - LINE_HEIGHT;
    tft_draw_string_center(TFT_WIDTH / 2, y, COLOR_ERROR, "ERROR");
    y += LINE_HEIGHT;
    tft_draw_string_center(TFT_WIDTH / 2, y, COLOR_TEXT, msg);
    
    tft_swap_sync();
}

void tft_set_backlight(uint8_t brightness) {
    if (!initialized) {
        return;
    }
    
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), brightness);
}

bool tft_is_busy(void) {
    // pico-tft uses blocking sync, so we're never busy
    return false;
}

void tft_task(void) {
    // pico-tft doesn't need background tasks
    // All updates are synchronous
}
