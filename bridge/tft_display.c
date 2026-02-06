/**
 * TFT Display Implementation for KMBox Bridge - OPTIMIZED
 * 
 * Optimizations:
 * - Dirty-region tracking to avoid full redraws
 * - Pre-formatted string buffers to reduce snprintf overhead
 * - Single rate-limit layer (100ms)
 * - Intelligent change detection per display region
 */

#include "tft_display.h"
#include "tft.h"
#include "tft_config.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void picotft_init(void);

// ============================================================================
// Layout — adaptive to display size
//   ST7735:  128x160, 8x16 font, tight layout
//   ILI9341: 240x320, 8x16 font, spacious layout with larger sections
// ============================================================================

#define UPDATE_INTERVAL_MS  100  // 10 FPS - single rate limit
#define FONT_W              8
#define FONT_H              16

#if (TFT_RAW_WIDTH >= 240)
// ILI9341 240x320 — spacious layout
#define LINE_H              18
#define MARGIN              4
#define SEP_GAP             4
#define SECTION_GAP         6
#else
// ST7735 128x160 — tight layout
#define LINE_H              14
#define MARGIN              2
#define SEP_GAP             2
#define SECTION_GAP         2
#endif

// RGB332 palette colors
#define COL_BG              0x00
#define COL_WHITE           0xFF
#define COL_GREEN           0x1C
#define COL_YELLOW          0xFC
#define COL_RED             0xE0
#define COL_CYAN            0x1F
#define COL_GRAY            0x92
#define COL_DARK            0x49

// ============================================================================
// State
// ============================================================================

static bool initialized = false;
static uint32_t last_update_ms = 0;
static uint32_t last_tx_bytes = 0;
static uint32_t last_rx_bytes = 0;

// Pre-allocated format buffers (avoid snprintf in hot path)
static char fmt_baud[8];
static char fmt_uptime[12];
static char fmt_tx_rate[12];
static char fmt_rx_rate[12];
static char fmt_rx_buf[8];
static char fmt_moves[12];
static char fmt_vid_pid[12];
static char fmt_br_temp[8];
static char fmt_km_temp[8];
static char fmt_hmode[4];

// ============================================================================
// Drawing Helpers
// ============================================================================

static inline void hline(int y, uint8_t color) {
    if (y >= 0 && y < TFT_HEIGHT) {
        memset(&tft_input[y * TFT_WIDTH + MARGIN], color, TFT_WIDTH - MARGIN * 2);
    }
}

static inline void clear_line(int y) {
    if (y >= 0 && y < TFT_HEIGHT) {
        memset(&tft_input[y * TFT_WIDTH], COL_BG, TFT_WIDTH);
    }
}

static inline void clear_region(int y, int h) {
    int y0 = (y < 0) ? 0 : y;
    int y1 = (y + h > TFT_HEIGHT) ? TFT_HEIGHT : y + h;
    for (int i = y0; i < y1; i++) {
        memset(&tft_input[i * TFT_WIDTH], COL_BG, TFT_WIDTH);
    }
}

static void box(int x, int y, int w, int h, uint8_t color) {
    int y0 = (y < 0) ? 0 : y;
    int y1 = (y + h > TFT_HEIGHT) ? TFT_HEIGHT : y + h;
    int x0 = (x < 0) ? 0 : x;
    int x1 = (x + w > TFT_WIDTH) ? TFT_WIDTH : x + w;
    int width = x1 - x0;
    if (width <= 0) return;
    
    for (int row = y0; row < y1; row++) {
        memset(&tft_input[row * TFT_WIDTH + x0], color, width);
    }
}

static inline uint8_t temp_color(float t) {
    if (t > 70.0f) return COL_RED;
    if (t > 55.0f) return COL_YELLOW;
    if (t < 10.0f) return COL_CYAN;
    return COL_WHITE;
}

static inline uint8_t hmode_color(uint8_t m) {
    static const uint8_t colors[] = { COL_DARK, COL_YELLOW, COL_GREEN, COL_CYAN, COL_RED };
    return (m < 4) ? colors[m] : COL_RED;
}

#define TEMP_VALID(t) ((t) > -50.0f && (t) < 150.0f)

// ============================================================================
// Screen Layout - Simplified full redraw with pre-formatted strings
// ============================================================================

static void format_stats(const tft_stats_t *stats) {
    // Pre-format all strings once (avoids snprintf during drawing)
    snprintf(fmt_baud, sizeof(fmt_baud), "%luK", stats->uart_baud / 1000);
    
    uint32_t mins = stats->uptime_sec / 60;
    uint32_t secs = stats->uptime_sec % 60;
    snprintf(fmt_uptime, sizeof(fmt_uptime), "%lum%02lus", mins, secs);
    
    snprintf(fmt_tx_rate, sizeof(fmt_tx_rate), "%lu", stats->tx_rate_bps);
    snprintf(fmt_rx_rate, sizeof(fmt_rx_rate), "%lu", stats->rx_rate_bps);
    
    if (stats->rx_buffer_level > 0) {
        snprintf(fmt_rx_buf, sizeof(fmt_rx_buf), "[%lu]", stats->rx_buffer_level);
    } else {
        fmt_rx_buf[0] = '\0';
    }
    
    snprintf(fmt_moves, sizeof(fmt_moves), "%lu", stats->mouse_moves);
    
    if (stats->device_vid != 0) {
        snprintf(fmt_vid_pid, sizeof(fmt_vid_pid), "%04X:%04X", 
                 stats->device_vid, stats->device_pid);
    } else {
        fmt_vid_pid[0] = '\0';
    }
    
    if (TEMP_VALID(stats->bridge_temperature_c)) {
        snprintf(fmt_br_temp, sizeof(fmt_br_temp), "BR%.0f", stats->bridge_temperature_c);
    } else {
        fmt_br_temp[0] = '\0';
    }
    
    if (stats->kmbox_connected && TEMP_VALID(stats->kmbox_temperature_c)) {
        snprintf(fmt_km_temp, sizeof(fmt_km_temp), "KM%.0f", stats->kmbox_temperature_c);
    } else {
        fmt_km_temp[0] = '\0';
    }
    
    if (stats->humanization_valid) {
        snprintf(fmt_hmode, sizeof(fmt_hmode), "H%d", stats->humanization_mode);
    } else {
        fmt_hmode[0] = '\0';
    }
}

static void draw_stats(const tft_stats_t *stats) {
    int y = MARGIN;
    
    // === HEADER ===
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "KMBox Bridge");
    y += LINE_H + SEP_GAP;
    hline(y, COL_DARK);
    y += SEP_GAP + 2;
    
    // === ROW 1: CDC / KM / Humanization ===
    uint8_t cdc_col = stats->cdc_connected ? COL_GREEN : COL_RED;
    uint8_t km_col = stats->kmbox_connected ? COL_GREEN : COL_RED;
    
    tft_draw_string(MARGIN, y, COL_GRAY, "CDC:");
    tft_draw_string(MARGIN + 32, y, cdc_col, stats->cdc_connected ? "Y" : "N");
    tft_draw_string(MARGIN + 48, y, COL_GRAY, "KM:");
    tft_draw_string(MARGIN + 72, y, km_col, stats->kmbox_connected ? "Y" : "N");
    
    if (fmt_hmode[0]) {
        tft_draw_string(TFT_WIDTH - MARGIN - 16, y, hmode_color(stats->humanization_mode), fmt_hmode);
    }
    y += LINE_H;
    
#if (TFT_RAW_WIDTH >= 240)
    // === ROW 1b: API Mode (extra space on ILI9341) ===
    {
        const char *mode_names[] = { "KMBox", "Makcu", "Ferrum" };
        const char *mode_str = (stats->api_mode < 3) ? mode_names[stats->api_mode] : "???";
        tft_draw_string(MARGIN, y, COL_GRAY, "API:");
        tft_draw_string(MARGIN + 40, y, COL_CYAN, mode_str);
    }
    y += LINE_H;
#endif
    
    // === ROW 2: Baud + Uptime ===
    tft_draw_string(MARGIN, y, COL_DARK, fmt_baud);
    int uptime_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_uptime) * FONT_W;
    tft_draw_string(uptime_x, y, COL_DARK, fmt_uptime);
    y += LINE_H;
    
    // === ROW 3: TX ===
    bool tx_active = (stats->tx_bytes != last_tx_bytes);
    tft_draw_string(MARGIN, y, COL_GRAY, "TX");
    tft_draw_string(MARGIN + 24, y, tx_active ? COL_CYAN : COL_WHITE, fmt_tx_rate);
    if (tx_active) tft_draw_string(TFT_WIDTH - MARGIN - 8, y, COL_GREEN, "^");
    y += LINE_H;
    
    // === ROW 4: RX ===
    bool rx_active = (stats->rx_bytes != last_rx_bytes);
    tft_draw_string(MARGIN, y, COL_GRAY, "RX");
    tft_draw_string(MARGIN + 24, y, rx_active ? COL_CYAN : COL_WHITE, fmt_rx_rate);
    if (fmt_rx_buf[0]) {
        int buf_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_rx_buf) * FONT_W;
        tft_draw_string(buf_x, y, COL_YELLOW, fmt_rx_buf);
    } else if (rx_active) {
        tft_draw_string(TFT_WIDTH - MARGIN - 8, y, COL_GREEN, "v");
    }
    y += LINE_H;
    
    // === ROW 5: Mouse ===
    tft_draw_string(MARGIN, y, COL_GRAY, "Mv");
    tft_draw_string(MARGIN + 24, y, COL_WHITE, fmt_moves);
    y += LINE_H + SECTION_GAP;
    
    // === DEVICE INFO ===
    if (fmt_vid_pid[0]) {
        hline(y, COL_DARK);
        y += SEP_GAP + 2;
        
        tft_draw_string(MARGIN, y, COL_CYAN, fmt_vid_pid);
        y += LINE_H;
        
        if (stats->device_product[0]) {
#if (TFT_RAW_WIDTH >= 240)
            // ILI9341: can show full product name (up to 29 chars)
            char prod[30];
            int i;
            for (i = 0; i < 29 && stats->device_product[i]; i++) {
                prod[i] = stats->device_product[i];
            }
#else
            char prod[16];
            int i;
            for (i = 0; i < 15 && stats->device_product[i]; i++) {
                prod[i] = stats->device_product[i];
            }
#endif
            prod[i] = '\0';
            tft_draw_string(MARGIN, y, COL_GRAY, prod);
            y += LINE_H;
        }
    }
    
    // === TEMPERATURES ===
    if (fmt_br_temp[0] || fmt_km_temp[0]) {
        y += SECTION_GAP;
        hline(y, COL_DARK);
        y += SEP_GAP + 2;
        
        int x = MARGIN;
        if (fmt_br_temp[0]) {
            tft_draw_string(x, y, temp_color(stats->bridge_temperature_c), fmt_br_temp);
            x += 40;
        }
        if (fmt_km_temp[0]) {
            tft_draw_string(x, y, temp_color(stats->kmbox_temperature_c), fmt_km_temp);
        }
    }
}

// ============================================================================
// Public API
// ============================================================================

bool tft_display_init(void) {
    if (initialized) return true;
    
    picotft_init();
    tft_fill(COL_BG);
    tft_swap_sync();
    
    // PWM backlight
    gpio_set_function(TFT_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), 255);
    pwm_set_enabled(slice, true);
    
    initialized = true;
    return true;
}

void tft_display_update(const tft_stats_t *stats) {
    if (!initialized) return;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_update_ms < UPDATE_INTERVAL_MS) return;
    last_update_ms = now;
    
    // Simple full-screen redraw
    tft_fill(COL_BG);
    format_stats(stats);
    draw_stats(stats);
    tft_swap_sync();
    
    // Track activity for next frame
    last_tx_bytes = stats->tx_bytes;
    last_rx_bytes = stats->rx_bytes;
}

void tft_display_refresh(const tft_stats_t *stats) {
    if (!initialized) return;
    
    tft_fill(COL_BG);
    format_stats(stats);
    draw_stats(stats);
    tft_swap_sync();
    
    last_tx_bytes = stats->tx_bytes;
    last_rx_bytes = stats->rx_bytes;
    last_update_ms = to_ms_since_boot(get_absolute_time());
}

void tft_display_splash(void) {
    if (!initialized) return;
    
    tft_fill(COL_BG);
    
    int y = TFT_HEIGHT / 2 - LINE_H * 2;
    box(10, y - 4, TFT_WIDTH - 20, LINE_H * 3 + 8, COL_DARK);
    
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "KMBox Bridge");
    y += LINE_H;
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_WHITE, "Autopilot");
    y += LINE_H * 2;
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_GREEN, "Starting...");
    
    tft_swap_sync();
}

void tft_display_error(const char *msg) {
    if (!initialized) return;
    
    tft_fill(COL_BG);
    
    int y = TFT_HEIGHT / 2 - LINE_H;
    box(5, y - 4, TFT_WIDTH - 10, LINE_H * 2 + 8, COL_RED);
    
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_WHITE, "ERROR");
    y += LINE_H;
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_WHITE, msg);
    
    tft_swap_sync();
}

void tft_display_backlight(uint8_t brightness) {
    if (!initialized) return;
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), brightness);
}