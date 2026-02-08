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
#include "hardware/spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if (TFT_RAW_WIDTH >= 240)
// Touch screen support for ILI9341
#include "xpt2046_touch.h"
#define TOUCH_ENABLED 1
#else
#define TOUCH_ENABLED 0
#endif

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
#define COL_DARK            0x6D
#define COL_DIM_LINE        0x49

// ============================================================================
// State
// ============================================================================

static bool initialized = false;
static uint32_t last_update_ms = 0;
static uint32_t last_tx_bytes = 0;
static uint32_t last_rx_bytes = 0;
static tft_view_mode_t current_view = TFT_VIEW_DETAILED;

// Touch handling - interrupt driven
static volatile bool touch_toggle_requested = false;

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
static char fmt_cpu[8];
static char fmt_cmd_rate[12];
static char fmt_lat_avg[12];
static char fmt_lat_range[24];
static char fmt_lat_jitter[12];
static char fmt_errors[16];
static char fmt_buttons[12];
static char fmt_tx_peak[12];
static char fmt_rx_peak[12];

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
// Touch Interrupt Callback
// ============================================================================

#if TOUCH_ENABLED
static void touch_event_callback(void) {
    // Called from ISR - just set flag, don't do heavy work
    touch_toggle_requested = true;
}
#endif

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
    
    // CPU frequency
    snprintf(fmt_cpu, sizeof(fmt_cpu), "%luM", stats->cpu_freq_mhz);
    
    // Command rate
    if (stats->commands_per_sec > 0) {
        snprintf(fmt_cmd_rate, sizeof(fmt_cmd_rate), "%lu/s", stats->commands_per_sec);
    } else {
        fmt_cmd_rate[0] = '\0';
    }
    
    // Latency stats
    if (stats->latency_samples > 0) {
        snprintf(fmt_lat_avg, sizeof(fmt_lat_avg), "%lu", stats->latency_avg_us);
        snprintf(fmt_lat_range, sizeof(fmt_lat_range), "%lu-%lu", 
                 stats->latency_min_us, stats->latency_max_us);
        snprintf(fmt_lat_jitter, sizeof(fmt_lat_jitter), "~%lu", stats->latency_jitter_us);
    } else {
        fmt_lat_avg[0] = '\0';
        fmt_lat_range[0] = '\0';
        fmt_lat_jitter[0] = '\0';
    }
    
    // Errors
    if (stats->uart_errors > 0 || stats->frame_errors > 0) {
        snprintf(fmt_errors, sizeof(fmt_errors), "E:%lu/%lu", 
                 stats->uart_errors, stats->frame_errors);
    } else {
        fmt_errors[0] = '\0';
    }
    
    // Button presses
    if (stats->button_presses > 0) {
        snprintf(fmt_buttons, sizeof(fmt_buttons), "Btn:%lu", stats->button_presses);
    } else {
        fmt_buttons[0] = '\0';
    }
    
    // Peak rates
    if (stats->tx_peak_bps > 0) {
        snprintf(fmt_tx_peak, sizeof(fmt_tx_peak), "pk:%lu", stats->tx_peak_bps);
    } else {
        fmt_tx_peak[0] = '\0';
    }
    
    if (stats->rx_peak_bps > 0) {
        snprintf(fmt_rx_peak, sizeof(fmt_rx_peak), "pk:%lu", stats->rx_peak_bps);
    } else {
        fmt_rx_peak[0] = '\0';
    }
}

static void draw_stats(const tft_stats_t *stats) {
    int y = MARGIN;
    
    // === HEADER ===
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "KMBox Bridge");
    y += LINE_H + SEP_GAP;
    hline(y, COL_DIM_LINE);
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
    tft_draw_string(MARGIN, y, COL_GREEN, fmt_baud);
    int uptime_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_uptime) * FONT_W;
    tft_draw_string(uptime_x, y, COL_CYAN, fmt_uptime);
    y += LINE_H;
    
    // === ROW 2b: CPU + Command Rate ===
    tft_draw_string(MARGIN, y, COL_GRAY, "CPU:");
    tft_draw_string(MARGIN + 32, y, COL_CYAN, fmt_cpu);
    if (fmt_cmd_rate[0]) {
        int cmd_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_cmd_rate) * FONT_W;
        tft_draw_string(cmd_x, y, COL_GREEN, fmt_cmd_rate);
    }
    y += LINE_H;
    
    // === ROW 3: TX ===
    bool tx_active = (stats->tx_bytes != last_tx_bytes);
    uint8_t tx_val_col = tx_active ? COL_GREEN : (stats->tx_rate_bps == 0 ? COL_RED : COL_WHITE);
    tft_draw_string(MARGIN, y, COL_GRAY, "TX");
    tft_draw_string(MARGIN + 24, y, tx_val_col, fmt_tx_rate);
    if (fmt_tx_peak[0]) {
        int peak_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_tx_peak) * FONT_W;
        tft_draw_string(peak_x, y, COL_GRAY, fmt_tx_peak);
    } else if (tx_active) {
        tft_draw_string(TFT_WIDTH - MARGIN - 8, y, COL_GREEN, "^");
    }
    y += LINE_H;
    
    // === ROW 4: RX ===
    bool rx_active = (stats->rx_bytes != last_rx_bytes);
    uint8_t rx_val_col = rx_active ? COL_GREEN : (stats->rx_rate_bps == 0 ? COL_RED : COL_WHITE);
    tft_draw_string(MARGIN, y, COL_GRAY, "RX");
    tft_draw_string(MARGIN + 24, y, rx_val_col, fmt_rx_rate);
    if (fmt_rx_buf[0]) {
        int buf_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_rx_buf) * FONT_W;
        tft_draw_string(buf_x, y, COL_YELLOW, fmt_rx_buf);
    } else if (fmt_rx_peak[0]) {
        int peak_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_rx_peak) * FONT_W;
        tft_draw_string(peak_x, y, COL_GRAY, fmt_rx_peak);
    } else if (rx_active) {
        tft_draw_string(TFT_WIDTH - MARGIN - 8, y, COL_GREEN, "v");
    }
    y += LINE_H;
    
    // === ROW 5: Mouse + Buttons ===
    uint8_t mv_col = (stats->mouse_moves > 0) ? COL_WHITE : COL_RED;
    tft_draw_string(MARGIN, y, COL_GRAY, "Mv");
    tft_draw_string(MARGIN + 24, y, mv_col, fmt_moves);
    if (fmt_buttons[0]) {
        int btn_x = TFT_WIDTH - MARGIN - (int)strlen(fmt_buttons) * FONT_W;
        tft_draw_string(btn_x, y, COL_YELLOW, fmt_buttons);
    }
    y += LINE_H;
    
    // === ROW 6: Errors (if any) ===
    if (fmt_errors[0]) {
        tft_draw_string(MARGIN, y, COL_RED, fmt_errors);
        y += LINE_H;
    }
    
    y += SECTION_GAP;
    
    // === LATENCY STATS ===
    if (fmt_lat_avg[0]) {
        hline(y, COL_DIM_LINE);
        y += SEP_GAP + 2;
        
        tft_draw_string(MARGIN, y, COL_GRAY, "Lat:");
        tft_draw_string(MARGIN + 32, y, COL_GREEN, fmt_lat_avg);
        tft_draw_string(MARGIN + 32 + (int)strlen(fmt_lat_avg) * FONT_W, y, COL_GRAY, "us");
        y += LINE_H;
        
#if (TFT_RAW_WIDTH >= 240)
        // ILI9341: Show range and jitter on separate lines
        if (fmt_lat_range[0]) {
            tft_draw_string(MARGIN, y, COL_GRAY, "Rng:");
            tft_draw_string(MARGIN + 32, y, COL_GRAY, fmt_lat_range);
            tft_draw_string(MARGIN + 32 + (int)strlen(fmt_lat_range) * FONT_W, y, COL_GRAY, "us");
            y += LINE_H;
        }
        if (fmt_lat_jitter[0]) {
            tft_draw_string(MARGIN, y, COL_GRAY, "Jtr:");
            tft_draw_string(MARGIN + 32, y, COL_YELLOW, fmt_lat_jitter);
            tft_draw_string(MARGIN + 32 + (int)strlen(fmt_lat_jitter) * FONT_W, y, COL_GRAY, "us");
            y += LINE_H;
        }
#else
        // ST7735: Compact format - just show jitter
        if (fmt_lat_jitter[0]) {
            tft_draw_string(MARGIN, y, COL_GRAY, "Jtr:");
            tft_draw_string(MARGIN + 32, y, COL_YELLOW, fmt_lat_jitter);
            tft_draw_string(MARGIN + 32 + (int)strlen(fmt_lat_jitter) * FONT_W, y, COL_GRAY, "us");
            y += LINE_H;
        }
#endif
        y += SECTION_GAP;
    }
    
    // === DEVICE INFO ===
    if (fmt_vid_pid[0]) {
        hline(y, COL_DIM_LINE);
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
        hline(y, COL_DIM_LINE);
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
// Gauge View - Large Visual Indicators
// ============================================================================

static void draw_circular_gauge(int cx, int cy, int radius, float value, float max_value, 
                                const char* label, uint8_t color) {
    // Draw gauge outline
    for (int r = radius - 2; r <= radius; r++) {
        for (int angle = 0; angle < 360; angle += 2) {
            float rad = angle * 3.14159f / 180.0f;
            int x = cx + (int)(r * cosf(rad));
            int y = cy + (int)(r * sinf(rad));
            if (x >= 0 && x < TFT_WIDTH && y >= 0 && y < TFT_HEIGHT) {
                tft_input[y * TFT_WIDTH + x] = COL_DARK;
            }
        }
    }
    
    // Draw value arc
    float percentage = (value / max_value);
    if (percentage > 1.0f) percentage = 1.0f;
    int end_angle = (int)(percentage * 270.0f) - 135;  // -135 to 135 degrees
    
    for (int angle = -135; angle < end_angle && angle < 135; angle += 2) {
        float rad = angle * 3.14159f / 180.0f;
        for (int r = radius - 8; r <= radius - 3; r++) {
            int x = cx + (int)(r * cosf(rad));
            int y = cy + (int)(r * sinf(rad));
            if (x >= 0 && x < TFT_WIDTH && y >= 0 && y < TFT_HEIGHT) {
                tft_input[y * TFT_WIDTH + x] = color;
            }
        }
    }
    
    // Draw value text in center
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "%.0f", value);
    int text_len = strlen(value_str);
    int text_x = cx - (text_len * FONT_W) / 2;
    tft_draw_string(text_x, cy - FONT_H / 2, color, value_str);
    
    // Draw label below
    int label_len = strlen(label);
    int label_x = cx - (label_len * FONT_W) / 2;
    tft_draw_string(label_x, cy + radius + 4, COL_GRAY, label);
}

static void draw_bar_gauge(int x, int y, int width, int height, float value, float max_value,
                           const char* label, uint8_t color) {
    // Draw border
    box(x, y, width, 2, COL_DARK);
    box(x, y + height - 2, width, 2, COL_DARK);
    box(x, y, 2, height, COL_DARK);
    box(x + width - 2, y, 2, height, COL_DARK);
    
    // Draw filled portion
    float percentage = value / max_value;
    if (percentage > 1.0f) percentage = 1.0f;
    int fill_width = (int)((width - 8) * percentage);
    if (fill_width > 0) {
        box(x + 4, y + 4, fill_width, height - 8, color);
    }
    
    // Draw value text
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "%.0f", value);
    tft_draw_string(x + width / 2 - (strlen(value_str) * FONT_W) / 2, 
                    y + height / 2 - FONT_H / 2, COL_WHITE, value_str);
    
    // Draw label above
    tft_draw_string(x + 4, y - LINE_H, COL_GRAY, label);
}

static void draw_gauge_view(const tft_stats_t *stats) {
    int y = MARGIN;
    
    // === HEADER ===
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "KMBox Gauges");
    y += LINE_H + SEP_GAP;
    hline(y, COL_DARK);
    y += SEP_GAP + 10;
    
#if (TFT_RAW_WIDTH >= 240)
    // Large display: circular gauges in a grid
    int gauge_radius = 50;
    int gauge_spacing_x = TFT_WIDTH / 2;
    int gauge_spacing_y = 120;
    
    // Row 1: Latency and Command Rate
    if (stats->latency_samples > 0) {
        draw_circular_gauge(gauge_spacing_x / 2, y + gauge_radius, gauge_radius,
                           stats->latency_avg_us, 1000.0f, "Lat us",
                           stats->latency_avg_us < 200 ? COL_GREEN : 
                           stats->latency_avg_us < 500 ? COL_YELLOW : COL_RED);
    }
    
    if (stats->commands_per_sec > 0) {
        draw_circular_gauge(gauge_spacing_x + gauge_spacing_x / 2, y + gauge_radius, gauge_radius,
                           stats->commands_per_sec, 1000.0f, "Cmd/s", COL_GREEN);
    }
    
    y += gauge_spacing_y;
    
    // Row 2: TX and RX rates
    float tx_mbps = stats->tx_rate_bps / 1000.0f;
    float rx_mbps = stats->rx_rate_bps / 1000.0f;
    
    draw_circular_gauge(gauge_spacing_x / 2, y + gauge_radius, gauge_radius,
                       tx_mbps, 100.0f, "TX KB/s", COL_CYAN);
    
    draw_circular_gauge(gauge_spacing_x + gauge_spacing_x / 2, y + gauge_radius, gauge_radius,
                       rx_mbps, 100.0f, "RX KB/s", COL_CYAN);
    
    y += gauge_spacing_y + 10;
    
    // Connection status row
    hline(y, COL_DARK);
    y += 4;
    
    uint8_t cdc_col = stats->cdc_connected ? COL_GREEN : COL_RED;
    uint8_t km_col = stats->kmbox_connected ? COL_GREEN : COL_RED;
    
    tft_draw_string(MARGIN, y, COL_GRAY, "CDC:");
    tft_draw_string(MARGIN + 40, y, cdc_col, stats->cdc_connected ? "CONN" : "DISC");
    
    tft_draw_string(TFT_WIDTH / 2, y, COL_GRAY, "KM:");
    tft_draw_string(TFT_WIDTH / 2 + 32, y, km_col, stats->kmbox_connected ? "CONN" : "DISC");
    
    y += LINE_H;
    
    // Temperature
    if (TEMP_VALID(stats->bridge_temperature_c)) {
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.0fC", stats->bridge_temperature_c);
        tft_draw_string(MARGIN, y, temp_color(stats->bridge_temperature_c), temp_str);
    }
    
    // Uptime
    uint32_t mins = stats->uptime_sec / 60;
    uint32_t secs = stats->uptime_sec % 60;
    char uptime_str[16];
    snprintf(uptime_str, sizeof(uptime_str), "%lum%02lus", mins, secs);
    int uptime_x = TFT_WIDTH - MARGIN - (int)strlen(uptime_str) * FONT_W;
    tft_draw_string(uptime_x, y, COL_CYAN, uptime_str);
#else
    // Small display: horizontal bar gauges
    int bar_height = 24;
    int bar_width = TFT_WIDTH - MARGIN * 2;
    
    // Latency bar
    if (stats->latency_samples > 0) {
        draw_bar_gauge(MARGIN, y, bar_width, bar_height, stats->latency_avg_us, 1000.0f,
                      "Latency (us)", stats->latency_avg_us < 200 ? COL_GREEN : COL_YELLOW);
        y += bar_height + LINE_H + 4;
    }
    
    // Command rate bar
    if (stats->commands_per_sec > 0) {
        draw_bar_gauge(MARGIN, y, bar_width, bar_height, stats->commands_per_sec, 500.0f,
                      "Commands/sec", COL_GREEN);
        y += bar_height + LINE_H + 4;
    }
    
    // TX rate bar
    draw_bar_gauge(MARGIN, y, bar_width, bar_height, stats->tx_rate_bps / 1000.0f, 50.0f,
                  "TX (KB/s)", COL_CYAN);
    y += bar_height + LINE_H + 4;
    
    // RX rate bar  
    draw_bar_gauge(MARGIN, y, bar_width, bar_height, stats->rx_rate_bps / 1000.0f, 50.0f,
                  "RX (KB/s)", COL_CYAN);
#endif
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
    
#if TOUCH_ENABLED
    // Initialize touch controller on same SPI as display
    // Touch CS is typically on a different pin (check your hardware)
    // For ILI9341 Arduino shields, touch CS is usually pin 4
    #ifdef BRIDGE_TOUCH_CS_PIN
    int8_t touch_irq = -1;
    #ifdef BRIDGE_TOUCH_IRQ_PIN
    touch_irq = BRIDGE_TOUCH_IRQ_PIN;
    #endif
    
    xpt2046_init(TFT_SPI_DEV, BRIDGE_TOUCH_CS_PIN, touch_irq);
    
    // Register interrupt callback for touch events
    xpt2046_set_callback(touch_event_callback);
    #endif
    
    // Set calibration for ILI9341 (may need adjustment)
    touch_calibration_t cal = {
        .x_min = 300,
        .x_max = 3800,
        .y_min = 300,
        .y_max = 3800,
        .swap_xy = true,
        .invert_x = false,
        .invert_y = true
    };
    xpt2046_set_calibration(&cal);
#endif
    
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
    
    // Draw based on current view mode
    if (current_view == TFT_VIEW_GAUGES) {
        draw_gauge_view(stats);
    } else {
        draw_stats(stats);
    }
    
    tft_swap_sync();
    
    // Track activity for next frame
    last_tx_bytes = stats->tx_bytes;
    last_rx_bytes = stats->rx_bytes;
}

void tft_display_refresh(const tft_stats_t *stats) {
    if (!initialized) return;
    
    tft_fill(COL_BG);
    format_stats(stats);
    
    // Draw based on current view mode
    if (current_view == TFT_VIEW_GAUGES) {
        draw_gauge_view(stats);
    } else {
        draw_stats(stats);
    }
    
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

tft_view_mode_t tft_display_get_view(void) {
    return current_view;
}

void tft_display_set_view(tft_view_mode_t mode) {
    current_view = mode;
}

void tft_display_toggle_view(void) {
    if (current_view == TFT_VIEW_DETAILED) {
        current_view = TFT_VIEW_GAUGES;
    } else {
        current_view = TFT_VIEW_DETAILED;
    }
}

void tft_display_handle_touch(void) {
#if TOUCH_ENABLED
    // Check flag set by interrupt callback
    if (touch_toggle_requested) {
        touch_toggle_requested = false;
        tft_display_toggle_view();
    }
#endif
}