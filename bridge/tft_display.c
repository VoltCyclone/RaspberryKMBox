/**
 * TFT Display Implementation for KMBox Bridge
 * 
 * Uses pico-tft library with ST7735 128x160 display.
 * Compact status display with 10 FPS update rate.
 */

#include "tft_display.h"
#include "tft.h"
#include "tft_config.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>

extern void picotft_init(void);

// ============================================================================
// Layout (128x160 display, 8x16 font)
// ============================================================================

#define UPDATE_INTERVAL_MS  100
#define FONT_W              8
#define FONT_H              16
#define LINE_H              14      // Tight but no overlap
#define MARGIN              2
#define SEP_GAP             2

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

// ============================================================================
// Drawing Helpers
// ============================================================================

static void hline(int y, uint8_t color) {
    if (y >= 0 && y < TFT_HEIGHT) {
        memset(&tft_input[y * TFT_WIDTH + MARGIN], color, TFT_WIDTH - MARGIN * 2);
    }
}

static void box(int x, int y, int w, int h, uint8_t color) {
    for (int row = y; row < y + h && row < TFT_HEIGHT; row++) {
        if (row < 0) continue;
        int x0 = (x < 0) ? 0 : x;
        int x1 = (x + w > TFT_WIDTH) ? TFT_WIDTH : x + w;
        if (x1 > x0) memset(&tft_input[row * TFT_WIDTH + x0], color, x1 - x0);
    }
}

static uint8_t temp_color(float t) {
    if (t > 70.0f) return COL_RED;
    if (t > 55.0f) return COL_YELLOW;
    if (t < 10.0f) return COL_CYAN;
    return COL_WHITE;
}

static uint8_t hmode_color(uint8_t m) {
    switch (m) {
        case 0: return COL_DARK;
        case 1: return COL_YELLOW;
        case 2: return COL_GREEN;
        case 3: return COL_CYAN;
        default: return COL_RED;
    }
}

#define TEMP_VALID(t) ((t) > -50.0f && (t) < 150.0f)

// ============================================================================
// Screen Layout
// ============================================================================

static void draw_stats(const tft_stats_t *stats) {
    char buf[24];
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
    
    if (stats->humanization_valid) {
        snprintf(buf, sizeof(buf), "H%d", stats->humanization_mode);
        tft_draw_string(TFT_WIDTH - MARGIN - 16, y, hmode_color(stats->humanization_mode), buf);
    }
    y += LINE_H;
    
    // === ROW 2: Baud + Uptime ===
    snprintf(buf, sizeof(buf), "%luK", stats->uart_baud / 1000);
    tft_draw_string(MARGIN, y, COL_DARK, buf);
    
    uint32_t mins = stats->uptime_sec / 60;
    uint32_t secs = stats->uptime_sec % 60;
    snprintf(buf, sizeof(buf), "%lum%02lus", mins, secs);
    tft_draw_string(TFT_WIDTH - MARGIN - strlen(buf) * FONT_W, y, COL_DARK, buf);
    y += LINE_H;
    
    // === ROW 3: TX ===
    bool tx_active = (stats->tx_bytes != last_tx_bytes);
    tft_draw_string(MARGIN, y, COL_GRAY, "TX");
    snprintf(buf, sizeof(buf), "%lu", stats->tx_rate_bps);
    tft_draw_string(MARGIN + 24, y, tx_active ? COL_CYAN : COL_WHITE, buf);
    if (tx_active) tft_draw_string(TFT_WIDTH - MARGIN - 8, y, COL_GREEN, "^");
    y += LINE_H;
    
    // === ROW 4: RX ===
    bool rx_active = (stats->rx_bytes != last_rx_bytes);
    tft_draw_string(MARGIN, y, COL_GRAY, "RX");
    snprintf(buf, sizeof(buf), "%lu", stats->rx_rate_bps);
    tft_draw_string(MARGIN + 24, y, rx_active ? COL_CYAN : COL_WHITE, buf);
    if (stats->rx_buffer_level > 0) {
        snprintf(buf, sizeof(buf), "[%lu]", stats->rx_buffer_level);
        tft_draw_string(TFT_WIDTH - MARGIN - strlen(buf) * FONT_W, y, COL_YELLOW, buf);
    } else if (rx_active) {
        tft_draw_string(TFT_WIDTH - MARGIN - 8, y, COL_GREEN, "v");
    }
    y += LINE_H;
    
    // === ROW 5: Mouse ===
    tft_draw_string(MARGIN, y, COL_GRAY, "Mv");
    snprintf(buf, sizeof(buf), "%lu", stats->mouse_moves);
    tft_draw_string(MARGIN + 24, y, COL_WHITE, buf);
    y += LINE_H + SEP_GAP;
    
    // === DEVICE INFO ===
    if (stats->device_vid != 0) {
        hline(y, COL_DARK);
        y += SEP_GAP + 2;
        
        snprintf(buf, sizeof(buf), "%04X:%04X", stats->device_vid, stats->device_pid);
        tft_draw_string(MARGIN, y, COL_CYAN, buf);
        y += LINE_H;
        
        if (stats->device_product[0]) {
            char prod[16];
            strncpy(prod, stats->device_product, 15);
            prod[15] = '\0';
            tft_draw_string(MARGIN, y, COL_GRAY, prod);
            y += LINE_H;
        }
    }
    
    // === TEMPERATURES (single row) ===
    bool has_br = TEMP_VALID(stats->bridge_temperature_c);
    bool has_km = stats->kmbox_connected && TEMP_VALID(stats->kmbox_temperature_c);
    
    if (has_br || has_km) {
        y += SEP_GAP;
        hline(y, COL_DARK);
        y += SEP_GAP + 2;
        
        int x = MARGIN;
        if (has_br) {
            snprintf(buf, sizeof(buf), "BR%.0f", stats->bridge_temperature_c);
            tft_draw_string(x, y, temp_color(stats->bridge_temperature_c), buf);
            x += 40;
        }
        if (has_km) {
            snprintf(buf, sizeof(buf), "KM%.0f", stats->kmbox_temperature_c);
            tft_draw_string(x, y, temp_color(stats->kmbox_temperature_c), buf);
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
    
    tft_fill(COL_BG);
    draw_stats(stats);
    tft_swap_sync();
    
    last_tx_bytes = stats->tx_bytes;
    last_rx_bytes = stats->rx_bytes;
}

void tft_display_refresh(const tft_stats_t *stats) {
    if (!initialized) return;
    
    tft_fill(COL_BG);
    draw_stats(stats);
    tft_swap_sync();
    
    last_update_ms = to_ms_since_boot(get_absolute_time());
    last_tx_bytes = stats->tx_bytes;
    last_rx_bytes = stats->rx_bytes;
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