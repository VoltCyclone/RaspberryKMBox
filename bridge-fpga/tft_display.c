/**
 * TFT Display Implementation for KMBox FPGA Bridge
 *
 * Renders status information to ILI9341 240x320 TFT at 10 FPS.
 * Uses timer ISR for rendering (decoupled from main loop),
 * and SPI DMA for transfers (non-blocking).
 *
 * Layout (240x320, 8x16 font):
 *   [Title bar]           "FPGA Bridge"
 *   [Connection section]  CDC: OK/--  KMBox: OK/--
 *   [Tracking section]    Enabled, frames, delta, blob
 *   [Commands section]    Sent, rate
 *   [System section]      Temp, CPU, Uptime
 *   [Touch hints]         Bottom edge
 */

#include "tft_display.h"
#include "tft.h"
#include "ft6206_touch.h"

#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include <string.h>
#include <stdio.h>

// ============================================================================
// Color palette indices (from tft.c palette)
// ============================================================================
#define COL_BG           0x00   // Black
#define COL_WHITE        0x0F   // Pure white
#define COL_GREEN        0xF4   // Bright green
#define COL_YELLOW       0xF1   // Bright yellow-orange
#define COL_RED          0xFF   // Pure red
#define COL_CYAN         0xF7   // Bright cyan
#define COL_GRAY         0x0A   // Gray (labels)
#define COL_DARK         0x06   // Medium gray
#define COL_DIM_LINE     0x04   // Dim separator

// ============================================================================
// Layout constants (ILI9341 240x320, 8x16 font)
// ============================================================================
#define LINE_H       18     // Line height (16px font + 2px gap)
#define MARGIN       4      // Left/right margin
#define SEC_GAP      4      // Gap between sections
#define LABEL_X      MARGIN
#define VALUE_X      (TFT_WIDTH - MARGIN)
#define TITLE_Y      2

// ============================================================================
// State
// ============================================================================
static tft_stats_t shared_stats;
static tft_stats_t render_stats;
static volatile bool frame_ready = false;
static volatile bool stats_updated = false;
static struct repeating_timer render_timer;
static bool display_initialized = false;

// Touch
volatile bool touch_tracker_toggle_requested = false;
static bool touch_available = false;

// ============================================================================
// Fast integer-to-string (no snprintf overhead)
// ============================================================================
static void u32_to_str(uint32_t val, char *buf) {
    if (val == 0) { buf[0] = '0'; buf[1] = '\0'; return; }
    char tmp[11];
    int i = 0;
    while (val > 0) { tmp[i++] = '0' + (val % 10); val /= 10; }
    for (int j = 0; j < i; j++) buf[j] = tmp[i - 1 - j];
    buf[i] = '\0';
}

static void i16_to_str(int16_t val, char *buf) {
    if (val < 0) { *buf++ = '-'; val = -val; }
    u32_to_str((uint32_t)val, buf);
}

static void temp_to_str(float temp, char *buf) {
    int whole = (int)temp;
    int frac = (int)((temp - whole) * 10);
    if (frac < 0) frac = -frac;
    if (temp < 0 && whole == 0) { *buf++ = '-'; }
    u32_to_str((uint32_t)(whole < 0 ? -whole : whole), buf);
    while (*buf) buf++;
    *buf++ = '.';
    *buf++ = '0' + frac;
    *buf++ = 'C';
    *buf++ = '\0';
}

// ============================================================================
// Drawing helpers
// ============================================================================
static int draw_y;  // Current Y position

static void draw_separator(void) {
    tft_draw_rect(MARGIN, draw_y, TFT_WIDTH - MARGIN - 1, draw_y, COL_DIM_LINE);
    draw_y += SEC_GAP;
}

static void draw_label_value(const char *label, const char *value, uint8_t val_color) {
    tft_draw_string(LABEL_X, draw_y, COL_GRAY, label);
    tft_draw_string_right(VALUE_X, draw_y, val_color, value);
    draw_y += LINE_H;
}

static void draw_section_header(const char *title) {
    draw_separator();
    tft_draw_string(LABEL_X, draw_y, COL_CYAN, title);
    draw_y += LINE_H;
}

// ============================================================================
// Render frame (called from timer ISR context)
// ============================================================================
static void render_frame(void) {
    // Snapshot stats
    render_stats = shared_stats;

    // Clear framebuffer
    tft_fill(COL_BG);

    draw_y = TITLE_Y;
    char buf[32];

    // ---- Title bar ----
    tft_draw_string_center(TFT_WIDTH / 2, draw_y, COL_WHITE, "FPGA Bridge");
    draw_y += LINE_H + 2;

    // ---- Connection section ----
    draw_section_header("CONNECTION");

    draw_label_value("CDC:",
        render_stats.cdc_connected ? "Connected" : "---",
        render_stats.cdc_connected ? COL_GREEN : COL_RED);

    draw_label_value("KMBox:",
        render_stats.kmbox_connected ? "Connected" : "---",
        render_stats.kmbox_connected ? COL_GREEN : COL_RED);

    // ---- Tracking section ----
    draw_section_header("TRACKING");

    draw_label_value("Status:",
        render_stats.tracker_enabled ? "Enabled" : "Disabled",
        render_stats.tracker_enabled ? COL_GREEN : COL_GRAY);

    u32_to_str(render_stats.frames_processed, buf);
    draw_label_value("Frames:", buf, COL_WHITE);

    // Delta X,Y
    char delta_buf[24];
    char *p = delta_buf;
    i16_to_str(render_stats.last_dx, p);
    while (*p) p++;
    *p++ = ','; *p++ = ' ';
    i16_to_str(render_stats.last_dy, p);
    draw_label_value("Delta:", delta_buf, COL_WHITE);

    u32_to_str(render_stats.blob_size, buf);
    draw_label_value("Blob:", buf, render_stats.blob_size > 0 ? COL_GREEN : COL_GRAY);

    // ---- Commands section ----
    draw_section_header("COMMANDS");

    u32_to_str(render_stats.commands_sent, buf);
    draw_label_value("Sent:", buf, COL_WHITE);

    u32_to_str(render_stats.commands_per_sec, buf);
    char rate_buf[16];
    strcpy(rate_buf, buf);
    strcat(rate_buf, "/s");
    draw_label_value("Rate:", rate_buf, COL_WHITE);

    // ---- System section ----
    draw_section_header("SYSTEM");

    temp_to_str(render_stats.temperature_c, buf);
    uint8_t temp_col = COL_GREEN;
    if (render_stats.temperature_c > 70.0f) temp_col = COL_RED;
    else if (render_stats.temperature_c > 55.0f) temp_col = COL_YELLOW;
    draw_label_value("Temp:", buf, temp_col);

    u32_to_str(render_stats.cpu_freq_mhz, buf);
    strcat(buf, " MHz");
    draw_label_value("CPU:", buf, COL_WHITE);

    // Uptime
    uint32_t secs = render_stats.uptime_sec;
    uint32_t mins = secs / 60; secs %= 60;
    uint32_t hrs = mins / 60; mins %= 60;
    char uptime_buf[16];
    p = uptime_buf;
    u32_to_str(hrs, p); while (*p) p++;
    *p++ = ':';
    if (mins < 10) *p++ = '0';
    u32_to_str(mins, p); while (*p) p++;
    *p++ = ':';
    if (secs < 10) *p++ = '0';
    u32_to_str(secs, p);
    draw_label_value("Uptime:", uptime_buf, COL_WHITE);

    // ---- Touch hint ----
    tft_draw_string_right(TFT_WIDTH - 2, TFT_HEIGHT - LINE_H, COL_DIM_LINE, "T");

    frame_ready = true;
}

// Timer callback (100ms = 10 FPS)
static bool render_timer_callback(struct repeating_timer *t) {
    (void)t;
    if (!frame_ready) {
        render_frame();
    }
    return true;
}

// ============================================================================
// Touch handling
// ============================================================================
static void touch_event_callback(uint16_t x, uint16_t y) {
    (void)x;
    // Bottom half: toggle tracker
    if (y > TFT_HEIGHT / 2) {
        touch_tracker_toggle_requested = true;
    }
}

// ============================================================================
// Public API
// ============================================================================

bool tft_display_init(void) {
    // Initialize TFT hardware
    picotft_init();

    // Clear screen
    tft_fill(COL_BG);
    tft_swap_sync();

    // PWM backlight
    gpio_set_function(TFT_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), 255);
    pwm_set_enabled(slice, true);

    // Initialize FT6206 touch on I2C1
    touch_available = ft6206_init(i2c1, 26, 27);
    if (touch_available) {
        ft6206_set_callback(touch_event_callback);
        printf("[TFT] FT6206 touch initialized on I2C1 (SDA=26, SCL=27)\n");
    } else {
        printf("[TFT] FT6206 touch not detected\n");
    }

    // Start 10 FPS render timer
    add_repeating_timer_ms(-100, render_timer_callback, NULL, &render_timer);

    display_initialized = true;
    printf("[TFT] Display initialized (ILI9341 240x320, 10 FPS)\n");
    return true;
}

void tft_display_submit_stats(const tft_stats_t *stats) {
    if (!stats) return;
    memcpy((void *)&shared_stats, stats, sizeof(tft_stats_t));
    stats_updated = true;
}

bool tft_display_flush(void) {
    if (!frame_ready) return false;

    frame_ready = false;
    tft_swap_sync();
    return true;
}

void tft_display_splash(void) {
    tft_fill(COL_BG);

    tft_draw_string_center(TFT_WIDTH / 2, TFT_HEIGHT / 2 - 24, COL_WHITE, "KMBox");
    tft_draw_string_center(TFT_WIDTH / 2, TFT_HEIGHT / 2 - 4, COL_CYAN, "FPGA Bridge");
    tft_draw_string_center(TFT_WIDTH / 2, TFT_HEIGHT / 2 + 20, COL_GRAY, "pico2-ice");

    tft_swap_sync();
    sleep_ms(1000);
}

void tft_display_error(const char *msg) {
    tft_fill(COL_BG);
    tft_draw_string_center(TFT_WIDTH / 2, TFT_HEIGHT / 2 - 8, COL_RED, "ERROR");
    tft_draw_string_center(TFT_WIDTH / 2, TFT_HEIGHT / 2 + 12, COL_WHITE, msg);
    tft_swap_sync();
}

void tft_display_backlight(uint8_t brightness) {
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), brightness);
}

void tft_display_handle_touch(void) {
    if (touch_available) {
        ft6206_poll();
    }
}
