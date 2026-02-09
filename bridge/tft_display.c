/**
 * TFT Display Implementation for KMBox Bridge - OPTIMIZED
 * 
 * Optimizations:
 * - Full-screen redraw per update (no dirty-region tracking)
 * - Pre-formatted string buffers with lean u32/hex formatters (no snprintf)
 * - Precomputed sin/cos LUT for gauge arcs (no runtime trig)
 * - Cached string lengths to avoid redundant strlen calls
 * - Single rate-limit layer (100ms)
 */

#include "tft_display.h"
#include "tft.h"
#include "tft_config.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if (TFT_RAW_WIDTH >= 240) && defined(BRIDGE_TOUCH_SDA_PIN)
// Capacitive touch via FT6206 over I2C (Adafruit ILI9341 shield)
#include "ft6206_touch.h"
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

// Palette indices into tft_palette[256]
// Indices 0x00-0x0F = grayscale ramp (black→white)
// Indices 0xF0-0xFF = saturated colors
#define COL_BG              0x00        // Black
#define COL_WHITE           0x0F        // Pure white  (255,255,255)
#define COL_GREEN           0xF4        // Bright green (32,255,0)
#define COL_YELLOW          0xF1        // Bright yellow-orange (255,186,0)
#define COL_RED             0xFF        // Pure red (255,0,0)
#define COL_CYAN            0xF7        // Bright cyan (0,255,255)
#define COL_GRAY            0x0A        // Medium-bright gray (172,170,172) - labels
#define COL_DARK            0x06        // Medium gray (98,101,98) - secondary text
#define COL_DIM_LINE        0x04        // Dim gray (65,68,65) - separator lines & zone hints

// ============================================================================
// Fast integer-to-string helpers (avoid snprintf overhead)
// ============================================================================

// Write uint32 to buf, return pointer past last char written.
static char *u32_to_str(char *buf, uint32_t v) {
    char tmp[10];
    int i = 0;
    if (v == 0) { *buf++ = '0'; return buf; }
    while (v) { tmp[i++] = '0' + (v % 10); v /= 10; }
    while (i--) *buf++ = tmp[i];
    return buf;
}

// Write uint32 as zero-padded 2-digit decimal.
static char *u32_to_str02(char *buf, uint32_t v) {
    *buf++ = '0' + (v / 10) % 10;
    *buf++ = '0' + v % 10;
    return buf;
}

// Write uint16 as 4-digit uppercase hex.
static char *u16_to_hex4(char *buf, uint16_t v) {
    static const char hex[] = "0123456789ABCDEF";
    *buf++ = hex[(v >> 12) & 0xF];
    *buf++ = hex[(v >> 8)  & 0xF];
    *buf++ = hex[(v >> 4)  & 0xF];
    *buf++ = hex[v & 0xF];
    return buf;
}

// Format into buf, NUL-terminate, return length.
static inline int fmt_len(const char *buf, char *end) {
    *end = '\0';
    return (int)(end - buf);
}

// ============================================================================
// Precomputed sin/cos LUT for gauge arcs (step=2°, 0..358°)
// Stored as Q15 fixed-point: value = (int)(sinf(a) * 32767)
// ============================================================================

#define SINCOS_LUT_SIZE 180  // 360° / 2° step

static const int16_t sin_lut[SINCOS_LUT_SIZE] = {
        0,   1143,   2285,   3425,   4560,   5690,   6812,   7927,
     9032,  10125,  11207,  12274,  13327,  14364,  15383,  16383,
    17363,  18323,  19260,  20173,  21062,  21925,  22762,  23570,
    24350,  25100,  25820,  26509,  27165,  27787,  28377,  28931,
    29450,  29934,  30381,  30791,  31163,  31497,  31793,  32050,
    32269,  32448,  32587,  32687,  32747,  32767,  32747,  32687,
    32587,  32448,  32269,  32050,  31793,  31497,  31163,  30791,
    30381,  29934,  29450,  28931,  28377,  27787,  27165,  26509,
    25820,  25100,  24350,  23570,  22762,  21925,  21062,  20173,
    19260,  18323,  17363,  16383,  15383,  14364,  13327,  12274,
    11207,  10125,   9032,   7927,   6812,   5690,   4560,   3425,
     2285,   1143,      0,  -1143,  -2285,  -3425,  -4560,  -5690,
    -6812,  -7927,  -9032, -10125, -11207, -12274, -13327, -14364,
   -15383, -16383, -17363, -18323, -19260, -20173, -21062, -21925,
   -22762, -23570, -24350, -25100, -25820, -26509, -27165, -27787,
   -28377, -28931, -29450, -29934, -30381, -30791, -31163, -31497,
   -31793, -32050, -32269, -32448, -32587, -32687, -32747, -32767,
   -32747, -32687, -32587, -32448, -32269, -32050, -31793, -31497,
   -31163, -30791, -30381, -29934, -29450, -28931, -28377, -27787,
   -27165, -26509, -25820, -25100, -24350, -23570, -22762, -21925,
   -21062, -20173, -19260, -18323, -17363, -16383, -15383, -14364,
   -13327, -12274, -11207, -10125,  -9032,  -7927,  -6812,  -5690,
    -4560,  -3425,  -2285,  -1143,
};

// cos(a) = sin(a + 90°)
static inline int16_t lut_sin(int angle_idx) { return sin_lut[angle_idx % SINCOS_LUT_SIZE]; }
static inline int16_t lut_cos(int angle_idx) { return sin_lut[(angle_idx + 45) % SINCOS_LUT_SIZE]; } // +45 entries = +90°

// ============================================================================
// State — timer-driven background rendering
//
// Architecture:
//   Main loop calls tft_display_submit_stats() to copy stats into shared buf.
//   A repeating hardware timer fires every 100ms, picks up the latest stats,
//   formats strings, and draws into the tft_input framebuffer.
//   Main loop calls tft_display_flush() which does tft_swap_sync() (SPI DMA)
//   only when a new frame has been rendered.
// ============================================================================

static bool initialized = false;
static uint32_t last_tx_bytes = 0;
static uint32_t last_rx_bytes = 0;
static tft_view_mode_t current_view = TFT_VIEW_DETAILED;

// Touch handling - zone-based tap detection
// Screen is divided into 3 horizontal zones:
//   Top    (y 0..106):   Cycle display view
//   Middle (y 107..213): Cycle API protocol
//   Bottom (y 214..319): Cycle humanization mode
static volatile bool touch_view_requested = false;
static volatile bool touch_api_requested = false;
static volatile bool touch_human_requested = false;

// Touch zone boundaries (240x320 display split into thirds)
#define TOUCH_ZONE_TOP_END     106
#define TOUCH_ZONE_MID_END     213

// Double-buffered stats: main loop writes, timer ISR reads
static tft_stats_t shared_stats;           // Latest stats from main loop
static volatile bool stats_pending = false; // Main loop set, timer clears

// Frame ready flag: timer ISR sets after drawing, main loop clears after DMA
static volatile bool frame_ready = false;

// Repeating timer handle
static repeating_timer_t tft_render_timer;

// Pre-allocated format buffers with cached lengths
typedef struct { char str[24]; int len; } fmtbuf_t;

static fmtbuf_t fmt_baud;
static fmtbuf_t fmt_uptime;
static fmtbuf_t fmt_tx_rate;
static fmtbuf_t fmt_rx_rate;
static fmtbuf_t fmt_rx_buf;
static fmtbuf_t fmt_moves;
static fmtbuf_t fmt_vid_pid;
static fmtbuf_t fmt_br_temp;
static fmtbuf_t fmt_km_temp;
static fmtbuf_t fmt_hmode;
static fmtbuf_t fmt_hdetail;
static fmtbuf_t fmt_queuebar;
static fmtbuf_t fmt_injcount;
static fmtbuf_t fmt_cpu;
static fmtbuf_t fmt_cmd_rate;
static fmtbuf_t fmt_lat_avg;
static fmtbuf_t fmt_lat_range;
static fmtbuf_t fmt_lat_jitter;
static fmtbuf_t fmt_errors;
static fmtbuf_t fmt_buttons;
static fmtbuf_t fmt_tx_peak;
static fmtbuf_t fmt_rx_peak;

// Helper to clear a fmtbuf
static inline void fmt_clear(fmtbuf_t *f) { f->str[0] = '\0'; f->len = 0; }

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
    if (y1 > y0) {
        memset(&tft_input[y0 * TFT_WIDTH], COL_BG, (y1 - y0) * TFT_WIDTH);
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
static void touch_event_callback(uint16_t x, uint16_t y) {
    // Route tap to the correct zone flag based on Y coordinate
    // The display is 240x320, divided into 3 horizontal bands
    if (y <= TOUCH_ZONE_TOP_END) {
        touch_view_requested = true;       // Top: cycle view
    } else if (y <= TOUCH_ZONE_MID_END) {
        touch_api_requested = true;        // Middle: cycle API protocol
    } else {
        touch_human_requested = true;      // Bottom: cycle humanization
    }
}
#endif

// ============================================================================
// Screen Layout - Simplified full redraw with pre-formatted strings
// ============================================================================

static void format_stats(const tft_stats_t *stats) {
    char *p;

    // Baud rate: "115K" or "921K"
    p = u32_to_str(fmt_baud.str, stats->uart_baud / 1000);
    *p++ = 'K';
    fmt_baud.len = fmt_len(fmt_baud.str, p);

    // Uptime: "12m05s"
    p = u32_to_str(fmt_uptime.str, stats->uptime_sec / 60);
    *p++ = 'm';
    p = u32_to_str02(p, stats->uptime_sec % 60);
    *p++ = 's';
    fmt_uptime.len = fmt_len(fmt_uptime.str, p);

    // TX/RX byte rates
    p = u32_to_str(fmt_tx_rate.str, stats->tx_rate_bps);
    *p++ = ' '; *p++ = 'B'; *p++ = '/'; *p++ = 's';
    fmt_tx_rate.len = fmt_len(fmt_tx_rate.str, p);

    p = u32_to_str(fmt_rx_rate.str, stats->rx_rate_bps);
    *p++ = ' '; *p++ = 'B'; *p++ = '/'; *p++ = 's';
    fmt_rx_rate.len = fmt_len(fmt_rx_rate.str, p);

    // RX buffer level
    if (stats->rx_buffer_level > 0) {
        p = fmt_rx_buf.str;
        *p++ = 'b'; *p++ = 'u'; *p++ = 'f'; *p++ = ' ';
        p = u32_to_str(p, stats->rx_buffer_level);
        fmt_rx_buf.len = fmt_len(fmt_rx_buf.str, p);
    } else { fmt_clear(&fmt_rx_buf); }

    // Mouse moves
    p = u32_to_str(fmt_moves.str, stats->mouse_moves);
    fmt_moves.len = fmt_len(fmt_moves.str, p);

    // VID:PID
    if (stats->device_vid != 0) {
        p = u16_to_hex4(fmt_vid_pid.str, stats->device_vid);
        *p++ = ':';
        p = u16_to_hex4(p, stats->device_pid);
        fmt_vid_pid.len = fmt_len(fmt_vid_pid.str, p);
    } else { fmt_clear(&fmt_vid_pid); }

    // Bridge temp: "Bridge 42C"
    if (TEMP_VALID(stats->bridge_temperature_c)) {
        p = fmt_br_temp.str;
        p = u32_to_str(p, (uint32_t)(stats->bridge_temperature_c + 0.5f));
        *p++ = 'C';
        fmt_br_temp.len = fmt_len(fmt_br_temp.str, p);
    } else { fmt_clear(&fmt_br_temp); }

    // KMBox temp: "KMBox 38C"
    if (stats->kmbox_connected && TEMP_VALID(stats->kmbox_temperature_c)) {
        p = fmt_km_temp.str;
        p = u32_to_str(p, (uint32_t)(stats->kmbox_temperature_c + 0.5f));
        *p++ = 'C';
        fmt_km_temp.len = fmt_len(fmt_km_temp.str, p);
    } else { fmt_clear(&fmt_km_temp); }

    // Humanization mode: "Off", "Low", "Med", "High"
    if (stats->humanization_valid) {
        static const char *hmode_labels[] = { "Off", "Low", "Med", "High" };
        const char *label = (stats->humanization_mode < 4) ? hmode_labels[stats->humanization_mode] : "?";
        p = fmt_hmode.str;
        const char *lp = label;
        while (*lp) *p++ = *lp++;
        fmt_hmode.len = fmt_len(fmt_hmode.str, p);
    } else { fmt_clear(&fmt_hmode); }

    // Humanization detail flags
    if (stats->humanization_valid) {
        p = fmt_hdetail.str;
        if (stats->jitter_enabled && stats->velocity_matching) {
            const char *s = "Jit+Vel";
            while (*s) *p++ = *s++;
        } else if (stats->jitter_enabled) {
            const char *s = "Jitter";
            while (*s) *p++ = *s++;
        } else if (stats->velocity_matching) {
            const char *s = "VelMatch";
            while (*s) *p++ = *s++;
        } else {
            const char *s = "None";
            while (*s) *p++ = *s++;
        }
        fmt_hdetail.len = fmt_len(fmt_hdetail.str, p);
    } else { fmt_clear(&fmt_hdetail); }

    // Queue depth: "3/32"
    if (stats->humanization_valid) {
        p = fmt_queuebar.str;
        p = u32_to_str(p, stats->queue_depth);
        *p++ = '/';
        p = u32_to_str(p, stats->queue_capacity);
        fmt_queuebar.len = fmt_len(fmt_queuebar.str, p);
    } else { fmt_clear(&fmt_queuebar); }

    // Injection count
    if (stats->humanization_valid && stats->total_injected > 0) {
        p = u32_to_str(fmt_injcount.str, stats->total_injected);
        fmt_injcount.len = fmt_len(fmt_injcount.str, p);
    } else { fmt_clear(&fmt_injcount); }

    // CPU frequency: "150 MHz"
    p = u32_to_str(fmt_cpu.str, stats->cpu_freq_mhz);
    *p++ = ' '; *p++ = 'M'; *p++ = 'H'; *p++ = 'z';
    fmt_cpu.len = fmt_len(fmt_cpu.str, p);

    // Command rate: "123 cmd/s"
    if (stats->commands_per_sec > 0) {
        p = u32_to_str(fmt_cmd_rate.str, stats->commands_per_sec);
        *p++ = ' '; *p++ = 'c'; *p++ = 'm'; *p++ = 'd'; *p++ = '/'; *p++ = 's';
        fmt_cmd_rate.len = fmt_len(fmt_cmd_rate.str, p);
    } else { fmt_clear(&fmt_cmd_rate); }

    // Latency
    if (stats->latency_samples > 0) {
        p = u32_to_str(fmt_lat_avg.str, stats->latency_avg_us);
        *p++ = ' '; *p++ = 'u'; *p++ = 's';
        fmt_lat_avg.len = fmt_len(fmt_lat_avg.str, p);

        p = u32_to_str(fmt_lat_range.str, stats->latency_min_us);
        *p++ = '-';
        p = u32_to_str(p, stats->latency_max_us);
        *p++ = ' '; *p++ = 'u'; *p++ = 's';
        fmt_lat_range.len = fmt_len(fmt_lat_range.str, p);

        p = fmt_lat_jitter.str; *p++ = '+'; *p++ = '/';
        *p++ = '-';
        p = u32_to_str(p, stats->latency_jitter_us);
        *p++ = ' '; *p++ = 'u'; *p++ = 's';
        fmt_lat_jitter.len = fmt_len(fmt_lat_jitter.str, p);
    } else {
        fmt_clear(&fmt_lat_avg);
        fmt_clear(&fmt_lat_range);
        fmt_clear(&fmt_lat_jitter);
    }

    // Errors
    if (stats->uart_errors > 0 || stats->frame_errors > 0) {
        p = fmt_errors.str;
        p = u32_to_str(p, stats->uart_errors);
        *p++ = ' '; *p++ = 'U'; *p++ = 'A'; *p++ = 'R'; *p++ = 'T';
        *p++ = ' ';
        p = u32_to_str(p, stats->frame_errors);
        *p++ = ' '; *p++ = 'F'; *p++ = 'r'; *p++ = 'm';
        fmt_errors.len = fmt_len(fmt_errors.str, p);
    } else { fmt_clear(&fmt_errors); }

    // Button presses
    if (stats->button_presses > 0) {
        p = u32_to_str(fmt_buttons.str, stats->button_presses);
        fmt_buttons.len = fmt_len(fmt_buttons.str, p);
    } else { fmt_clear(&fmt_buttons); }

    // Peak rates
    if (stats->tx_peak_bps > 0) {
        p = fmt_tx_peak.str;
        *p++ = 'p'; *p++ = 'k'; *p++ = ' ';
        p = u32_to_str(p, stats->tx_peak_bps);
        fmt_tx_peak.len = fmt_len(fmt_tx_peak.str, p);
    } else { fmt_clear(&fmt_tx_peak); }

    if (stats->rx_peak_bps > 0) {
        p = fmt_rx_peak.str;
        *p++ = 'p'; *p++ = 'k'; *p++ = ' ';
        p = u32_to_str(p, stats->rx_peak_bps);
        fmt_rx_peak.len = fmt_len(fmt_rx_peak.str, p);
    } else { fmt_clear(&fmt_rx_peak); }
}

// Helper to draw a section header label
static int draw_section_header(int y, const char *title) {
    hline(y, COL_DIM_LINE);
    y += SEP_GAP + 1;
    tft_draw_string(MARGIN, y, COL_CYAN, title);
    y += LINE_H;
    return y;
}

// Helper to draw a label: value pair, right-aligned value
static void draw_label_value(int y, const char *label, const char *value, uint8_t val_color) {
    tft_draw_string(MARGIN, y, COL_GRAY, label);
    int lbl_len = 0;
    const char *lp = label;
    while (*lp++) lbl_len++;
    int val_x = MARGIN + (lbl_len + 1) * FONT_W;
    tft_draw_string(val_x, y, val_color, value);
}

// Helper to draw label on left, value right-aligned
static void draw_row_lr(int y, const char *label, uint8_t lbl_col,
                        const char *value, int val_len, uint8_t val_col) {
    tft_draw_string(MARGIN, y, lbl_col, label);
    int rx = TFT_WIDTH - MARGIN - val_len * FONT_W;
    tft_draw_string(rx, y, val_col, value);
}

static void draw_stats(const tft_stats_t *stats) {
    int y = MARGIN;
    
    // === HEADER ===
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "KMBox Bridge");
    y += LINE_H + SEP_GAP;
    hline(y, COL_DIM_LINE);
    y += SEP_GAP + 2;
    
    // === CONNECTION STATUS ===
    {
        uint8_t cdc_col = stats->cdc_connected ? COL_GREEN : COL_RED;
        uint8_t km_col  = stats->kmbox_connected ? COL_GREEN : COL_RED;
        
        tft_draw_string(MARGIN, y, COL_GRAY, "Host");
        tft_draw_string(MARGIN + 5 * FONT_W, y, cdc_col,
                        stats->cdc_connected ? "OK" : "--");

#if (TFT_RAW_WIDTH >= 240)
        tft_draw_string(MARGIN + 10 * FONT_W, y, COL_GRAY, "KMBox");
        tft_draw_string(MARGIN + 16 * FONT_W, y, km_col,
                        stats->kmbox_connected ? "OK" : "--");
#else
        tft_draw_string(MARGIN + 8 * FONT_W, y, COL_GRAY, "KM");
        tft_draw_string(MARGIN + 11 * FONT_W, y, km_col,
                        stats->kmbox_connected ? "OK" : "--");
#endif
        y += LINE_H;
    }
    
    // === API + Uptime row ===
    {
        const char *mode_names[] = { "KMBox", "Makcu", "Ferrum" };
        const char *mode_str = (stats->api_mode < 3) ? mode_names[stats->api_mode] : "???";
        tft_draw_string(MARGIN, y, COL_GRAY, "API");
        tft_draw_string(MARGIN + 4 * FONT_W, y, COL_WHITE, mode_str);
        // Uptime right-aligned
        int ux = TFT_WIDTH - MARGIN - fmt_uptime.len * FONT_W;
        tft_draw_string(ux, y, COL_CYAN, fmt_uptime.str);
        y += LINE_H;
    }
    
    // === HUMANIZATION SECTION ===
    if (fmt_hmode.len) {
        y += SECTION_GAP;
        y = draw_section_header(y, "Humanize");
        
        // Mode + detail flags
        tft_draw_string(MARGIN, y, COL_GRAY, "Mode");
        tft_draw_string(MARGIN + 5 * FONT_W, y,
                        hmode_color(stats->humanization_mode), fmt_hmode.str);
        if (fmt_hdetail.len) {
            int dx = TFT_WIDTH - MARGIN - fmt_hdetail.len * FONT_W;
            tft_draw_string(dx, y, COL_GRAY, fmt_hdetail.str);
        }
        y += LINE_H;
        
        // Queue bar + injection count
        if (fmt_queuebar.len) {
            tft_draw_string(MARGIN, y, COL_GRAY, "Queue");
            uint8_t q_col = COL_GREEN;
            if (stats->queue_depth > stats->queue_capacity * 3 / 4) q_col = COL_RED;
            else if (stats->queue_depth > stats->queue_capacity / 2) q_col = COL_YELLOW;
            tft_draw_string(MARGIN + 6 * FONT_W, y, q_col, fmt_queuebar.str);
            
            if (fmt_injcount.len) {
                int ix = TFT_WIDTH - MARGIN - fmt_injcount.len * FONT_W;
                tft_draw_string(ix, y, COL_GRAY, fmt_injcount.str);
            }
            y += LINE_H;
        }
        
        // Queue overflows (if any)
        if (stats->queue_overflows > 0) {
            char ovf_buf[16]; char *op = ovf_buf;
            const char *s = "Overflow ";
            while (*s) *op++ = *s++;
            op = u32_to_str(op, stats->queue_overflows);
            *op = '\0';
            tft_draw_string(MARGIN, y, COL_RED, ovf_buf);
            y += LINE_H;
        }
    }
    
    // === DATA SECTION ===
    y += SECTION_GAP;
    y = draw_section_header(y, "Data");
    
    // TX row
    {
        bool tx_active = (stats->tx_bytes != last_tx_bytes);
        uint8_t tx_col = tx_active ? COL_GREEN : (stats->tx_rate_bps == 0 ? COL_RED : COL_WHITE);
        tft_draw_string(MARGIN, y, COL_GRAY, "TX");
        tft_draw_string(MARGIN + 3 * FONT_W, y, tx_col, fmt_tx_rate.str);
        if (fmt_tx_peak.len) {
            int px = TFT_WIDTH - MARGIN - fmt_tx_peak.len * FONT_W;
            tft_draw_string(px, y, COL_DARK, fmt_tx_peak.str);
        }
        y += LINE_H;
    }
    
    // RX row
    {
        bool rx_active = (stats->rx_bytes != last_rx_bytes);
        uint8_t rx_col = rx_active ? COL_GREEN : (stats->rx_rate_bps == 0 ? COL_RED : COL_WHITE);
        tft_draw_string(MARGIN, y, COL_GRAY, "RX");
        tft_draw_string(MARGIN + 3 * FONT_W, y, rx_col, fmt_rx_rate.str);
        if (fmt_rx_buf.len) {
            int bx = TFT_WIDTH - MARGIN - fmt_rx_buf.len * FONT_W;
            tft_draw_string(bx, y, COL_YELLOW, fmt_rx_buf.str);
        } else if (fmt_rx_peak.len) {
            int px = TFT_WIDTH - MARGIN - fmt_rx_peak.len * FONT_W;
            tft_draw_string(px, y, COL_DARK, fmt_rx_peak.str);
        }
        y += LINE_H;
    }
    
    // UART baud + command rate
    {
        tft_draw_string(MARGIN, y, COL_GRAY, "Baud");
        tft_draw_string(MARGIN + 5 * FONT_W, y, COL_GREEN, fmt_baud.str);
        if (fmt_cmd_rate.len) {
            int cx = TFT_WIDTH - MARGIN - fmt_cmd_rate.len * FONT_W;
            tft_draw_string(cx, y, COL_GREEN, fmt_cmd_rate.str);
        }
        y += LINE_H;
    }
    
    // Mouse moves + button presses
    {
        uint8_t mv_col = (stats->mouse_moves > 0) ? COL_WHITE : COL_DARK;
        tft_draw_string(MARGIN, y, COL_GRAY, "Mouse");
        tft_draw_string(MARGIN + 6 * FONT_W, y, mv_col, fmt_moves.str);
        if (fmt_buttons.len) {
            tft_draw_string(TFT_WIDTH - MARGIN - (fmt_buttons.len + 5) * FONT_W, y, COL_GRAY, "Btns");
            int bx = TFT_WIDTH - MARGIN - fmt_buttons.len * FONT_W;
            tft_draw_string(bx, y, COL_YELLOW, fmt_buttons.str);
        }
        y += LINE_H;
    }
    
    // Errors (only when present)
    if (fmt_errors.len) {
        tft_draw_string(MARGIN, y, COL_RED, fmt_errors.str);
        y += LINE_H;
    }
    
    // === LATENCY SECTION ===
    if (fmt_lat_avg.len) {
        y += SECTION_GAP;
        y = draw_section_header(y, "Latency");
        
        draw_row_lr(y, "Avg", COL_GRAY, fmt_lat_avg.str, fmt_lat_avg.len, COL_GREEN);
        y += LINE_H;
        
#if (TFT_RAW_WIDTH >= 240)
        if (fmt_lat_range.len) {
            draw_row_lr(y, "Range", COL_GRAY, fmt_lat_range.str, fmt_lat_range.len, COL_GRAY);
            y += LINE_H;
        }
#endif
        if (fmt_lat_jitter.len) {
            draw_row_lr(y, "Jitter", COL_GRAY, fmt_lat_jitter.str, fmt_lat_jitter.len, COL_YELLOW);
            y += LINE_H;
        }
    }
    
    // === SYSTEM SECTION ===
    {
        y += SECTION_GAP;
        y = draw_section_header(y, "System");
        
        // CPU + Device ID
        tft_draw_string(MARGIN, y, COL_GRAY, "CPU");
        tft_draw_string(MARGIN + 4 * FONT_W, y, COL_WHITE, fmt_cpu.str);
        if (fmt_vid_pid.len) {
            int vx = TFT_WIDTH - MARGIN - fmt_vid_pid.len * FONT_W;
            tft_draw_string(vx, y, COL_CYAN, fmt_vid_pid.str);
        }
        y += LINE_H;
        
        // Product name (if available)
        if (stats->device_product[0]) {
#if (TFT_RAW_WIDTH >= 240)
            char prod[30];
            int i;
            for (i = 0; i < 29 && stats->device_product[i]; i++)
                prod[i] = stats->device_product[i];
#else
            char prod[16];
            int i;
            for (i = 0; i < 15 && stats->device_product[i]; i++)
                prod[i] = stats->device_product[i];
#endif
            prod[i] = '\0';
            tft_draw_string(MARGIN, y, COL_DARK, prod);
            y += LINE_H;
        }
        
        // Temperatures
        if (fmt_br_temp.len || fmt_km_temp.len) {
            tft_draw_string(MARGIN, y, COL_GRAY, "Temp");
            int tx = MARGIN + 5 * FONT_W;
            if (fmt_br_temp.len) {
                tft_draw_string(tx, y, temp_color(stats->bridge_temperature_c), fmt_br_temp.str);
                tx += (fmt_br_temp.len + 1) * FONT_W;
            }
            if (fmt_km_temp.len) {
                tft_draw_string(tx, y, temp_color(stats->kmbox_temperature_c), fmt_km_temp.str);
            }
        }
    }
    
#if TOUCH_ENABLED
    // === TOUCH ZONE HINTS (right edge, centered in each zone) ===
    // No hlines — they overlap content rows. Small right-edge labels only.
    {
        int zone_x = TFT_WIDTH - MARGIN - FONT_W;  // Single-char width from right edge
        // Zone 1 (top):    y 0 .. TOUCH_ZONE_TOP_END  → center vertically
        int z1_y = (TOUCH_ZONE_TOP_END) / 2 - FONT_H / 2;
        // Zone 2 (middle): y TOUCH_ZONE_TOP_END+1 .. TOUCH_ZONE_MID_END
        int z2_y = (TOUCH_ZONE_TOP_END + 1 + TOUCH_ZONE_MID_END) / 2 - FONT_H / 2;
        // Zone 3 (bottom): y TOUCH_ZONE_MID_END+1 .. TFT_HEIGHT-1
        int z3_y = (TOUCH_ZONE_MID_END + 1 + TFT_HEIGHT) / 2 - FONT_H / 2;
        tft_draw_string(zone_x, z1_y, COL_DIM_LINE, "V");
        tft_draw_string(zone_x, z2_y, COL_DIM_LINE, "A");
        tft_draw_string(zone_x, z3_y, COL_DIM_LINE, "H");
    }
#endif
}

// ============================================================================
// Gauge View - Large Visual Indicators
// ============================================================================

// Convert angle in degrees to LUT index (step 2°, wrapping).
// Input angle can be negative.
static inline int angle_to_idx(int deg) {
    int idx = ((deg % 360) + 360) / 2 % SINCOS_LUT_SIZE;
    return idx;
}

static void draw_circular_gauge(int cx, int cy, int radius, float value, float max_value, 
                                const char* label, uint8_t color) {
    // Draw gauge outline using LUT
    for (int r = radius - 2; r <= radius; r++) {
        for (int ai = 0; ai < SINCOS_LUT_SIZE; ai++) {
            int x = cx + (r * lut_cos(ai) + 16384) / 32768;
            int y = cy + (r * lut_sin(ai) + 16384) / 32768;
            if (x >= 0 && x < TFT_WIDTH && y >= 0 && y < TFT_HEIGHT) {
                tft_input[y * TFT_WIDTH + x] = COL_DARK;
            }
        }
    }
    
    // Draw value arc using LUT
    float percentage = (value / max_value);
    if (percentage > 1.0f) percentage = 1.0f;
    int end_angle = (int)(percentage * 270.0f) - 135;
    
    for (int angle = -135; angle < end_angle && angle < 135; angle += 2) {
        int ai = angle_to_idx(angle);
        int16_t s = lut_sin(ai);
        int16_t c = lut_cos(ai);
        for (int r = radius - 8; r <= radius - 3; r++) {
            int x = cx + (r * c + 16384) / 32768;
            int y = cy + (r * s + 16384) / 32768;
            if (x >= 0 && x < TFT_WIDTH && y >= 0 && y < TFT_HEIGHT) {
                tft_input[y * TFT_WIDTH + x] = color;
            }
        }
    }
    
    // Draw value text in center (lean formatting)
    char value_str[16];
    char *p = u32_to_str(value_str, (uint32_t)(value + 0.5f));
    *p = '\0';
    int text_len = (int)(p - value_str);
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
    
    // Draw value text (lean formatting)
    char value_str[16];
    char *p = u32_to_str(value_str, (uint32_t)(value + 0.5f));
    *p = '\0';
    int text_len = (int)(p - value_str);
    tft_draw_string(x + width / 2 - (text_len * FONT_W) / 2, 
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
        char *tp = u32_to_str(temp_str, (uint32_t)(stats->bridge_temperature_c + 0.5f));
        *tp++ = 'C'; *tp = '\0';
        tft_draw_string(MARGIN, y, temp_color(stats->bridge_temperature_c), temp_str);
    }
    
    // Uptime (reuse pre-formatted buffer)
    int uptime_x = TFT_WIDTH - MARGIN - fmt_uptime.len * FONT_W;
    tft_draw_string(uptime_x, y, COL_CYAN, fmt_uptime.str);
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

#if TOUCH_ENABLED
    // Touch zone hints (right edge, centered in each zone — no hlines)
    {
        int zone_x = TFT_WIDTH - MARGIN - FONT_W;
        int z1_y = (TOUCH_ZONE_TOP_END) / 2 - FONT_H / 2;
        int z2_y = (TOUCH_ZONE_TOP_END + 1 + TOUCH_ZONE_MID_END) / 2 - FONT_H / 2;
        int z3_y = (TOUCH_ZONE_MID_END + 1 + TFT_HEIGHT) / 2 - FONT_H / 2;
        tft_draw_string(zone_x, z1_y, COL_DIM_LINE, "V");
        tft_draw_string(zone_x, z2_y, COL_DIM_LINE, "A");
        tft_draw_string(zone_x, z3_y, COL_DIM_LINE, "H");
    }
#endif
}

// ============================================================================
// Public API
// ============================================================================

// Forward declaration — timer callback defined below
static bool tft_render_timer_callback(repeating_timer_t *rt);

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
    // Initialize FT6206 capacitive touch controller over I2C
    // SDA = Arduino A4 (GPIO20), SCL = Arduino A5 (GPIO21)
    if (ft6206_init(i2c0, BRIDGE_TOUCH_SDA_PIN, BRIDGE_TOUCH_SCL_PIN)) {
        ft6206_set_callback(touch_event_callback);
        printf("[TFT] FT6206 capacitive touch initialized (SDA=%d SCL=%d)\n",
               BRIDGE_TOUCH_SDA_PIN, BRIDGE_TOUCH_SCL_PIN);
    } else {
        printf("[TFT] FT6206 touch controller not detected\n");
    }
#endif
    
    initialized = true;
    
    // Start background render timer (100ms = 10 FPS)
    // Negative interval means the timer fires every 100ms regardless of callback duration.
    add_repeating_timer_ms(-UPDATE_INTERVAL_MS, tft_render_timer_callback, NULL, &tft_render_timer);
    
    return true;
}

// ============================================================================
// Timer callback — fires every 100ms from hardware alarm IRQ
// Does all CPU-intensive work: formatting + drawing into framebuffer.
// Does NOT touch SPI/DMA (that stays on the main loop via flush).
// ============================================================================

static bool tft_render_timer_callback(repeating_timer_t *rt) {
    (void)rt;
    if (!initialized) return true;
    
    // Skip if the previous frame hasn't been flushed yet
    if (frame_ready) return true;
    
    // Grab latest stats (atomic-enough: single-word flag guard)
    if (!stats_pending) return true;
    tft_stats_t local_stats = shared_stats;  // Snapshot
    stats_pending = false;
    
    // Render into the back buffer
    tft_fill(COL_BG);
    format_stats(&local_stats);
    
    if (current_view == TFT_VIEW_GAUGES) {
        draw_gauge_view(&local_stats);
    } else {
        draw_stats(&local_stats);
    }
    
    // Track activity for next frame
    last_tx_bytes = local_stats.tx_bytes;
    last_rx_bytes = local_stats.rx_bytes;
    
    // Signal main loop that a frame is ready for DMA
    frame_ready = true;
    return true;  // Keep timer running
}

// ============================================================================
// Public API — lightweight calls for the main loop
// ============================================================================

void tft_display_submit_stats(const tft_stats_t *stats) {
    // Copy stats into shared buffer — called from main loop.
    // The timer ISR will pick these up on its next tick.
    shared_stats = *stats;
    stats_pending = true;
}

bool tft_display_flush(void) {
    // If the timer ISR has rendered a new frame, push it to the display.
    // This does the SPI DMA transfer (must run on Core0, not in ISR).
    if (!initialized || !frame_ready) return false;
    
    tft_swap_sync();
    frame_ready = false;
    return true;
}

void tft_display_update(const tft_stats_t *stats) {
    // Convenience wrapper: submit + flush in one call.
    // For best performance, use submit_stats/flush separately so the main
    // loop can do other work between submitting stats and flushing DMA.
    tft_display_submit_stats(stats);
    tft_display_flush();
}

void tft_display_refresh(const tft_stats_t *stats) {
    if (!initialized) return;
    
    // Force an immediate synchronous render (bypasses timer).
    tft_fill(COL_BG);
    format_stats(stats);
    
    if (current_view == TFT_VIEW_GAUGES) {
        draw_gauge_view(stats);
    } else {
        draw_stats(stats);
    }
    
    tft_swap_sync();
    
    last_tx_bytes = stats->tx_bytes;
    last_rx_bytes = stats->rx_bytes;
    frame_ready = false;
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
    // Poll FT6206 for touch events (handles debouncing internally)
    ft6206_poll();
    
    // Process zone-based touch flags
    if (touch_view_requested) {
        touch_view_requested = false;
        tft_display_toggle_view();
    }
    
    if (touch_api_requested) {
        touch_api_requested = false;
        // Signal API mode cycle request (checked by bridge main loop)
        extern volatile bool api_cycle_requested;
        api_cycle_requested = true;
    }
    
    if (touch_human_requested) {
        touch_human_requested = false;
        // Signal humanization cycle request (checked by bridge main loop)
        extern volatile bool humanization_cycle_requested;
        humanization_cycle_requested = true;
    }
#endif
}