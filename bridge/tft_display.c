/**
 * TFT Display Driver Implementation
 * 
 * ST7735 driver with DMA-accelerated SPI transfers.
 * Updates are rate-limited and non-blocking to avoid
 * impacting UART communication.
 */

#include "tft_display.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// ST7735 Command Definitions
// ============================================================================
#define ST7735_NOP      0x00
#define ST7735_SWRESET  0x01
#define ST7735_SLPOUT   0x11
#define ST7735_NORON    0x13
#define ST7735_INVOFF   0x20
#define ST7735_INVON    0x21
#define ST7735_DISPOFF  0x28
#define ST7735_DISPON   0x29
#define ST7735_CASET    0x2A
#define ST7735_RASET    0x2B
#define ST7735_RAMWR    0x2C
#define ST7735_MADCTL   0x36
#define ST7735_COLMOD   0x3A
#define ST7735_FRMCTR1  0xB1
#define ST7735_FRMCTR2  0xB2
#define ST7735_FRMCTR3  0xB3
#define ST7735_INVCTR   0xB4
#define ST7735_PWCTR1   0xC0
#define ST7735_PWCTR2   0xC1
#define ST7735_PWCTR3   0xC2
#define ST7735_PWCTR4   0xC3
#define ST7735_PWCTR5   0xC4
#define ST7735_VMCTR1   0xC5
#define ST7735_GMCTRP1  0xE0
#define ST7735_GMCTRN1  0xE1

// MADCTL flags
#define MADCTL_MY       0x80
#define MADCTL_MX       0x40
#define MADCTL_MV       0x20
#define MADCTL_ML       0x10
#define MADCTL_RGB      0x00
#define MADCTL_BGR      0x08

// ============================================================================
// Internal State
// ============================================================================
static bool initialized = false;
static int dma_tx_chan = -1;
static volatile bool dma_busy = false;

// Frame buffer for partial updates (status bar area)
#define STATUS_BAR_HEIGHT   20
#define FRAMEBUF_SIZE       (TFT_WIDTH * STATUS_BAR_HEIGHT * 2)
static uint8_t framebuffer[FRAMEBUF_SIZE] __attribute__((aligned(4)));

// Update rate limiting
static uint32_t last_update_time = 0;
#define UPDATE_INTERVAL_MS  100  // 10 FPS max

// Cached stats for change detection
static tft_stats_t cached_stats = {0};
static bool needs_full_redraw = true;

// Simple 5x7 font (ASCII 32-127)
// Each character is 5 pixels wide, 7 pixels tall
// Stored as 5 bytes per character (column-major)
static const uint8_t font5x7[] = {
    0x00,0x00,0x00,0x00,0x00, // Space
    0x00,0x00,0x5F,0x00,0x00, // !
    0x00,0x07,0x00,0x07,0x00, // "
    0x14,0x7F,0x14,0x7F,0x14, // #
    0x24,0x2A,0x7F,0x2A,0x12, // $
    0x23,0x13,0x08,0x64,0x62, // %
    0x36,0x49,0x55,0x22,0x50, // &
    0x00,0x05,0x03,0x00,0x00, // '
    0x00,0x1C,0x22,0x41,0x00, // (
    0x00,0x41,0x22,0x1C,0x00, // )
    0x08,0x2A,0x1C,0x2A,0x08, // *
    0x08,0x08,0x3E,0x08,0x08, // +
    0x00,0x50,0x30,0x00,0x00, // ,
    0x08,0x08,0x08,0x08,0x08, // -
    0x00,0x60,0x60,0x00,0x00, // .
    0x20,0x10,0x08,0x04,0x02, // /
    0x3E,0x51,0x49,0x45,0x3E, // 0
    0x00,0x42,0x7F,0x40,0x00, // 1
    0x42,0x61,0x51,0x49,0x46, // 2
    0x21,0x41,0x45,0x4B,0x31, // 3
    0x18,0x14,0x12,0x7F,0x10, // 4
    0x27,0x45,0x45,0x45,0x39, // 5
    0x3C,0x4A,0x49,0x49,0x30, // 6
    0x01,0x71,0x09,0x05,0x03, // 7
    0x36,0x49,0x49,0x49,0x36, // 8
    0x06,0x49,0x49,0x29,0x1E, // 9
    0x00,0x36,0x36,0x00,0x00, // :
    0x00,0x56,0x36,0x00,0x00, // ;
    0x00,0x08,0x14,0x22,0x41, // <
    0x14,0x14,0x14,0x14,0x14, // =
    0x41,0x22,0x14,0x08,0x00, // >
    0x02,0x01,0x51,0x09,0x06, // ?
    0x32,0x49,0x79,0x41,0x3E, // @
    0x7E,0x11,0x11,0x11,0x7E, // A
    0x7F,0x49,0x49,0x49,0x36, // B
    0x3E,0x41,0x41,0x41,0x22, // C
    0x7F,0x41,0x41,0x22,0x1C, // D
    0x7F,0x49,0x49,0x49,0x41, // E
    0x7F,0x09,0x09,0x01,0x01, // F
    0x3E,0x41,0x41,0x51,0x32, // G
    0x7F,0x08,0x08,0x08,0x7F, // H
    0x00,0x41,0x7F,0x41,0x00, // I
    0x20,0x40,0x41,0x3F,0x01, // J
    0x7F,0x08,0x14,0x22,0x41, // K
    0x7F,0x40,0x40,0x40,0x40, // L
    0x7F,0x02,0x04,0x02,0x7F, // M
    0x7F,0x04,0x08,0x10,0x7F, // N
    0x3E,0x41,0x41,0x41,0x3E, // O
    0x7F,0x09,0x09,0x09,0x06, // P
    0x3E,0x41,0x51,0x21,0x5E, // Q
    0x7F,0x09,0x19,0x29,0x46, // R
    0x46,0x49,0x49,0x49,0x31, // S
    0x01,0x01,0x7F,0x01,0x01, // T
    0x3F,0x40,0x40,0x40,0x3F, // U
    0x1F,0x20,0x40,0x20,0x1F, // V
    0x7F,0x20,0x18,0x20,0x7F, // W
    0x63,0x14,0x08,0x14,0x63, // X
    0x03,0x04,0x78,0x04,0x03, // Y
    0x61,0x51,0x49,0x45,0x43, // Z
    0x00,0x00,0x7F,0x41,0x41, // [
    0x02,0x04,0x08,0x10,0x20, // backslash
    0x41,0x41,0x7F,0x00,0x00, // ]
    0x04,0x02,0x01,0x02,0x04, // ^
    0x40,0x40,0x40,0x40,0x40, // _
    0x00,0x01,0x02,0x04,0x00, // `
    0x20,0x54,0x54,0x54,0x78, // a
    0x7F,0x48,0x44,0x44,0x38, // b
    0x38,0x44,0x44,0x44,0x20, // c
    0x38,0x44,0x44,0x48,0x7F, // d
    0x38,0x54,0x54,0x54,0x18, // e
    0x08,0x7E,0x09,0x01,0x02, // f
    0x08,0x14,0x54,0x54,0x3C, // g
    0x7F,0x08,0x04,0x04,0x78, // h
    0x00,0x44,0x7D,0x40,0x00, // i
    0x20,0x40,0x44,0x3D,0x00, // j
    0x00,0x7F,0x10,0x28,0x44, // k
    0x00,0x41,0x7F,0x40,0x00, // l
    0x7C,0x04,0x18,0x04,0x78, // m
    0x7C,0x08,0x04,0x04,0x78, // n
    0x38,0x44,0x44,0x44,0x38, // o
    0x7C,0x14,0x14,0x14,0x08, // p
    0x08,0x14,0x14,0x18,0x7C, // q
    0x7C,0x08,0x04,0x04,0x08, // r
    0x48,0x54,0x54,0x54,0x20, // s
    0x04,0x3F,0x44,0x40,0x20, // t
    0x3C,0x40,0x40,0x20,0x7C, // u
    0x1C,0x20,0x40,0x20,0x1C, // v
    0x3C,0x40,0x30,0x40,0x3C, // w
    0x44,0x28,0x10,0x28,0x44, // x
    0x0C,0x50,0x50,0x50,0x3C, // y
    0x44,0x64,0x54,0x4C,0x44, // z
    0x00,0x08,0x36,0x41,0x00, // {
    0x00,0x00,0x7F,0x00,0x00, // |
    0x00,0x41,0x36,0x08,0x00, // }
    0x08,0x08,0x2A,0x1C,0x08, // ~
    0x08,0x1C,0x2A,0x08,0x08, // DEL (arrow)
};

// ============================================================================
// Low-level SPI functions
// ============================================================================

static inline void cs_select(void) {
    gpio_put(TFT_PIN_CS, 0);
}

static inline void cs_deselect(void) {
    gpio_put(TFT_PIN_CS, 1);
}

static inline void dc_command(void) {
    gpio_put(TFT_PIN_DC, 0);
}

static inline void dc_data(void) {
    gpio_put(TFT_PIN_DC, 1);
}

static void write_cmd(uint8_t cmd) {
    dc_command();
    cs_select();
    spi_write_blocking(TFT_SPI_PORT, &cmd, 1);
    cs_deselect();
}

static void write_data(const uint8_t *data, size_t len) {
    dc_data();
    cs_select();
    spi_write_blocking(TFT_SPI_PORT, data, len);
    cs_deselect();
}

static void write_data_byte(uint8_t data) {
    write_data(&data, 1);
}

// DMA completion handler
static void __attribute__((unused)) dma_handler(void) {
    if (dma_tx_chan >= 0) {
        dma_channel_acknowledge_irq0(dma_tx_chan);
    }
    cs_deselect();
    dma_busy = false;
}

// Note: DMA write function available for future optimization
// Currently using blocking SPI for simplicity and reliability
static bool __attribute__((unused)) write_data_dma(const uint8_t *data, size_t len) {
    if (dma_busy || dma_tx_chan < 0) {
        return false;
    }
    
    dc_data();
    cs_select();
    
    dma_busy = true;
    dma_channel_set_read_addr(dma_tx_chan, data, false);
    dma_channel_set_trans_count(dma_tx_chan, len, true);
    
    return true;
}

// ============================================================================
// ST7735 Initialization
// ============================================================================

static void st7735_init_sequence(void) {
    // Hardware reset
    gpio_put(TFT_PIN_RST, 0);
    sleep_ms(50);
    gpio_put(TFT_PIN_RST, 1);
    sleep_ms(50);
    
    // Software reset
    write_cmd(ST7735_SWRESET);
    sleep_ms(150);
    
    // Out of sleep mode
    write_cmd(ST7735_SLPOUT);
    sleep_ms(150);
    
    // Frame rate control
    write_cmd(ST7735_FRMCTR1);
    uint8_t frmctr[] = {0x01, 0x2C, 0x2D};
    write_data(frmctr, 3);
    
    write_cmd(ST7735_FRMCTR2);
    write_data(frmctr, 3);
    
    write_cmd(ST7735_FRMCTR3);
    write_data(frmctr, 3);
    uint8_t frmctr3[] = {0x01, 0x2C, 0x2D};
    write_data(frmctr3, 3);
    
    // Inversion control
    write_cmd(ST7735_INVCTR);
    write_data_byte(0x07);
    
    // Power control
    write_cmd(ST7735_PWCTR1);
    uint8_t pwctr1[] = {0xA2, 0x02, 0x84};
    write_data(pwctr1, 3);
    
    write_cmd(ST7735_PWCTR2);
    write_data_byte(0xC5);
    
    write_cmd(ST7735_PWCTR3);
    uint8_t pwctr3[] = {0x0A, 0x00};
    write_data(pwctr3, 2);
    
    write_cmd(ST7735_PWCTR4);
    uint8_t pwctr4[] = {0x8A, 0x2A};
    write_data(pwctr4, 2);
    
    write_cmd(ST7735_PWCTR5);
    uint8_t pwctr5[] = {0x8A, 0xEE};
    write_data(pwctr5, 2);
    
    // VCOM
    write_cmd(ST7735_VMCTR1);
    write_data_byte(0x0E);
    
    // Inversion off
    write_cmd(ST7735_INVOFF);
    
    // Memory data access control (rotation)
    write_cmd(ST7735_MADCTL);
    write_data_byte(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
    
    // 16-bit color (RGB565)
    write_cmd(ST7735_COLMOD);
    write_data_byte(0x05);
    
    // Gamma adjustment (positive)
    write_cmd(ST7735_GMCTRP1);
    uint8_t gmp[] = {0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
                    0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10};
    write_data(gmp, 16);
    
    // Gamma adjustment (negative)
    write_cmd(ST7735_GMCTRN1);
    uint8_t gmn[] = {0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                    0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10};
    write_data(gmn, 16);
    
    // Normal display on
    write_cmd(ST7735_NORON);
    sleep_ms(10);
    
    // Display on
    write_cmd(ST7735_DISPON);
    sleep_ms(100);
}

// ============================================================================
// Drawing Functions
// ============================================================================

static void set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    write_cmd(ST7735_CASET);
    uint8_t data[] = {0, (uint8_t)x0, 0, (uint8_t)x1};
    write_data(data, 4);
    
    write_cmd(ST7735_RASET);
    data[1] = y0;
    data[3] = y1;
    write_data(data, 4);
    
    write_cmd(ST7735_RAMWR);
}

static void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT) return;
    if (x + w > TFT_WIDTH) w = TFT_WIDTH - x;
    if (y + h > TFT_HEIGHT) h = TFT_HEIGHT - y;
    
    set_addr_window(x, y, x + w - 1, y + h - 1);
    
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    
    dc_data();
    cs_select();
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        spi_write_blocking(TFT_SPI_PORT, &hi, 1);
        spi_write_blocking(TFT_SPI_PORT, &lo, 1);
    }
    cs_deselect();
}

static void draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg) {
    if (c < 32 || c > 127) c = '?';
    
    const uint8_t *glyph = &font5x7[(c - 32) * 5];
    
    set_addr_window(x, y, x + 4, y + 6);
    
    dc_data();
    cs_select();
    
    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            uint16_t pixel = (glyph[col] & (1 << row)) ? color : bg;
            uint8_t hi = pixel >> 8;
            uint8_t lo = pixel & 0xFF;
            spi_write_blocking(TFT_SPI_PORT, &hi, 1);
            spi_write_blocking(TFT_SPI_PORT, &lo, 1);
        }
    }
    
    cs_deselect();
}

static void draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg) {
    while (*str) {
        draw_char(x, y, *str++, color, bg);
        x += 6;  // 5 pixels + 1 space
    }
}

static void draw_string_large(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg) {
    // Draw at 2x scale
    while (*str) {
        char c = *str++;
        if (c < 32 || c > 127) c = '?';
        const uint8_t *glyph = &font5x7[(c - 32) * 5];
        
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                uint16_t pixel = (glyph[col] & (1 << row)) ? color : bg;
                // Draw 2x2 block
                fill_rect(x + col * 2, y + row * 2, 2, 2, pixel);
            }
        }
        x += 12;  // 10 pixels + 2 space
    }
}

// ============================================================================
// Screen Layout Functions
// ============================================================================

// Draw a horizontal line separator
static void draw_separator(uint16_t y, uint16_t color) {
    fill_rect(0, y, TFT_WIDTH, 1, color);
}

static void draw_header(void) {
    // Title bar with gradient effect (darker at top)
    fill_rect(0, 0, TFT_WIDTH, 18, TFT_BLUE);
    fill_rect(0, 0, TFT_WIDTH, 2, 0x000F);  // Darker blue top edge
    draw_string(20, 5, "KMBox Bridge", TFT_WHITE, TFT_BLUE);
}

static void draw_status_section(const tft_stats_t *stats, uint16_t y) {
    // Connection status with status dots
    fill_rect(0, y, TFT_WIDTH, 12, TFT_BLACK);
    
    // KMBox status
    uint16_t kmbox_color = stats->kmbox_connected ? TFT_COLOR_OK : TFT_COLOR_ERROR;
    fill_rect(4, y + 2, 6, 6, kmbox_color);  // Status dot
    draw_string(14, y + 1, "KMBox", TFT_WHITE, TFT_BLACK);
    
    // CDC status  
    uint16_t cdc_color = stats->cdc_connected ? TFT_COLOR_OK : TFT_COLOR_INACTIVE;
    fill_rect(68, y + 2, 6, 6, cdc_color);  // Status dot
    draw_string(78, y + 1, "USB", TFT_WHITE, TFT_BLACK);
    
    // Mode indicator on right
    const char *mode_short[] = {"KM", "MK", "FE"};
    uint16_t mode_colors[] = {TFT_CYAN, TFT_MAGENTA, TFT_ORANGE};
    uint8_t mode = stats->api_mode;
    if (mode > 2) mode = 0;
    draw_string(108, y + 1, mode_short[mode], mode_colors[mode], TFT_BLACK);
}

static void draw_data_section(const tft_stats_t *stats, uint16_t y) {
    char buf[24];
    
    // Section header - UART Stats
    fill_rect(0, y, TFT_WIDTH, 42, TFT_BLACK);
    draw_string(4, y, "UART Stats", TFT_GRAY, TFT_BLACK);
    draw_separator(y + 9, TFT_DARKGRAY);
    
    // TX bytes and rate
    draw_string(4, y + 12, "TX:", TFT_GREEN, TFT_BLACK);
    if (stats->tx_bytes >= 1000000) {
        snprintf(buf, sizeof(buf), "%luM", stats->tx_bytes / 1000000);
    } else if (stats->tx_bytes >= 1000) {
        snprintf(buf, sizeof(buf), "%luK", stats->tx_bytes / 1000);
    } else {
        snprintf(buf, sizeof(buf), "%lu", stats->tx_bytes);
    }
    draw_string(28, y + 12, buf, TFT_GREEN, TFT_BLACK);
    
    // TX rate
    if (stats->tx_rate_bps >= 1000) {
        snprintf(buf, sizeof(buf), "%luK/s", stats->tx_rate_bps / 1000);
    } else {
        snprintf(buf, sizeof(buf), "%luB/s", stats->tx_rate_bps);
    }
    draw_string(80, y + 12, buf, TFT_GREEN, TFT_BLACK);
    
    // RX bytes and rate
    draw_string(4, y + 22, "RX:", TFT_CYAN, TFT_BLACK);
    if (stats->rx_bytes >= 1000000) {
        snprintf(buf, sizeof(buf), "%luM", stats->rx_bytes / 1000000);
    } else if (stats->rx_bytes >= 1000) {
        snprintf(buf, sizeof(buf), "%luK", stats->rx_bytes / 1000);
    } else {
        snprintf(buf, sizeof(buf), "%lu", stats->rx_bytes);
    }
    draw_string(28, y + 22, buf, TFT_CYAN, TFT_BLACK);
    
    // RX rate
    if (stats->rx_rate_bps >= 1000) {
        snprintf(buf, sizeof(buf), "%luK/s", stats->rx_rate_bps / 1000);
    } else {
        snprintf(buf, sizeof(buf), "%luB/s", stats->rx_rate_bps);
    }
    draw_string(80, y + 22, buf, TFT_CYAN, TFT_BLACK);
    
    // Baud rate info (dynamic from stats)
    if (stats->uart_baud >= 1000000) {
        snprintf(buf, sizeof(buf), "%luM 8N1", stats->uart_baud / 1000000);
    } else if (stats->uart_baud >= 1000) {
        snprintf(buf, sizeof(buf), "%luK 8N1", stats->uart_baud / 1000);
    } else {
        snprintf(buf, sizeof(buf), "%lu 8N1", stats->uart_baud);
    }
    draw_string(4, y + 32, buf, TFT_DARKGRAY, TFT_BLACK);
}

static void draw_injection_section(const tft_stats_t *stats, uint16_t y) {
    char buf[28];
    
    // Section header - Device Info
    fill_rect(0, y, TFT_WIDTH, 42, TFT_BLACK);
    draw_string(4, y, "Mouse", TFT_GRAY, TFT_BLACK);
    draw_separator(y + 9, TFT_DARKGRAY);
    
    // Product name (truncate if needed)
    if (stats->device_product[0]) {
        // Truncate to fit display (~20 chars max)
        char prod[21];
        strncpy(prod, stats->device_product, 20);
        prod[20] = '\0';
        draw_string(4, y + 12, prod, TFT_WHITE, TFT_BLACK);
    } else {
        draw_string(4, y + 12, "Waiting...", TFT_DARKGRAY, TFT_BLACK);
    }
    
    // VID:PID - always show (0000:0000 if no device)
    snprintf(buf, sizeof(buf), "%04X:%04X", stats->device_vid, stats->device_pid);
    draw_string(4, y + 22, buf, stats->device_vid ? TFT_CYAN : TFT_DARKGRAY, TFT_BLACK);
    
    // Manufacturer (if available)
    if (stats->device_manufacturer[0]) {
        char mfr[16];
        strncpy(mfr, stats->device_manufacturer, 15);
        mfr[15] = '\0';
        draw_string(64, y + 22, mfr, TFT_DARKGRAY, TFT_BLACK);
    }
    
    // Injection count
    snprintf(buf, sizeof(buf), "INJ:%lu", stats->mouse_clicks);
    draw_string(4, y + 32, buf, TFT_YELLOW, TFT_BLACK);
}

static void draw_mode_section(const tft_stats_t *stats, uint16_t y) {
    char buf[24];
    
    // Section header
    fill_rect(0, y, TFT_WIDTH, 32, TFT_BLACK);
    draw_string(4, y, "Bridge", TFT_GRAY, TFT_BLACK);
    draw_separator(y + 9, TFT_DARKGRAY);
    
    // API mode with color coding
    const char *mode_names[] = {"KMBox", "Makcu", "Ferrum"};
    uint16_t mode_colors[] = {TFT_CYAN, TFT_MAGENTA, TFT_ORANGE};
    uint8_t mode = stats->api_mode;
    if (mode > 2) mode = 0;
    
    draw_string(4, y + 12, "API:", TFT_LIGHTGRAY, TFT_BLACK);
    draw_string(34, y + 12, mode_names[mode], mode_colors[mode], TFT_BLACK);
    
    // Command count
    snprintf(buf, sizeof(buf), "Cmds:%lu", stats->mouse_moves);
    draw_string(76, y + 12, buf, TFT_LIGHTGRAY, TFT_BLACK);
    
    // Board info
    draw_string(4, y + 22, "Feather RP2350", TFT_DARKGRAY, TFT_BLACK);
}

static void draw_footer(const tft_stats_t *stats) {
    char buf[24];
    uint16_t y = TFT_HEIGHT - 14;
    
    // Footer bar
    fill_rect(0, y, TFT_WIDTH, 14, TFT_DARKGRAY);
    fill_rect(0, y, TFT_WIDTH, 1, 0x6B4D);  // Light edge
    
    // Uptime MM:SS
    uint32_t mins = stats->uptime_sec / 60;
    uint32_t secs = stats->uptime_sec % 60;
    snprintf(buf, sizeof(buf), "%02lu:%02lu", mins, secs);
    draw_string(4, y + 4, buf, TFT_WHITE, TFT_DARKGRAY);
    
    // CPU frequency on right
    snprintf(buf, sizeof(buf), "%luMHz", stats->cpu_freq_mhz);
    draw_string(88, y + 4, buf, TFT_LIGHTGRAY, TFT_DARKGRAY);
}

// ============================================================================
// Public API Implementation
// ============================================================================

bool tft_init(void) {
    if (initialized) return true;
    
    // Initialize SPI at high speed
    spi_init(TFT_SPI_PORT, TFT_SPI_FREQ);
    gpio_set_function(TFT_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(TFT_PIN_MOSI, GPIO_FUNC_SPI);
    
    // Initialize control pins
    gpio_init(TFT_PIN_CS);
    gpio_set_dir(TFT_PIN_CS, GPIO_OUT);
    gpio_put(TFT_PIN_CS, 1);
    
    gpio_init(TFT_PIN_DC);
    gpio_set_dir(TFT_PIN_DC, GPIO_OUT);
    
    gpio_init(TFT_PIN_RST);
    gpio_set_dir(TFT_PIN_RST, GPIO_OUT);
    gpio_put(TFT_PIN_RST, 1);
    
    // Initialize backlight with PWM
    gpio_set_function(TFT_PIN_BL, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(TFT_PIN_BL);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_PIN_BL), 255);
    pwm_set_enabled(slice, true);
    
    // Skip DMA setup - use blocking SPI for reliability
    // DMA IRQ can conflict with USB/TinyUSB
    dma_tx_chan = -1;
    
    // Initialize display
    st7735_init_sequence();
    
    // Clear screen
    fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
    
    initialized = true;
    needs_full_redraw = true;
    
    return true;
}

void tft_update(const tft_stats_t *stats) {
    if (!initialized || dma_busy) return;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_update_time < UPDATE_INTERVAL_MS) {
        return;  // Rate limit
    }
    last_update_time = now;
    
    // Full redraw on first update or if needed
    if (needs_full_redraw) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
        draw_header();
        needs_full_redraw = false;
    }
    
    // Update sections with simplified layout
    // Header: 0-18, Status: 20-32, UART: 34-76, Injection: 78-110, Mode: 112-144, Footer: 146-160
    draw_status_section(stats, 20);
    draw_data_section(stats, 34);
    draw_injection_section(stats, 78);
    draw_mode_section(stats, 112);
    draw_footer(stats);
    
    // Cache stats
    cached_stats = *stats;
}

void tft_refresh_now(const tft_stats_t *stats) {
    if (!initialized) return;
    
    // Wait for any pending DMA
    while (dma_busy) {
        tight_loop_contents();
    }
    
    needs_full_redraw = true;
    tft_update(stats);
}

void tft_show_splash(void) {
    if (!initialized) return;
    
    fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLUE);
    
    draw_string_large(20, 50, "KMBox", TFT_WHITE, TFT_BLUE);
    draw_string_large(14, 80, "Bridge", TFT_WHITE, TFT_BLUE);
    
    draw_string(30, 120, "Initializing...", TFT_CYAN, TFT_BLUE);
    
    // Mark that we need full redraw when updates start
    needs_full_redraw = true;
    // Reset last update time so first tft_update() isn't rate-limited
    last_update_time = 0;
}

void tft_show_error(const char *msg) {
    if (!initialized) return;
    
    fill_rect(0, 60, TFT_WIDTH, 40, TFT_RED);
    draw_string(4, 70, "ERROR:", TFT_WHITE, TFT_RED);
    draw_string(4, 82, msg, TFT_YELLOW, TFT_RED);
}

void tft_set_backlight(uint8_t brightness) {
    uint slice = pwm_gpio_to_slice_num(TFT_PIN_BL);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_PIN_BL), brightness);
}

bool tft_is_busy(void) {
    return dma_busy;
}

void tft_task(void) {
    // Currently just a placeholder for future async operations
    // The DMA interrupt handles completion
}
