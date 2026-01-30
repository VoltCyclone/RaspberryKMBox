/**
 * KMBox Bridge - Advanced Autopilot Firmware
 * 
 * This firmware runs on an Adafruit Feather RP2350 and provides:
 * - USB CDC interface for receiving RGB frames from PC
 * - Color-based target tracking with configurable parameters
 * - PIO UART TX to send mouse commands to RP2040 KMBox
 * - PIO UART RX to receive status messages from KMBox
 * - NeoPixel and onboard LED status indicators
 * 
 * Architecture:
 *   PC (capture tool) -> USB CDC -> RP2350 (this) -> PIO UART -> RP2040 (KMBox)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "tusb.h"

#include "config.h"
#include "tracker.h"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"
#include "../lib/kmbox-commands/kmbox_commands.h"

// PIO UART state
static PIO pio = pio0;
static uint sm_tx;
static uint sm_rx;
static uint tx_offset;
static uint rx_offset;

// Frame reception state
static uint8_t frame_buffer[FRAME_BUFFER_SIZE];
static volatile bool frame_ready = false;
static volatile uint16_t current_roi_w = ROI_DEFAULT_SIZE;
static volatile uint16_t current_roi_h = ROI_DEFAULT_SIZE;

// Status tracking
typedef enum {
    STATUS_BOOTING,
    STATUS_IDLE,
    STATUS_CDC_CONNECTED,
    STATUS_TRACKING,
    STATUS_DISABLED,
    STATUS_ERROR
} bridge_status_t;

static bridge_status_t current_status = STATUS_BOOTING;
static uint32_t status_change_time = 0;

// Status message buffer from KMBox
static char status_buffer[STATUS_BUFFER_SIZE];
static uint8_t status_buffer_idx = 0;

// KMBox Connection Management
typedef enum {
    KMBOX_DISCONNECTED,
    KMBOX_CONNECTED
} kmbox_connection_state_t;

static kmbox_connection_state_t kmbox_state = KMBOX_DISCONNECTED;
static uint32_t kmbox_last_rx_time = 0;
static uint32_t kmbox_last_ping_time = 0;
static uint32_t kmbox_ping_count = 0;
static uint32_t kmbox_response_count = 0;
static uint32_t kmbox_connect_attempts = 0;

// Connection timing (milliseconds)
#define KMBOX_PING_INTERVAL_MS      1000    // Send ping every 1s
#define KMBOX_TIMEOUT_MS            3000    // Consider disconnected after 3s no RX
#define KMBOX_CONNECT_RETRY_MS      1000    // Retry connect every 1s
#define KMBOX_INITIAL_DELAY_MS      500     // Wait for KMBox to boot

// CDC receive state machine
typedef enum {
    RX_STATE_IDLE,
    RX_STATE_HEADER,
    RX_STATE_PIXELS
} rx_state_t;

static rx_state_t rx_state = RX_STATE_IDLE;
static uint32_t rx_bytes_expected = 0;
static uint32_t rx_bytes_received = 0;

// Frame header from PC
typedef struct __attribute__((packed)) {
    uint8_t magic[2];      // 'F' 'R'
    uint16_t width;
    uint16_t height;
    uint16_t reserved;
} frame_header_t;

static frame_header_t pending_header;
static uint8_t header_bytes_received = 0;

// WS2812 NeoPixel PIO program (minimal inline version)
static const uint16_t ws2812_program_instructions[] = {
    0x6221, //  0: out    x, 1            side 0 [2]
    0x1123, //  1: jmp    !x, 3           side 1 [1]
    0x1400, //  2: jmp    0               side 1 [4]
    0xa442, //  3: nop                    side 0 [4]
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

static PIO ws2812_pio = pio1;
static uint ws2812_sm = 0;

// ============================================================================
// WS2812 NeoPixel Control
// ============================================================================

static void ws2812_init(void) {
    uint offset = pio_add_program(ws2812_pio, &ws2812_program);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    pio_gpio_init(ws2812_pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(ws2812_pio, ws2812_sm, WS2812_PIN, 1, true);
    sm_config_set_sideset_pins(&c, WS2812_PIN);
    
    float div = clock_get_hz(clk_sys) / (800000.0f * 10.0f);
    sm_config_set_clkdiv(&c, div);
    
    pio_sm_init(ws2812_pio, ws2812_sm, offset, &c);
    pio_sm_set_enabled(ws2812_pio, ws2812_sm, true);
}

static void ws2812_put_rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    pio_sm_put_blocking(ws2812_pio, ws2812_sm, grb << 8);
}

// ============================================================================
// PIO UART TX/RX Initialization
// ============================================================================

static void pio_uart_tx_init(uint pin, uint baud) {
    tx_offset = pio_add_program(pio, &uart_tx_program);
    sm_tx = pio_claim_unused_sm(pio, true);
    uart_tx_program_init(pio, sm_tx, tx_offset, pin, baud);
}

static inline void pio_uart_tx_byte(uint8_t b) {
    uart_tx_program_putc(pio, sm_tx, b);
}

static void pio_uart_rx_init(uint pin, uint baud) {
    rx_offset = pio_add_program(pio, &uart_rx_program);
    sm_rx = pio_claim_unused_sm(pio, true);
    uart_rx_program_init(pio, sm_rx, rx_offset, pin, baud);
}

static inline bool pio_uart_rx_available(void) {
    return uart_rx_program_available(pio, sm_rx);
}

static inline char pio_uart_rx_getc(void) {
    return uart_rx_program_getc(pio, sm_rx);
}

// ============================================================================
// UART RX Processing - Status Messages from KMBox
// ============================================================================

static void process_status_message(const char* msg, size_t len) {
    // Check for handshake response
    if (len >= 11 && strncmp(msg, "KMBOX_READY", 11) == 0) {
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            kmbox_response_count++;
            if (tud_cdc_connected()) {
                tud_cdc_write_str("[Bridge] KMBox connected (handshake OK)\r\n");
                tud_cdc_write_flush();
            }
        }
        return;  // Don't echo handshake messages
    }
    
    // Check for pong response  
    if (len >= 10 && strncmp(msg, "KMBOX_PONG", 10) == 0) {
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            kmbox_response_count++;
        }
        // Don't echo pong messages
        return;
    }
    
    // Forward other status messages to CDC
    if (tud_cdc_connected()) {
        tud_cdc_write_str("[KMBox] ");
        tud_cdc_write(msg, len);
        tud_cdc_write_str("\r\n");
        tud_cdc_write_flush();
    }
}

static uint32_t uart_rx_bytes_total = 0;
static uint32_t uart_tx_bytes_total = 0;

static void uart_rx_task(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    static uint8_t binary_packet[8];
    static uint8_t binary_idx = 0;
    static bool in_binary_packet = false;
    
    while (pio_uart_rx_available()) {
        uint8_t c = pio_uart_rx_getc();
        uart_rx_bytes_total++;
        kmbox_last_rx_time = now;  // Track last RX time
        
        // Check for start of binary response packet (0xFF or 0xFE from KMBox)
        if (!in_binary_packet && (c == 0xFF || c == 0xFE)) {
            in_binary_packet = true;
            binary_idx = 0;
            binary_packet[binary_idx++] = c;
            continue;
        }
        
        // Continue receiving binary packet
        if (in_binary_packet) {
            binary_packet[binary_idx++] = c;
            if (binary_idx >= 8) {
                // Complete packet received - forward to PC and process
                if (binary_packet[0] == 0xFF) {
                    kmbox_response_count++;
                    
                    // Forward response packet to PC over CDC
                    if (tud_cdc_connected()) {
                        tud_cdc_write(binary_packet, 8);
                        tud_cdc_write_flush();
                    }
                    
                    if (kmbox_state != KMBOX_CONNECTED) {
                        kmbox_state = KMBOX_CONNECTED;
                    }
                }
                in_binary_packet = false;
                binary_idx = 0;
            }
            continue;
        }
        
        // Process as text message
        if (c == '\n' || c == '\r') {
            if (status_buffer_idx > 0) {
                status_buffer[status_buffer_idx] = '\0';
                process_status_message(status_buffer, status_buffer_idx);
                status_buffer_idx = 0;
            }
        } else if (c >= 0x20 && c < 0x7F) {  // Printable ASCII only
            if (status_buffer_idx < STATUS_BUFFER_SIZE - 1) {
                status_buffer[status_buffer_idx++] = c;
            }
        }
        // Ignore other non-printable bytes
    }
}

// ============================================================================
// KMBox Connection Management
// ============================================================================

static void send_kmbox_ping(void) {
    // Send 8-byte ping packet: [0xFE][0][0][0][0][0][0][0]
    pio_uart_tx_byte(0xFE);  // FAST_CMD_PING
    for (int i = 0; i < 7; i++) {
        pio_uart_tx_byte(0x00);
    }
    kmbox_ping_count++;
    uart_tx_bytes_total += 8;
}

static void send_kmbox_handshake(void) {
    // Send bridge sync request - KMBox responds with "KMBOX_READY"
    const char* handshake = "KMBOX_BRIDGE_SYNC\n";
    while (*handshake) {
        pio_uart_tx_byte(*handshake++);
    }
    uart_tx_bytes_total += 18;
}

static void kmbox_connection_task(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Simple logic: any RX data within timeout = connected
    bool have_recent_data = (uart_rx_bytes_total > 0) && 
                            (now - kmbox_last_rx_time < KMBOX_TIMEOUT_MS);
    
    if (have_recent_data) {
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            if (tud_cdc_connected()) {
                tud_cdc_write_str("[Bridge] KMBox connected\r\n");
                tud_cdc_write_flush();
            }
        }
        
        // Send keepalive ping periodically
        if (now - kmbox_last_ping_time >= KMBOX_PING_INTERVAL_MS) {
            kmbox_last_ping_time = now;
            send_kmbox_ping();
        }
    } else {
        // No recent data - disconnected
        if (kmbox_state == KMBOX_CONNECTED) {
            kmbox_state = KMBOX_DISCONNECTED;
            if (tud_cdc_connected()) {
                tud_cdc_write_str("[Bridge] KMBox disconnected\r\n");
                tud_cdc_write_flush();
            }
        }
        
        // Try to reconnect periodically
        if (now - kmbox_last_ping_time >= KMBOX_CONNECT_RETRY_MS) {
            kmbox_last_ping_time = now;
            kmbox_connect_attempts++;
            
            // Alternate strategies
            if (kmbox_connect_attempts % 3 == 0) {
                send_kmbox_handshake();
            } else {
                send_kmbox_ping();
            }
        }
    }
}

static void uart_debug_task(void) {
    static uint32_t last_debug_time = 0;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_debug_time >= 5000) {  // Every 5 seconds
        last_debug_time = now;
        if (tud_cdc_connected()) {
            const char* state_str = (kmbox_state == KMBOX_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
            
            char debug_msg[128];
            snprintf(debug_msg, sizeof(debug_msg), 
                     "[Bridge] %s | TX: %lu | RX: %lu bytes\r\n", 
                     state_str, uart_tx_bytes_total, uart_rx_bytes_total);
            tud_cdc_write_str(debug_msg);
            tud_cdc_write_flush();
        }
    }
}

// ============================================================================
// Mouse Command Protocol - Send to KMBox
// ============================================================================

static void send_mouse_command(int16_t dx, int16_t dy, uint8_t buttons) {
    // 8-byte packet matching KMBox fast_cmd_move_t format
    pio_uart_tx_byte(CMD_MOUSE_MOVE);
    pio_uart_tx_byte(dx & 0xFF);
    pio_uart_tx_byte((dx >> 8) & 0xFF);
    pio_uart_tx_byte(dy & 0xFF);
    pio_uart_tx_byte((dy >> 8) & 0xFF);
    pio_uart_tx_byte(buttons);
    pio_uart_tx_byte(0);  // wheel
    pio_uart_tx_byte(0);  // padding
    uart_tx_bytes_total += 8;
}

// ============================================================================
// CDC Data Reception - Binary Commands, Text Commands, and Frames
// ============================================================================

// Fast command byte detection (matches KMBox protocol)
static inline bool is_fast_cmd_byte(uint8_t b) {
    return (b >= 0x01 && b <= 0x0B) || b == 0xFE;
}

// Text command buffer
static char cmd_buffer[128];
static uint8_t cmd_buffer_idx = 0;

// Handle text commands from PC
static void handle_text_command(const char* cmd) {
    // km.move(x,y) - relative mouse movement  
    int x, y;
    if (kmbox_parse_move_command(cmd, &x, &y)) {
        send_mouse_command(x, y, 0);
        if (tud_cdc_connected()) {
            tud_cdc_write_str("OK\r\n");
            tud_cdc_write_flush();
        }
        return;
    }
    
    // km.click(btn) - mouse click
    int btn;
    if (kmbox_parse_click_command(cmd, &btn)) {
        // Send click packet to KMBox
        pio_uart_tx_byte(0x02);  // FAST_CMD_MOUSE_CLICK
        pio_uart_tx_byte(btn);
        pio_uart_tx_byte(1);     // count
        for (int i = 0; i < 5; i++) pio_uart_tx_byte(0);
        uart_tx_bytes_total += 8;
        if (tud_cdc_connected()) {
            tud_cdc_write_str("OK\r\n");
            tud_cdc_write_flush();
        }
        return;
    }
    
    // km.version() - report version
    if (kmbox_is_version_command(cmd)) {
        if (tud_cdc_connected()) {
            tud_cdc_write_str("kmbox bridge v1.0\r\n");
            tud_cdc_write_flush();
        }
        return;
    }
    
    // ping - connection test
    if (strcmp(cmd, "ping") == 0) {
        // Send ping to KMBox and respond
        send_kmbox_ping();
        if (tud_cdc_connected()) {
            char resp[64];
            snprintf(resp, sizeof(resp), "pong (kmbox: %s)\r\n", 
                     kmbox_state == KMBOX_CONNECTED ? "connected" : "disconnected");
            tud_cdc_write_str(resp);
            tud_cdc_write_flush();
        }
        return;
    }
    
    // stats - show statistics
    if (strcmp(cmd, "stats") == 0) {
        if (tud_cdc_connected()) {
            char resp[128];
            snprintf(resp, sizeof(resp), 
                     "TX: %lu bytes | RX: %lu bytes | KMBox: %s\r\n",
                     uart_tx_bytes_total, uart_rx_bytes_total,
                     kmbox_state == KMBOX_CONNECTED ? "connected" : "disconnected");
            tud_cdc_write_str(resp);
            tud_cdc_write_flush();
        }
        return;
    }
    
    // Unknown command
    if (tud_cdc_connected()) {
        tud_cdc_write_str("ERR: unknown command\r\n");
        tud_cdc_write_flush();
    }
}

static void process_cdc_byte(uint8_t b) {
    switch (rx_state) {
        case RX_STATE_IDLE:
            if (b == FRAME_MAGIC_0) {
                header_bytes_received = 1;
                ((uint8_t*)&pending_header)[0] = b;
                rx_state = RX_STATE_HEADER;
            }
            break;

        case RX_STATE_HEADER:
            ((uint8_t*)&pending_header)[header_bytes_received++] = b;
            if (header_bytes_received == sizeof(frame_header_t)) {
                if (pending_header.magic[1] == FRAME_MAGIC_1 &&
                    pending_header.width <= ROI_MAX_SIZE &&
                    pending_header.height <= ROI_MAX_SIZE &&
                    pending_header.width > 0 &&
                    pending_header.height > 0) {
                    
                    rx_bytes_expected = pending_header.width * pending_header.height * 3;
                    rx_bytes_received = 0;
                    rx_state = RX_STATE_PIXELS;
                } else {
                    rx_state = RX_STATE_IDLE;
                }
            }
            break;

        case RX_STATE_PIXELS:
            if (rx_bytes_received < sizeof(frame_buffer)) {
                frame_buffer[rx_bytes_received++] = b;
            }
            if (rx_bytes_received >= rx_bytes_expected) {
                current_roi_w = pending_header.width;
                current_roi_h = pending_header.height;
                frame_ready = true;
                rx_state = RX_STATE_IDLE;
            }
            break;
    }
}

static void cdc_task(void) {
    if (!tud_cdc_connected() || !tud_cdc_available()) {
        return;
    }

    uint8_t buf[512];
    uint32_t count = tud_cdc_read(buf, sizeof(buf));
    
    uint32_t i = 0;
    while (i < count) {
        uint8_t b = buf[i];
        
        // Priority 1: Binary fast commands (0x01-0x0B, 0xFE)
        // Forward directly to KMBox via PIO UART
        if (is_fast_cmd_byte(b) && rx_state == RX_STATE_IDLE) {
            if (i + 8 <= count) {
                // Forward complete 8-byte packet to KMBox
                for (int j = 0; j < 8; j++) {
                    pio_uart_tx_byte(buf[i + j]);
                }
                uart_tx_bytes_total += 8;
                i += 8;
                continue;
            } else {
                // Incomplete packet at end - wait for more data
                break;
            }
        }
        
        // Priority 2: Frame data (in progress or starting with 'F')
        if (rx_state != RX_STATE_IDLE || b == FRAME_MAGIC_0) {
            process_cdc_byte(b);
            i++;
            continue;
        }
        
        // Priority 3: Text commands (printable ASCII)
        if (b >= 0x20 && b <= 0x7E) {
            if (cmd_buffer_idx < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_buffer_idx++] = b;
            }
            i++;
            continue;
        }
        
        // End of text command (CR or LF)
        if (b == '\r' || b == '\n') {
            if (cmd_buffer_idx > 0) {
                cmd_buffer[cmd_buffer_idx] = '\0';
                handle_text_command(cmd_buffer);
                cmd_buffer_idx = 0;
            }
            i++;
            continue;
        }
        
        // Skip unknown bytes
        i++;
    }
}

// ============================================================================
// Configuration Commands from PC
// ============================================================================

static void handle_config_command(const uint8_t* cmd, size_t len) {
    if (len < 2) return;
    
    switch (cmd[0]) {
        case CMD_CONFIG:
            if (len >= 9) {
                tracker_config_t cfg = {
                    .r_min = cmd[1],
                    .r_max = cmd[2],
                    .g_max = cmd[3],
                    .b_max = cmd[4],
                    .gain_x = cmd[5] / 10.0f,
                    .gain_y = cmd[6] / 10.0f,
                    .deadzone = cmd[7],
                    .min_blob_size = cmd[8]
                };
                tracker_set_config(&cfg);
            }
            break;
            
        case CMD_ENABLE:
            tracker_set_enabled(cmd[1] != 0);
            break;
            
        case CMD_STATUS:
            {
                tracker_stats_t stats;
                tracker_get_stats(&stats);
                char response[64];
                int n = snprintf(response, sizeof(response), 
                    "S:%d,%d,%u,%u\n",
                    stats.last_dx, stats.last_dy, 
                    stats.blob_size, stats.fps);
                tud_cdc_write(response, n);
                tud_cdc_write_flush();
            }
            break;
    }
}

// ============================================================================
// Status LED Management
// ============================================================================

static void heartbeat_task(void) {
    static uint32_t last_toggle = 0;
    uint32_t now = time_us_32();
    
    if (now - last_toggle > 500000) {
        gpio_xor_mask(1u << LED_PIN);
        last_toggle = now;
    }
}

static void set_status(bridge_status_t new_status) {
    if (current_status != new_status) {
        current_status = new_status;
        status_change_time = time_us_32();
    }
}

static void update_status_neopixel(void) {
    static uint32_t last_update = 0;
    static uint8_t activity_countdown = 0;
    static uint8_t brightness = 255;
    static int8_t brightness_dir = -2;
    uint32_t now = time_us_32();
    
    if (activity_countdown > 0) {
        ws2812_put_rgb(LED_COLOR_ACTIVITY);
        activity_countdown--;
        return;
    }
    
    if (now - last_update < 50000) return;
    last_update = now;
    
    if (!tracker_is_enabled()) {
        set_status(STATUS_DISABLED);
    } else if (frame_ready) {
        set_status(STATUS_TRACKING);
        activity_countdown = 2;
    } else if (tud_cdc_connected()) {
        set_status(STATUS_CDC_CONNECTED);
    } else {
        set_status(STATUS_IDLE);
    }
    
    bool apply_breathing = (current_status == STATUS_BOOTING || 
                           current_status == STATUS_IDLE ||
                           current_status == STATUS_ERROR);
    
    if (apply_breathing) {
        brightness += brightness_dir;
        if (brightness <= 50) {
            brightness = 50;
            brightness_dir = 2;
        } else if (brightness >= 255) {
            brightness = 255;
            brightness_dir = -2;
        }
    } else {
        brightness = 255;
    }
    
    uint8_t r, g, b;
    switch (current_status) {
        case STATUS_BOOTING:
            r = 0; g = 0; b = 255;
            break;
        case STATUS_IDLE:
            r = 255; g = 0; b = 0;
            break;
        case STATUS_CDC_CONNECTED:
            r = 0; g = 255; b = 255;
            break;
        case STATUS_TRACKING:
            r = 0; g = 255; b = 0;
            break;
        case STATUS_DISABLED:
            r = 255; g = 255; b = 0;
            break;
        case STATUS_ERROR:
        default:
            r = 255; g = 0; b = 0;
            break;
    }
    
    if (apply_breathing) {
        r = (r * brightness) / 255;
        g = (g * brightness) / 255;
        b = (b * brightness) / 255;
    }
    
    ws2812_put_rgb(r, g, b);
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(void) {
    stdio_init_all();
    sleep_ms(100);
    
    // Initialize GPIO
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    
    // Initialize peripherals
    ws2812_init();
    pio_uart_tx_init(UART_TX_PIN, UART_BAUD);
    pio_uart_rx_init(UART_RX_PIN, UART_BAUD);
    
    // Initialize tracker
    tracker_init();
    
    // Initial LED state
    ws2812_put_rgb(LED_COLOR_BOOTING);
    set_status(STATUS_BOOTING);
    
    // TinyUSB initialization
    tusb_init();
    
    // Initialize connection state with current time
    uint32_t boot_time = to_ms_since_boot(get_absolute_time());
    kmbox_last_rx_time = boot_time;
    kmbox_last_ping_time = boot_time;
    
    // Wait for KMBox to boot before trying to connect
    sleep_ms(KMBOX_INITIAL_DELAY_MS);
    
    // Performance tracking
    uint32_t last_frame_time = 0;
    uint32_t frame_count = 0;
    uint32_t fps_timer = 0;
    
    bool startup_msg_sent = false;
    
    while (1) {
        tud_task();
        cdc_task();
        uart_rx_task();
        kmbox_connection_task();  // Handle KMBox connection
        
        // Send startup message once CDC is connected
        if (tud_cdc_connected() && !startup_msg_sent) {
            sleep_ms(100);
            printf("\n=== KMBox Bridge Autopilot ===\n");
            printf("Board: Adafruit Feather RP2350\n");
            printf("System clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
            printf("\nUART TX: GPIO%d @ %d baud\n", UART_TX_PIN, UART_BAUD);
            printf("  PIO: pio0, SM: %d, Offset: %d\n", sm_tx, tx_offset);
            printf("\nUART RX: GPIO%d @ %d baud\n", UART_RX_PIN, UART_BAUD);
            printf("  PIO: pio0, SM: %d, Offset: %d\n", sm_rx, rx_offset);
            printf("  GPIO%d state: %d (should be 1 when idle)\n", UART_RX_PIN, gpio_get(UART_RX_PIN));
            printf("\nConnection: Auto-reconnect enabled (ping every %dms, timeout %dms)\n", 
                   KMBOX_PING_INTERVAL_MS, KMBOX_TIMEOUT_MS);
            printf("\nWaiting for KMBox connection...\n\n");
            startup_msg_sent = true;
        }
        
        // Process frame if ready
        if (frame_ready) {
            frame_ready = false;
            
            uint32_t now = time_us_32();
            
            tracker_result_t result;
            tracker_process_frame(frame_buffer, current_roi_w, current_roi_h, &result);
            
            // Only send commands if KMBox is connected
            if (result.valid && tracker_is_enabled() && kmbox_state == KMBOX_CONNECTED) {
                send_mouse_command(result.dx, result.dy, 0);
            }
            
            // FPS calculation
            frame_count++;
            if (now - fps_timer >= 1000000) {
                tracker_update_fps(frame_count);
                frame_count = 0;
                fps_timer = now;
            }
            
            last_frame_time = now;
        }
        
        // Update status indicators
        heartbeat_task();
        update_status_neopixel();
        uart_debug_task();
        
        tight_loop_contents();
    }
    
    return 0;
}