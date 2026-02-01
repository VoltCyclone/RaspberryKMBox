/**
 * KMBox Bridge - Advanced Autopilot Firmware
 * 
 * This firmware runs on an Adafruit Feather RP2350 and provides:
 * - USB CDC interface for receiving RGB frames from PC
 * - Color-based target tracking with configurable parameters
 * - Hardware UART TX/RX for high-speed communication with RP2040 KMBox
 * - NeoPixel and onboard LED status indicators
 * - High-precision latency tracking using freed PIO resources
 * 
 * Architecture:
 *   PC (capture tool) -> USB CDC -> RP2350 (this) -> HW UART -> RP2040 (KMBox)
 * 
 * Note: TX/RX wires are crossed at hardware level, allowing direct
 * hardware UART usage instead of PIO-based pin swapping.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "tusb.h"

#include "config.h"
#include "tracker.h"
#include "hw_uart.h"
#include "latency_tracker.h"
#include "tft_display.h"
#include "bridge_protocol.h"
#include "makcu_protocol.h"
#include "makcu_translator.h"
#include "ferrum_protocol.h"
#include "ferrum_translator.h"
#include "core1_translator.h"
#include "../lib/kmbox-commands/kmbox_commands.h"

// PIO instance (used for WS2812 NeoPixel and timing, UART moved to hardware)
static PIO pio = pio0;

// Hardware UART is now used instead of PIO UART
// DMA circular buffers are handled internally by hw_uart module

// API Mode State (KMBox native, Makcu, or Ferrum)
// Note: api_mode_t is defined in makcu_protocol.h

static api_mode_t current_api_mode = API_MODE_KMBOX;
static uint32_t last_button_check = 0;
static bool button_state = false;
static uint32_t button_press_start = 0;
static bool button_init_done = false;

// Ferrum line buffer (file scope for mode change reset)
static char ferrum_line[256];
static uint8_t ferrum_idx = 0;

#define MODE_BUTTON_PIN 7
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONG_PRESS_MS 2000

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
#define KMBOX_PING_INTERVAL_MS      2000    // Send ping every 2s (was 1s)
#define KMBOX_TIMEOUT_MS            5000    // Consider disconnected after 5s no RX (was 3s)
#define KMBOX_CONNECT_RETRY_MS      2000    // Retry connect every 2s
#define KMBOX_INITIAL_DELAY_MS      500     // Wait for KMBox to boot

// TFT display tracking variables
static uint32_t boot_time_ms = 0;
static uint32_t tft_mouse_activity_count = 0;

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
    sm_config_set_wrap(&c, offset, offset + ws2812_program.length - 1);
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
// Hardware UART Wrappers (using hw_uart module)
// ============================================================================

// UART statistics from hw_uart module
static uint32_t uart_rx_bytes_total = 0;
static uint32_t uart_tx_bytes_total = 0;

// Wrapper functions for compatibility with existing code
static inline bool uart_rx_available(void) {
    return hw_uart_rx_available();
}

static inline uint8_t uart_rx_getc(void) {
    int c = hw_uart_getc();
    if (c >= 0) {
        uart_rx_bytes_total++;
        return (uint8_t)c;
    }
    return 0;
}

static inline void uart_tx_byte(uint8_t c) {
    hw_uart_putc(c);
    uart_tx_bytes_total++;
}

// Initialize hardware UART for bridge communication
static void hw_uart_bridge_init(void) {
    if (!hw_uart_init(UART_TX_PIN, UART_RX_PIN, UART_BAUD)) {
        printf("ERROR: Failed to initialize hardware UART!\n");
        return;
    }
    printf("Hardware UART initialized on GPIO%d(TX)/%d(RX) at %d baud\n", 
           UART_TX_PIN, UART_RX_PIN, UART_BAUD);
    printf("Note: Wires crossed at hardware level for direct UART connection\n");
}

// Button handling for API mode toggle
static void button_init(void) {
    gpio_init(MODE_BUTTON_PIN);
    gpio_set_dir(MODE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MODE_BUTTON_PIN);
    sleep_ms(10);  // Let pull-up stabilize
    button_state = !gpio_get(MODE_BUTTON_PIN);  // Read initial state
}

static void button_task(void) {
    // Button feature disabled - API mode fixed to KMBox
    // TODO: Fix button debouncing before re-enabling
    (void)last_button_check;
    (void)button_state;
    (void)button_press_start;
    (void)button_init_done;
}

// Send packet via Hardware UART (DMA accelerated)
static inline bool send_uart_packet(const uint8_t* data, size_t len) {
    if (hw_uart_send(data, len)) {
        uart_tx_bytes_total += len;
        return true;
    }
    return false;
}

// ============================================================================
// UART RX Processing - Status Messages from KMBox
// ============================================================================

// Echo test tracking (defined earlier in the file)
extern uint32_t echo_test_count;
extern uint32_t echo_response_count;

static void process_status_message(const char* msg, size_t len) {
    
    // Check for echo response: "ECHO:TEST..."
    if (len >= 5 && strncmp(msg, "ECHO:", 5) == 0) {
        echo_response_count++;
        if (tud_cdc_connected()) {
            char dbg[80];
            snprintf(dbg, sizeof(dbg), "[ECHO_OK] Got response: %s (sent=%lu, recv=%lu)\r\n", 
                     msg, echo_test_count, echo_response_count);
            tud_cdc_write_str(dbg);
            tud_cdc_write_flush();
        }
        // Auto-connect on successful echo
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
        }
        return;
    }
    
    // Check for handshake response
    if (len >= 11 && strncmp(msg, "KMBOX_READY", 11) == 0) {
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            kmbox_response_count++;
        }
        // Still forward handshake message
        if (tud_cdc_connected()) {
            tud_cdc_write_str("[KMBox] ");
            tud_cdc_write(msg, len);
            tud_cdc_write_str("\r\n");
            tud_cdc_write_flush();
        }
        return;
    }
    
    // Check for pong response  
    if (len >= 10 && strncmp(msg, "KMBOX_PONG", 10) == 0) {
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            kmbox_response_count++;
        }
        // Don't echo pong messages (too frequent)
        return;
    }
    
    // Forward ALL other status messages to CDC
    if (tud_cdc_connected()) {
        tud_cdc_write_str("[KMBox] ");
        tud_cdc_write(msg, len);
        tud_cdc_write_str("\r\n");
        tud_cdc_write_flush();
    }
}

static void uart_rx_task(void) {
    static uint8_t binary_packet[8];
    static uint8_t binary_idx = 0;
    static bool in_binary_packet = false;
    static uint32_t binary_packet_start_time = 0;
    static uint32_t rx_debug_count = 0;
    
    // Batch process up to 64 bytes per call to avoid blocking too long
    uint8_t batch_count = 0;
    while (uart_rx_available() && batch_count < 64) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint8_t c = uart_rx_getc();
        batch_count++;
        
        // Debug: show first 100 bytes received
        if (rx_debug_count < 100 && tud_cdc_connected()) {
            char dbg[32];
            snprintf(dbg, sizeof(dbg), "[RX:%02X '%c']", c, (c >= 0x20 && c < 0x7F) ? c : '.');
            tud_cdc_write_str(dbg);
            if ((rx_debug_count % 8) == 7) tud_cdc_write_str("\r\n");
            tud_cdc_write_flush();
            rx_debug_count++;
        }
        
        // Update last RX time for EVERY byte received
        kmbox_last_rx_time = now;
        
        // Reset binary packet state if timeout (stuck packet)
        if (in_binary_packet && (now - binary_packet_start_time) > 100) {
            in_binary_packet = false;
            binary_idx = 0;
        }
        
        // Check for start of binary response packet (0xFF or 0xFE from KMBox)
        if (!in_binary_packet && (c == 0xFF || c == 0xFE)) {
            in_binary_packet = true;
            binary_idx = 0;
            binary_packet[binary_idx++] = c;
            binary_packet_start_time = now;
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
            } else {
                // Buffer overflow - reset
                status_buffer_idx = 0;
            }
        }
        // Ignore other non-printable bytes
    }
}

// ============================================================================
// KMBox Connection Management
// ============================================================================

static void send_kmbox_ping(void) {
    // In KMBOX text mode, send a simple text ping instead of binary
    // This avoids corrupting the text command stream with 0xFE bytes
    if (current_api_mode == API_MODE_KMBOX) {
        // Send text ping: "P\n" - simple, won't interfere with M/W/B commands
        hw_uart_puts("P\n");
        kmbox_ping_count++;
        uart_tx_bytes_total += 2;
    } else {
        // Binary ping for other modes
        uint8_t ping_packet[8] = {0xFE, 0, 0, 0, 0, 0, 0, 0};
        send_uart_packet(ping_packet, 8);
        kmbox_ping_count++;
    }
}

// UART Echo test - send test string and look for echo response
uint32_t echo_test_count = 0;
uint32_t echo_response_count = 0;

static void send_uart_echo_test(void) {
    echo_test_count++;
    char test_msg[32];
    snprintf(test_msg, sizeof(test_msg), "ETEST%lu\n", echo_test_count);
    
    // Use direct UART write (not DMA) for reliability test
    for (const char* p = test_msg; *p; p++) {
        while (!uart_is_writable(uart0)) {
            tight_loop_contents();
        }
        uart_putc_raw(uart0, *p);
    }
    uart_tx_bytes_total += strlen(test_msg);
    
    if (tud_cdc_connected()) {
        char dbg[64];
        snprintf(dbg, sizeof(dbg), "[ECHO_TEST] Sent: %s", test_msg);
        tud_cdc_write_str(dbg);
        tud_cdc_write_flush();
    }
}

static void send_kmbox_handshake(void) {
    // Send bridge sync request - KMBox responds with "KMBOX_READY"
    hw_uart_puts("KMBOX_BRIDGE_SYNC\n");
    uart_tx_bytes_total += 18;
}

static void kmbox_connection_task(void) {
    static uint32_t last_rx_count = 0;
    static uint32_t ping_sent_time = 0;
    static bool ping_pending = false;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Check if new data has arrived since last check
    bool got_new_data = (uart_rx_bytes_total > last_rx_count);
    if (got_new_data) {
        last_rx_count = uart_rx_bytes_total;
        ping_pending = false;  // Got response, cancel pending timeout
        
        // Transition to connected if not already
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            if (tud_cdc_connected()) {
                tud_cdc_write_str("[Bridge] KMBox connected\r\n");
                tud_cdc_write_flush();
            }
        }
    }
    
    if (kmbox_state == KMBOX_CONNECTED) {
        // Send keepalive ping periodically
        if (now - kmbox_last_ping_time >= KMBOX_PING_INTERVAL_MS) {
            kmbox_last_ping_time = now;
            ping_sent_time = now;
            ping_pending = true;
            send_kmbox_ping();
        }
        
        // Check for ping timeout (no response after sending ping)
        if (ping_pending && (now - ping_sent_time >= KMBOX_TIMEOUT_MS)) {
            kmbox_state = KMBOX_DISCONNECTED;
            ping_pending = false;
            if (tud_cdc_connected()) {
                tud_cdc_write_str("[Bridge] KMBox disconnected (ping timeout)\r\n");
                tud_cdc_write_flush();
            }
        }
    } else {
        // Not connected - try to connect periodically
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
    
    if (now - last_debug_time >= 10000) {  // Every 10 seconds
        last_debug_time = now;
        if (tud_cdc_connected()) {
            const char* state_str = (kmbox_state == KMBOX_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
            
            char debug_msg[80];
            snprintf(debug_msg, sizeof(debug_msg), 
                     "[Bridge] %s | TX:%lu RX:%lu\r\n", 
                     state_str, uart_tx_bytes_total, uart_rx_bytes_total);
            tud_cdc_write_str(debug_msg);
            tud_cdc_write_flush();
        }
    }
}

// ============================================================================
// Mouse Command Protocol - Send to KMBox (Simple Text Protocol)
// ============================================================================

// Send a simple text command to KMBox: "M<x>,<y>\n"
static void send_text_mouse_move(int16_t dx, int16_t dy) {
    char cmd[24];
    int len = snprintf(cmd, sizeof(cmd), "M%d,%d\n", dx, dy);
    
    // Flash LED on Bridge side to confirm we're sending
    gpio_put(LED_PIN, 1);
    
    bool sent = send_uart_packet((const uint8_t*)cmd, len);
    uart_tx_bytes_total += len;
    tft_mouse_activity_count++;  // Track mouse commands for TFT display
    
    // Debug: show UART sends (always now for debugging)
    if (tud_cdc_connected()) {
        char dbg[64];
        snprintf(dbg, sizeof(dbg), "[UART>%s sent=%d len=%d]", cmd, sent, len);
        // Remove newline from debug output
        char *nl = strchr(dbg, '\n');
        if (nl) *nl = ' ';
        tud_cdc_write_str(dbg);
        tud_cdc_write_str("\r\n");
        tud_cdc_write_flush();
    }
    
    // Brief LED pulse
    sleep_us(100);
    gpio_put(LED_PIN, 0);
}

// Send transform command to KMBox: "km.transform(scale_x, scale_y, enabled)\n"
// scale: 256 = 1.0x, 0 = block, -256 = invert, 128 = 0.5x
static void send_transform_command(int16_t scale_x, int16_t scale_y, bool enabled) {
    char cmd[48];
    int len = snprintf(cmd, sizeof(cmd), "km.transform(%d,%d,%d)\n", 
                       scale_x, scale_y, enabled ? 1 : 0);
    
    send_uart_packet((const uint8_t*)cmd, len);
    uart_tx_bytes_total += len;
    
    if (tud_cdc_connected()) {
        char dbg[64];
        snprintf(dbg, sizeof(dbg), "[UART>transform %d,%d,%d]\r\n", 
                 scale_x, scale_y, enabled ? 1 : 0);
        tud_cdc_write_str(dbg);
        tud_cdc_write_flush();
    }
}

// Legacy binary send (keep for now)
static void send_mouse_command(int16_t dx, int16_t dy, uint8_t buttons) {
    // Use simple text protocol instead of binary
    send_text_mouse_move(dx, dy);
    
    // If buttons changed, could send button command
    // For now, ignore buttons in text mode
    (void)buttons;
}

// ============================================================================
// CDC Data Reception - Binary Commands, Text Commands, and Frames
// ============================================================================

// Fast command byte detection (matches KMBox protocol)
// Note: We exclude 0x0A (\n) and 0x0D (\r) since they are line terminators
static inline bool is_fast_cmd_byte(uint8_t b) {
    return (b >= 0x01 && b <= 0x0B && b != 0x0A) || b == 0xFE;
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
            tud_cdc_write_str(">>>\r\n");
            tud_cdc_write_flush();
        }
        return;
    }
    
    // km.transform(scale_x, scale_y, enabled) - set mouse transform on KMBox
    // scale: 256=1.0x, 0=block, -256=invert, 128=0.5x
    int scale_x, scale_y, enabled;
    if (sscanf(cmd, "km.transform(%d , %d , %d)", &scale_x, &scale_y, &enabled) == 3 ||
        sscanf(cmd, "km.transform(%d,%d,%d)", &scale_x, &scale_y, &enabled) == 3) {
        send_transform_command((int16_t)scale_x, (int16_t)scale_y, enabled != 0);
        if (tud_cdc_connected()) {
            tud_cdc_write_str(">>>\r\n");
            tud_cdc_write_flush();
        }
        return;
    }
    
    // km.transform() - query current transform (forward to KMBox)
    if (strncmp(cmd, "km.transform()", 14) == 0) {
        send_uart_packet((const uint8_t*)"km.transform()\n", 15);
        uart_tx_bytes_total += 15;
        return;
    }
    
    // km.click(btn) - mouse click
    int btn;
    if (kmbox_parse_click_command(cmd, &btn)) {
        // Send click packet to KMBox
        uint8_t click_pkt[8] = {0x02, (uint8_t)btn, 1, 0, 0, 0, 0, 0};
        send_uart_packet(click_pkt, 8);
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
            char resp[256];
            latency_stats_t lat_stats;
            latency_get_stats(&lat_stats);
            
            snprintf(resp, sizeof(resp), 
                     "TX: %lu bytes | RX: %lu bytes | KMBox: %s\r\n"
                     "Latency: min=%luus avg=%luus max=%luus jitter=%luus (n=%lu)\r\n",
                     uart_tx_bytes_total, uart_rx_bytes_total,
                     kmbox_state == KMBOX_CONNECTED ? "connected" : "disconnected",
                     lat_stats.min_us, lat_stats.avg_us, lat_stats.max_us,
                     lat_stats.jitter_us, lat_stats.sample_count);
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
        
        // Protocol detection and routing based on API mode
        if (current_api_mode == API_MODE_MAKCU && b == MAKCU_FRAME_START && rx_state == RX_STATE_IDLE) {
            // Makcu binary frame - Parse header to get full frame
            // Header: [0x50] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...]
            if (i + 4 <= count) {
                uint8_t cmd = buf[i + 1];
                uint16_t payload_len = buf[i + 2] | (buf[i + 3] << 8);
                
                // Check if full frame available
                if (i + 4 + payload_len <= count && payload_len <= MAKCU_MAX_PAYLOAD) {
                    // Queue complete frame to Core1 (just payload, Core1 knows the format)
                    core1_queue_makcu_frame(cmd, &buf[i + 4], payload_len);
                    i += 4 + payload_len;
                    continue;
                }
            }
            break;  // Incomplete frame, wait for more data
        }
        
        // Ferrum text protocol - line-based
        if (current_api_mode == API_MODE_FERRUM) {
            if (b == '\n' || b == '\r') {
                if (ferrum_idx > 0) {
                    ferrum_line[ferrum_idx] = '\0';
                    
                    // Translate Ferrum command to bridge protocol
                    ferrum_translated_t result;
                    if (ferrum_translate_line(ferrum_line, ferrum_idx, &result) && result.length > 0) {
                        // Debug: verify first byte is 0xBD
                        static uint32_t pkt_count = 0;
                        pkt_count++;
                        if (pkt_count <= 5 && tud_cdc_connected()) {
                            char dbg[64];
                            snprintf(dbg, sizeof(dbg), "[DBG] PKT#%lu: [%02X %02X %02X %02X %02X %02X] len=%zu\r\n",
                                     pkt_count, result.buffer[0], result.buffer[1], 
                                     result.buffer[2], result.buffer[3],
                                     result.buffer[4], result.buffer[5], result.length);
                            tud_cdc_write_str(dbg);
                            tud_cdc_write_flush();
                        }
                        
                        // Send translated bridge protocol packets
                        send_uart_packet(result.buffer, result.length);
                        uart_tx_bytes_total += result.length;
                        
                        if (result.needs_response && tud_cdc_connected()) {
                            tud_cdc_write_str(FERRUM_RESPONSE);
                            tud_cdc_write_flush();
                            // Force immediate transmission by calling tud_task()
                            tud_task();
                            // Blink LED to show response sent
                            gpio_put(LED_PIN, 1);
                            sleep_us(100);
                            gpio_put(LED_PIN, 0);
                        }
                    } else if (tud_cdc_connected()) {
                        tud_cdc_write_str("ERR\r\n");
                        tud_cdc_write_flush();
                        tud_task();
                    }
                    
                    ferrum_idx = 0;
                }
            } else if (ferrum_idx < sizeof(ferrum_line) - 1) {
                ferrum_line[ferrum_idx++] = b;
            }
            i++;
            continue;
        }
        
        // KMBox native mode - fast binary commands
        if (current_api_mode == API_MODE_KMBOX && is_fast_cmd_byte(b) && rx_state == RX_STATE_IDLE) {
            if (i + 8 <= count) {
                // Convert 8-byte KMBox command to bridge protocol
                uint8_t bridge_packet[7];
                size_t bridge_len = 0;
                
                // Fast path: direct mouse move translation
                if (b == 0x01) {  // FAST_CMD_MOUSE_MOVE
                    int16_t x = buf[i+1] | (buf[i+2] << 8);
                    int16_t y = buf[i+3] | (buf[i+4] << 8);
                    // Ignore buttons and wheel for text protocol
                    
                    // Send simple text command
                    send_text_mouse_move(x, y);
                }
                i += 8;
                continue;
            } else {
                break;  // Incomplete packet
            }
        }
        
        // Frame data (tracking frames)
        if (rx_state != RX_STATE_IDLE || b == FRAME_MAGIC_0) {
            process_cdc_byte(b);
            i++;
            continue;
        }
        
        // Text commands (printable ASCII)
        if (b >= 0x20 && b <= 0x7E) {
            if (cmd_buffer_idx < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_buffer_idx++] = b;
            }
            i++;
            continue;
        }
        
        // End of text command
        if (b == '\r' || b == '\n') {
            if (cmd_buffer_idx > 0) {
                cmd_buffer[cmd_buffer_idx] = '\0';
                handle_text_command(cmd_buffer);
                cmd_buffer_idx = 0;
            }
            i++;
            continue;
        }
        
        i++;
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
// TFT Display Update Task (non-blocking, rate-limited)
// ============================================================================

static void tft_update_task(void) {
    // Gather all statistics
    tft_stats_t stats = {0};  // Zero-initialize all fields
    
    // Connection status
    stats.kmbox_connected = (kmbox_state == KMBOX_CONNECTED);
    stats.cdc_connected = tud_cdc_connected();
    
    // API mode (numeric)
    stats.api_mode = (uint8_t)current_api_mode;
    
    // Data rates
    stats.tx_bytes = uart_tx_bytes_total;
    stats.rx_bytes = uart_rx_bytes_total;
    // tx_rate_bps and rx_rate_bps would require tracking delta over time
    // For now, leave at 0 (could add rate calculation later)
    
    // Latency statistics
    latency_stats_t lat_stats;
    latency_get_stats(&lat_stats);
    stats.latency_min_us = lat_stats.min_us;
    stats.latency_avg_us = lat_stats.avg_us;
    stats.latency_max_us = lat_stats.max_us;
    
    // Tracker statistics (for mouse activity)
    tracker_stats_t tracker_stats;
    tracker_get_stats(&tracker_stats);
    stats.last_dx = tracker_stats.last_dx;
    stats.last_dy = tracker_stats.last_dy;
    stats.mouse_moves = tft_mouse_activity_count;
    // stats.mouse_clicks not tracked yet
    
    // Uptime
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    stats.uptime_sec = (now_ms - boot_time_ms) / 1000;
    
    // CPU frequency
    stats.cpu_freq_mhz = clock_get_hz(clk_sys) / 1000000;
    
    // Call TFT update (internally rate-limited)
    tft_update(&stats);
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
    
    // Initialize peripherals (fast init only)
    ws2812_init();
    button_init();
    hw_uart_bridge_init();  // Hardware UART with DMA (replaces PIO UART)
    latency_tracker_init(); // Initialize latency tracking
    
    // Initialize Core1 translator
    core1_translator_init();
    
    // Initialize tracker
    tracker_init();
    
    // Initial LED state
    ws2812_put_rgb(LED_COLOR_BOOTING);
    set_status(STATUS_BOOTING);
    
    // TinyUSB initialization - MUST happen early for CDC to enumerate
    tusb_init();
    
    // Now init TFT (has long delays, but USB is already initializing)
    tft_init();             // Initialize TFT display (SPI1)
    tft_show_splash();      // Show splash screen
    
    // Initialize connection state
    uint32_t boot_time = to_ms_since_boot(get_absolute_time());
    boot_time_ms = boot_time;  // Store for TFT uptime display
    kmbox_last_rx_time = boot_time;
    kmbox_last_ping_time = boot_time;
    
    // Wait for KMBox to boot
    sleep_ms(KMBOX_INITIAL_DELAY_MS);
    
    uint32_t last_frame_time = 0;
    uint32_t frame_count = 0;
    uint32_t fps_timer = 0;
    bool startup_msg_sent = false;
    
    while (1) {
        tud_task();
        cdc_task();
        uart_rx_task();
        kmbox_connection_task();
        button_task();
        
        // Periodic UART echo test (every 2 seconds while not connected)
        static uint32_t last_echo_test = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (kmbox_state != KMBOX_CONNECTED && (now - last_echo_test >= 2000)) {
            last_echo_test = now;
            send_uart_echo_test();
        }
        
        // Dequeue translated commands from Core1 and send
        uint8_t translated_packet[8];
        while (core1_has_kmbox_commands()) {
            if (core1_dequeue_kmbox_command(translated_packet)) {
                // Translate 8-byte KMBox to optimized bridge protocol
                // (Core1 outputs old format, we convert to new)
                int16_t x = translated_packet[1] | (translated_packet[2] << 8);
                int16_t y = translated_packet[3] | (translated_packet[4] << 8);
                
                uint8_t bridge_pkt[7];
                size_t len = bridge_build_mouse_move(bridge_pkt, x, y);
                send_uart_packet(bridge_pkt, len);
            }
        }
        
        // Send startup message once CDC is connected
        if (tud_cdc_connected() && !startup_msg_sent) {
            sleep_ms(100);
            printf("\n=== KMBox Bridge with Hardware UART ===\n");
            printf("Board: Adafruit Feather RP2350\n");
            printf("System clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
            printf("Core1: Protocol translator active\n");
            printf("UART: Hardware UART0 @ %d baud (DMA accelerated)\n", UART_BAUD);
            printf("  TX: GPIO%d -> crossed wire -> KMBox RX\n", UART_TX_PIN);
            printf("  RX: GPIO%d <- crossed wire <- KMBox TX\n", UART_RX_PIN);
            printf("PIO: Available for timing/WS2812 (freed from UART duty)\n");
            printf("Protocols: KMBox, Makcu, Ferrum (button to toggle)\n");
            printf("Waiting for KMBox connection...\n\n");
            startup_msg_sent = true;
        }
        
        // Process frame if ready
        if (frame_ready) {
            frame_ready = false;
            
            uint32_t now = time_us_32();
            
            // Track command latency
            uint32_t timing_start = latency_start_timing();
            
            tracker_result_t result;
            tracker_process_frame(frame_buffer, current_roi_w, current_roi_h, &result);
            
            // Only send commands if KMBox is connected
            if (result.valid && tracker_is_enabled() && kmbox_state == KMBOX_CONNECTED) {
                send_mouse_command(result.dx, result.dy, 0);
            }
            
            latency_end_timing(timing_start);
            
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
        tft_update_task();  // Update TFT display (non-blocking, rate-limited)
        
        tight_loop_contents();
    }
    
    return 0;
}