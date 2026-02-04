/*
 * KMBox Serial Command Handler
 * 
 * Handles UART communication with RP2350 bridge for mouse/keyboard injection.
 * Supports text protocol (M<x>,<y>) and 8-byte binary protocol for low latency.
 */

#include "kmbox_serial_handler.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "bridge_protocol.h"
#include "usb_hid.h"
#include "led_control.h"
#include "smooth_injection.h"
#include "uart_buffers.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include <string.h>
#include <stdio.h>

//--------------------------------------------------------------------+
// Configuration
//--------------------------------------------------------------------+

#define UART_RX_BUFFER_SIZE     256
#define UART_RX_BUFFER_MASK     (UART_RX_BUFFER_SIZE - 1)

//--------------------------------------------------------------------+
// Temperature Sensor (with caching for performance)
//--------------------------------------------------------------------+

static int16_t g_cached_temperature = 250;  // 25.0°C default
static uint32_t g_last_temp_read_ms = 0;
#define TEMP_CACHE_MS 1000

static inline int16_t read_temperature_decidegrees(void) {
    adc_select_input(4);
    uint16_t raw = adc_read();
    float voltage = raw * (3.3f / 4096.0f);
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;
    return (int16_t)(temp_c * 10.0f);  // Return in 0.1°C units
}

static inline int16_t get_cached_temperature(uint32_t now_ms) {
    if ((now_ms - g_last_temp_read_ms) >= TEMP_CACHE_MS) {
        g_cached_temperature = read_temperature_decidegrees();
        g_last_temp_read_ms = now_ms;
    }
    return g_cached_temperature;
}

//--------------------------------------------------------------------+
// Helper Functions
//--------------------------------------------------------------------+

static inline int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}

//--------------------------------------------------------------------+
// RX Ring Buffer (IRQ producer, main loop consumer)
//--------------------------------------------------------------------+

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE] __attribute__((aligned(UART_RX_BUFFER_SIZE)));
static volatile uint16_t uart_rx_head = 0;  // IRQ writes here
static volatile uint16_t uart_rx_tail = 0;  // Main loop reads here

//--------------------------------------------------------------------+
// TX Ring Buffer (Non-blocking IRQ-based transmission)
//--------------------------------------------------------------------+

static uart_tx_buffer_t g_uart_tx_buffer;
static volatile uint32_t g_uart_tx_dropped = 0;

// Forward declarations for IRQ handlers
static void __not_in_flash_func(on_uart_rx)(void);
static void __not_in_flash_func(on_uart_tx)(void);
static void __not_in_flash_func(on_uart_irq)(void);

static inline bool rx_put(uint8_t byte) {
    uint16_t next = (uart_rx_head + 1) & UART_RX_BUFFER_MASK;
    if (next == uart_rx_tail) return false;  // Full
    uart_rx_buffer[uart_rx_head] = byte;
    uart_rx_head = next;
    return true;
}

static inline uint16_t rx_available(void) {
    return (uart_rx_head - uart_rx_tail) & UART_RX_BUFFER_MASK;
}

static inline uint8_t rx_peek_at(uint16_t offset) {
    return uart_rx_buffer[(uart_rx_tail + offset) & UART_RX_BUFFER_MASK];
}

static inline void rx_consume(uint16_t count) {
    uart_rx_tail = (uart_rx_tail + count) & UART_RX_BUFFER_MASK;
}

//--------------------------------------------------------------------+
// Connection State
//--------------------------------------------------------------------+

static bridge_connection_state_t g_connection_state = BRIDGE_STATE_WAITING;
static uint32_t g_last_data_time_ms = 0;
static uint32_t g_last_heartbeat_check_ms = 0;
static uint32_t g_last_ping_time_ms = 0;

//--------------------------------------------------------------------+
// Clock Sync State (for timed commands)
//--------------------------------------------------------------------+

static struct {
    uint64_t device_base_us;
    uint32_t pc_base_us;
    uint8_t  seq_num;
    bool     synced;
} clock_sync = {0};

//--------------------------------------------------------------------+
// Timed Move (non-blocking scheduled execution)
//--------------------------------------------------------------------+

static struct {
    uint64_t target_time_us;
    int16_t x, y;
    uint8_t mode;
    bool pending;
} pending_timed_move = {0};

static inline void process_pending_timed_move(void) {
    if (pending_timed_move.pending && time_us_64() >= pending_timed_move.target_time_us) {
        inject_mode_t mode = (pending_timed_move.mode <= 3) 
            ? (inject_mode_t)pending_timed_move.mode : INJECT_MODE_SMOOTH;
        smooth_inject_movement(pending_timed_move.x, pending_timed_move.y, mode);
        pending_timed_move.pending = false;
    }
}

//--------------------------------------------------------------------+
// Fast Binary Command Stats
//--------------------------------------------------------------------+

static uint32_t fast_cmd_count = 0;
static uint32_t fast_cmd_errors = 0;

//--------------------------------------------------------------------+
// UART IRQ Handlers (RX and TX)
//--------------------------------------------------------------------+

static void __not_in_flash_func(on_uart_rx)(void) {
    while (uart_is_readable(KMBOX_UART)) {
        rx_put(uart_getc(KMBOX_UART));
    }
}

// Combined UART IRQ handler
static void __not_in_flash_func(on_uart_irq)(void) {
    // Handle RX
    if (uart_is_readable(KMBOX_UART)) {
        on_uart_rx();
    }
    
    // Handle TX
    if (uart_is_writable(KMBOX_UART) && g_uart_tx_buffer.tx_active) {
        on_uart_tx();
    }
}

//--------------------------------------------------------------------+
// UART TX IRQ Handler - sends next byte from ring buffer
//--------------------------------------------------------------------+

static void __not_in_flash_func(on_uart_tx)(void) {
    while (uart_is_writable(KMBOX_UART) && tx_buffer_has_data(&g_uart_tx_buffer)) {
        int byte = tx_buffer_get(&g_uart_tx_buffer);
        if (byte >= 0) {
            uart_putc_raw(KMBOX_UART, (uint8_t)byte);
        } else {
            break;
        }
    }
    
    // Disable TX IRQ if buffer empty
    if (!tx_buffer_has_data(&g_uart_tx_buffer)) {
        uart_set_irq_enables(KMBOX_UART, true, false);  // RX enabled, TX disabled
        g_uart_tx_buffer.tx_active = false;
    }
}

// Non-blocking TX: Add bytes to ring buffer and enable TX IRQ
static bool uart_send_bytes(const uint8_t *data, size_t len) {
    if (!data || len == 0) return true;
    
    // Only disable the UART IRQ, not all interrupts (allows USB to continue)
    int uart_irq = (KMBOX_UART == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(uart_irq, false);
    
    uint16_t written = tx_buffer_write(&g_uart_tx_buffer, data, len);
    
    // Enable TX IRQ to start transmission
    if (written > 0 && !g_uart_tx_buffer.tx_active) {
        g_uart_tx_buffer.tx_active = true;
        uart_set_irq_enables(KMBOX_UART, true, true);  // Enable both RX and TX IRQ
    }
    
    irq_set_enabled(uart_irq, true);
    
    // Track dropped bytes
    if (written < len) {
        g_uart_tx_dropped += (len - written);
        return false;
    }
    
    return true;
}

static bool uart_send_string(const char *str) {
    if (!str) return true;
    return uart_send_bytes((const uint8_t*)str, strlen(str));
}

//--------------------------------------------------------------------+
// Public TX API (Non-blocking)
//--------------------------------------------------------------------+

void kmbox_send_response(const char *response) {
    if (!response) return;
    neopixel_trigger_activity_flash_color(0x0000FF00);
    uart_send_string(response);
    uart_send_string("\r\n");
    // Non-blocking: TX IRQ handles actual transmission
}

void kmbox_send_status(const char *message) {
    if (message) kmbox_send_response(message);
}

void kmbox_send_ping_to_bridge(void) {
    uint8_t ping[8] = {FAST_CMD_PING, 0, 0, 0, 0, 0, 0, 0};
    neopixel_trigger_activity_flash_color(0x0000FF00);
    uart_send_bytes(ping, 8);
    // Non-blocking: TX IRQ handles actual transmission
}

// Get TX buffer stats (for debugging)
uint32_t kmbox_get_tx_dropped_bytes(void) {
    return g_uart_tx_dropped;
}

//--------------------------------------------------------------------+
// Connection State Management
//--------------------------------------------------------------------+

bridge_connection_state_t kmbox_get_connection_state(void) {
    return g_connection_state;
}

bool kmbox_is_connected(void) {
    return (g_connection_state == BRIDGE_STATE_CONNECTED || 
            g_connection_state == BRIDGE_STATE_ACTIVE);
}

static void set_connection_state(bridge_connection_state_t new_state) {
    if (g_connection_state == new_state) return;
    g_connection_state = new_state;
    
    switch (new_state) {
        case BRIDGE_STATE_WAITING:     neopixel_set_status_override(STATUS_BRIDGE_WAITING); break;
        case BRIDGE_STATE_CONNECTING:  neopixel_set_status_override(STATUS_BRIDGE_CONNECTING); break;
        case BRIDGE_STATE_CONNECTED:   neopixel_set_status_override(STATUS_BRIDGE_CONNECTED); break;
        case BRIDGE_STATE_ACTIVE:      neopixel_set_status_override(STATUS_BRIDGE_ACTIVE); break;
        case BRIDGE_STATE_DISCONNECTED: neopixel_set_status_override(STATUS_BRIDGE_DISCONNECTED); break;
    }
}

static void check_connection_timeout(uint32_t now_ms) {
    if (now_ms - g_last_heartbeat_check_ms < BRIDGE_HEARTBEAT_CHECK_MS) return;
    g_last_heartbeat_check_ms = now_ms;
    
    if (kmbox_is_connected() && (now_ms - g_last_data_time_ms > BRIDGE_HEARTBEAT_TIMEOUT_MS)) {
        set_connection_state(BRIDGE_STATE_DISCONNECTED);
    }
    
    if (g_connection_state == BRIDGE_STATE_DISCONNECTED &&
        (now_ms - g_last_data_time_ms > BRIDGE_HEARTBEAT_TIMEOUT_MS * 2)) {
        set_connection_state(BRIDGE_STATE_WAITING);
    }
}

static inline void mark_activity(uint32_t now_ms) {
    g_last_data_time_ms = now_ms;
    if (!kmbox_is_connected()) set_connection_state(BRIDGE_STATE_CONNECTED);
    if (g_connection_state == BRIDGE_STATE_CONNECTED) set_connection_state(BRIDGE_STATE_ACTIVE);
}

//--------------------------------------------------------------------+
// Bridge Protocol Parser (variable-length binary, sync byte 0xBD)
//--------------------------------------------------------------------+

static uint8_t __not_in_flash_func(process_bridge_packet)(const uint8_t *data, size_t available) {
    if (available < 2) return 0;
    if (data[0] != BRIDGE_SYNC_BYTE) return 1;  // Skip invalid sync
    
    switch (data[1]) {
        case BRIDGE_CMD_MOUSE_MOVE:
            if (available < 6) return 0;
            kmbox_add_mouse_movement(read_i16_le(data + 2), read_i16_le(data + 4));
            return 6;
            
        case BRIDGE_CMD_MOUSE_WHEEL:
            if (available < 3) return 0;
            kmbox_add_wheel_movement((int8_t)data[2]);
            return 3;
            
        case BRIDGE_CMD_BUTTON_SET:
            if (available < 4) return 0;
            kmbox_update_physical_buttons(data[3] ? data[2] : 0);
            return 4;
            
        case BRIDGE_CMD_MOUSE_MOVE_WHEEL:
            if (available < 7) return 0;
            kmbox_add_mouse_movement(read_i16_le(data + 2), read_i16_le(data + 4));
            kmbox_add_wheel_movement((int8_t)data[6]);
            return 7;
            
        case BRIDGE_CMD_PING:
            return 2;
            
        case BRIDGE_CMD_RESET:
            kmbox_add_mouse_movement(0, 0);
            kmbox_add_wheel_movement(0);
            kmbox_update_physical_buttons(0);
            return 2;
            
        default:
            return 1;  // Unknown command, skip sync byte
    }
}

//--------------------------------------------------------------------+
// Fast Binary Command Processing (8-byte fixed packets)
//--------------------------------------------------------------------+

extern void process_mouse_report(const hid_mouse_report_t *report);
extern void process_kbd_report(const hid_keyboard_report_t *report);

static inline bool is_fast_cmd_start(uint8_t byte) {
    if (byte == 0x0A || byte == 0x0D) return false;  // Exclude line terminators
    return (byte >= FAST_CMD_MOUSE_MOVE && byte <= FAST_CMD_INFO) || byte == FAST_CMD_PING;
}

static bool __not_in_flash_func(process_fast_command)(const uint8_t *pkt) {
    switch (pkt[0]) {
        case FAST_CMD_MOUSE_MOVE: {
            const fast_cmd_move_t *m = (const fast_cmd_move_t *)pkt;
            hid_mouse_report_t report = {
                .buttons = m->buttons,
                .x = kmbox_clamp_movement_i8(m->x),
                .y = kmbox_clamp_movement_i8(m->y),
                .wheel = m->wheel
            };
            process_mouse_report(&report);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_MOUSE_CLICK: {
            const fast_cmd_click_t *c = (const fast_cmd_click_t *)pkt;
            uint8_t btn_mask = kmbox_map_button_to_hid_mask(c->button);
            uint8_t count = c->count ? c->count : 1;
            hid_mouse_report_t report = {0};
            for (uint8_t i = 0; i < count && i < 10; i++) {
                report.buttons = btn_mask;
                process_mouse_report(&report);
                report.buttons = 0;
                process_mouse_report(&report);
            }
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_KEY_PRESS: {
            const fast_cmd_key_t *k = (const fast_cmd_key_t *)pkt;
            hid_keyboard_report_t report = {.modifier = k->modifiers, .keycode = {k->keycode}};
            process_kbd_report(&report);
            memset(&report, 0, sizeof(report));
            process_kbd_report(&report);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_KEY_COMBO: {
            const fast_cmd_combo_t *c = (const fast_cmd_combo_t *)pkt;
            hid_keyboard_report_t report = {.modifier = c->modifiers};
            uint8_t n = 0;
            for (int i = 0; i < 4 && c->keys[i]; i++) report.keycode[n++] = c->keys[i];
            process_kbd_report(&report);
            memset(&report, 0, sizeof(report));
            process_kbd_report(&report);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_MULTI_MOVE: {
            const fast_cmd_multi_t *m = (const fast_cmd_multi_t *)pkt;
            hid_mouse_report_t report = {0};
            if (m->x1 || m->y1) { report.x = m->x1; report.y = m->y1; process_mouse_report(&report); }
            if (m->x2 || m->y2) { report.x = m->x2; report.y = m->y2; process_mouse_report(&report); }
            if (m->x3 || m->y3) { report.x = m->x3; report.y = m->y3; process_mouse_report(&report); }
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_MOVE: {
            const fast_cmd_smooth_t *s = (const fast_cmd_smooth_t *)pkt;
            inject_mode_t mode = (s->mode <= 3) ? (inject_mode_t)s->mode : INJECT_MODE_SMOOTH;
            smooth_inject_movement(s->x, s->y, mode);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_CONFIG: {
            const fast_cmd_config_t *c = (const fast_cmd_config_t *)pkt;
            if (c->max_per_frame > 0) smooth_set_max_per_frame(c->max_per_frame);
            smooth_set_velocity_matching(c->vel_match != 0);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_CLEAR:
            smooth_clear_queue();
            fast_cmd_count++;
            return true;
        
        case FAST_CMD_TIMED_MOVE: {
            const fast_cmd_timed_t *t = (const fast_cmd_timed_t *)pkt;
            if (clock_sync.synced && t->time_us > 0) {
                uint64_t target = clock_sync.device_base_us + t->time_us;
                uint64_t now = time_us_64();
                if (target > now && (target - now) < 100000) {
                    pending_timed_move.target_time_us = target;
                    pending_timed_move.x = t->x;
                    pending_timed_move.y = t->y;
                    pending_timed_move.mode = t->mode;
                    pending_timed_move.pending = true;
                    fast_cmd_count++;
                    return true;
                }
            }
            inject_mode_t mode = (t->mode <= 3) ? (inject_mode_t)t->mode : INJECT_MODE_SMOOTH;
            smooth_inject_movement(t->x, t->y, mode);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SYNC: {
            const fast_cmd_sync_t *s = (const fast_cmd_sync_t *)pkt;
            clock_sync.device_base_us = time_us_64();
            clock_sync.pc_base_us = s->timestamp;
            clock_sync.seq_num = s->seq_num;
            clock_sync.synced = true;
            
            uint8_t resp[8] = {FAST_CMD_RESPONSE, 0x01, 0, 0, 0, 0, 0, 0};
            uint32_t ts = (uint32_t)(clock_sync.device_base_us & 0xFFFFFFFF);
            memcpy(resp + 4, &ts, 4);
            neopixel_trigger_activity_flash_color(0x0000FF00);
            uart_send_bytes(resp, 8);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_PING: {
            uint8_t resp[8] = {FAST_CMD_RESPONSE, 0x00, 0, 0, 0, 0, 0, 0};
            uint32_t ts = (uint32_t)(time_us_64() & 0xFFFFFFFF);
            memcpy(resp + 4, &ts, 4);
            neopixel_trigger_activity_flash_color(0x0000FF00);
            uart_send_bytes(resp, 8);
            return true;
        }
        
        case FAST_CMD_INFO: {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            int16_t temp = get_cached_temperature(now_ms);
            uint8_t resp[8] = {
                FAST_CMD_INFO,
                (uint8_t)smooth_get_humanization_mode(),
                (uint8_t)smooth_get_inject_mode(),
                (uint8_t)smooth_get_max_per_frame(),
                (uint8_t)smooth_get_velocity_matching(),
                (uint8_t)(temp & 0xFF),
                (uint8_t)((temp >> 8) & 0xFF),
                0
            };
            neopixel_trigger_activity_flash_color(0x0000FF00);
            uart_send_bytes(resp, 8);
            fast_cmd_count++;
            return true;
        }
        
        default:
            fast_cmd_errors++;
            return false;
    }
}

//--------------------------------------------------------------------+
// Fast Integer Parser (replaces sscanf for performance)
//--------------------------------------------------------------------+

// Fast signed integer parser - returns pointer to char after number, or NULL on error
static const char* __not_in_flash_func(parse_int16)(const char *p, int16_t *out) {
    int32_t val = 0;
    bool neg = false;
    
    if (*p == '-') { neg = true; p++; }
    else if (*p == '+') { p++; }
    
    if (*p < '0' || *p > '9') return NULL;
    
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        p++;
    }
    
    *out = (int16_t)(neg ? -val : val);
    return p;
}

//--------------------------------------------------------------------+
// Text Protocol Handling
//--------------------------------------------------------------------+

static bool handle_text_command(const char *line, size_t len, uint32_t now_ms) {
    (void)now_ms;
    
    // Mouse move: M<x>,<y>
    if (len >= 3 && line[0] == 'M') {
        int16_t x, y;
        const char *p = parse_int16(line + 1, &x);
        if (p && *p == ',') {
            p = parse_int16(p + 1, &y);
            if (p) {
                neopixel_trigger_activity_flash_color(0x0000FF00);
                kmbox_add_mouse_movement(x, y);
                return true;
            }
        }
    }
    
    // Wheel: W<delta>
    if (len >= 2 && line[0] == 'W') {
        int16_t delta;
        const char *p = parse_int16(line + 1, &delta);
        if (p) {
            kmbox_add_wheel_movement((int8_t)delta);
            return true;
        }
    }
    
    // Buttons: B<mask>
    if (len >= 2 && line[0] == 'B') {
        int16_t mask;
        const char *p = parse_int16(line + 1, &mask);
        if (p) {
            kmbox_update_physical_buttons((uint8_t)mask);
            return true;
        }
    }
    
    // Ping: P
    if (len >= 1 && line[0] == 'P') {
        kmbox_send_response("KMBOX_PONG");
        return true;
    }
    
    // Echo: E<data>
    if (len >= 2 && line[0] == 'E') {
        neopixel_trigger_activity_flash_color(0x00FFFF00);
        uart_send_string("ECHO:");
        uart_send_string(line + 1);
        uart_send_string("\r\n");
        uart_tx_wait_blocking(KMBOX_UART);
        return true;
    }
    
    // Protocol: KMBOX_BRIDGE_SYNC
    if (len >= 17 && strncmp(line, "KMBOX_BRIDGE_SYNC", 17) == 0) {
        kmbox_send_response("KMBOX_READY");
        set_connection_state(BRIDGE_STATE_CONNECTED);
        return true;
    }
    
    // Protocol: KMBOX_PING
    if (len >= 10 && strncmp(line, "KMBOX_PING", 10) == 0) {
        char resp[64];
        snprintf(resp, sizeof(resp), "KMBOX_PONG:v%s", KMBOX_PROTOCOL_VERSION);
        kmbox_send_response(resp);
        if (g_connection_state == BRIDGE_STATE_WAITING || g_connection_state == BRIDGE_STATE_DISCONNECTED) {
            set_connection_state(BRIDGE_STATE_CONNECTING);
        }
        return true;
    }
    
    // Protocol: KMBOX_CONNECT
    if (len >= 13 && strncmp(line, "KMBOX_CONNECT", 13) == 0) {
        kmbox_send_response("KMBOX_READY");
        set_connection_state(BRIDGE_STATE_CONNECTED);
        return true;
    }
    
    // Protocol: KMBOX_DISCONNECT
    if (len >= 16 && strncmp(line, "KMBOX_DISCONNECT", 16) == 0) {
        kmbox_send_response("KMBOX_BYE");
        set_connection_state(BRIDGE_STATE_WAITING);
        return true;
    }
    
    // Protocol: KMBOX_STATUS
    if (len >= 12 && strncmp(line, "KMBOX_STATUS", 12) == 0) {
        const char *names[] = {"WAITING", "CONNECTING", "CONNECTED", "ACTIVE", "DISCONNECTED"};
        char resp[64];
        snprintf(resp, sizeof(resp), "KMBOX_STATE:%s", names[g_connection_state]);
        kmbox_send_response(resp);
        return true;
    }
    
    // Protocol: KMBOX_INFO
    if (len >= 10 && strncmp(line, "KMBOX_INFO", 10) == 0) {
        char resp[128];
        snprintf(resp, sizeof(resp), "KMBOX_VID:%04X", get_attached_vid());
        kmbox_send_response(resp);
        snprintf(resp, sizeof(resp), "KMBOX_PID:%04X", get_attached_pid());
        kmbox_send_response(resp);
        snprintf(resp, sizeof(resp), "KMBOX_MFR:%s", get_attached_manufacturer()[0] ? get_attached_manufacturer() : "Unknown");
        kmbox_send_response(resp);
        snprintf(resp, sizeof(resp), "KMBOX_PROD:%s", get_attached_product()[0] ? get_attached_product() : "Unknown");
        kmbox_send_response(resp);
        return true;
    }
    
    return false;
}

// Parse a complete line from the ring buffer
static bool parse_text_line(char *buf, size_t bufsize, size_t *out_len) {
    uint16_t avail = rx_available();
    if (avail == 0) return false;
    
    // Scan for line terminator
    uint16_t line_end = 0;
    bool found = false;
    for (uint16_t i = 0; i < avail && i < bufsize - 1; i++) {
        uint8_t ch = rx_peek_at(i);
        if (ch == '\n' || ch == '\r') {
            line_end = i;
            found = true;
            break;
        }
    }
    if (!found) return false;
    
    // Copy line to buffer
    for (uint16_t i = 0; i < line_end; i++) {
        buf[i] = rx_peek_at(i);
    }
    buf[line_end] = '\0';
    *out_len = line_end;
    
    // Consume line + terminator(s)
    uint16_t consume = line_end + 1;
    if (consume < avail) {
        uint8_t next = rx_peek_at(consume);
        if (next == '\n' || next == '\r') consume++;  // Handle \r\n
    }
    rx_consume(consume);
    
    return true;
}

//--------------------------------------------------------------------+
// Initialization
//--------------------------------------------------------------------+

void kmbox_serial_init(void) {
    // Fix clk_peri for stable UART at 240MHz overclock
    if (clock_get_hz(clk_sys) > 133000000) {
        clock_configure(clk_peri, 0,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        48000000, 48000000);
    }
    
    // Initialize state
    uart_rx_head = 0;
    uart_rx_tail = 0;
    
    // Initialize TX ring buffer
    tx_buffer_init(&g_uart_tx_buffer);
    g_uart_tx_dropped = 0;
    
    g_connection_state = BRIDGE_STATE_WAITING;
    g_last_data_time_ms = to_ms_since_boot(get_absolute_time());
    
    // Configure GPIO pins
    // Use GPIO_FUNC_UART for default UART pins (GPIO0/GPIO1 for UART0)
    gpio_set_function(KMBOX_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(KMBOX_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(KMBOX_UART_RX_PIN);
    
    // Initialize UART
    uart_init(KMBOX_UART, KMBOX_UART_BAUDRATE);
    uart_set_format(KMBOX_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(KMBOX_UART, true);
    
    // Disable hardware flow control (no CTS/RTS)
    uart_set_hw_flow(KMBOX_UART, false, false);
    
    // Set up combined RX/TX interrupt handler
    int uart_irq = (KMBOX_UART == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, on_uart_irq);
    irq_set_priority(uart_irq, 0);  // Highest priority
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(KMBOX_UART, true, false);  // RX enabled, TX starts disabled
    
    // Initialize kmbox commands library
    kmbox_commands_init();
    kmbox_update_states(g_last_data_time_ms);
    
    // Set initial LED
    neopixel_set_status_override(STATUS_BRIDGE_WAITING);
    
    // Send startup message
    sleep_ms(50);
    kmbox_send_response("KMBOX_READY");
}

void kmbox_serial_init_dma(void) {
    // DMA disabled - using IRQ-based RX for reliability
}

//--------------------------------------------------------------------+
// Main Task
//--------------------------------------------------------------------+

void kmbox_serial_task(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    
    // Periodic ping to bridge (every 5 seconds)
    if (now_ms - g_last_ping_time_ms > 5000) {
        g_last_ping_time_ms = now_ms;
        kmbox_send_ping_to_bridge();
    }
    
    // Check connection timeout
    check_connection_timeout(now_ms);
    
    // Process pending timed moves
    process_pending_timed_move();
    
    // Process RX buffer
    while (rx_available() > 0) {
        uint8_t first = rx_peek_at(0);
        
        //--------------------------------------------------------------
        // Bridge protocol (sync byte 0xBD)
        //--------------------------------------------------------------
        if (first == BRIDGE_SYNC_BYTE) {
            uint16_t avail = rx_available();
            if (avail < 2) break;  // Need more data
            
            uint8_t pkt[8];
            size_t copy = (avail < 8) ? avail : 8;
            // Optimized peek - use memcpy if data is contiguous
            uint16_t tail = uart_rx_tail;
            uint16_t first_chunk = UART_RX_BUFFER_SIZE - tail;
            if (first_chunk >= copy) {
                // Data is contiguous, use fast memcpy
                memcpy(pkt, &uart_rx_buffer[tail], copy);
            } else {
                // Data wraps around, copy in two chunks
                memcpy(pkt, &uart_rx_buffer[tail], first_chunk);
                memcpy(pkt + first_chunk, uart_rx_buffer, copy - first_chunk);
            }
            
            uint8_t consumed = process_bridge_packet(pkt, copy);
            if (consumed >= 2) {
                rx_consume(consumed);
                mark_activity(now_ms);
                continue;
            } else if (consumed == 1) {
                rx_consume(1);  // Skip bad sync
                continue;
            }
            break;  // Incomplete packet
        }
        
        //--------------------------------------------------------------
        // Fast binary commands (8-byte fixed packets)
        //--------------------------------------------------------------
        if (is_fast_cmd_start(first)) {
            if (rx_available() >= 8) {
                uint8_t pkt[8];
                for (int i = 0; i < 8; i++) pkt[i] = rx_peek_at(i);
                rx_consume(8);
                process_fast_command(pkt);
                mark_activity(now_ms);
                continue;
            }
            break;  // Wait for complete packet
        }
        
        //--------------------------------------------------------------
        // Skip stray binary markers
        //--------------------------------------------------------------
        if (first == 0xFE || first == 0xFF || first == 0x00) {
            rx_consume(1);
            continue;
        }
        
        // Not binary - fall through to text processing
        break;
    }
    
    //------------------------------------------------------------------
    // Text command processing
    //------------------------------------------------------------------
    char line[KMBOX_CMD_BUFFER_SIZE];
    size_t len;
    while (parse_text_line(line, sizeof(line), &len)) {
        if (len > 0) {
            mark_activity(now_ms);
            if (!handle_text_command(line, len, now_ms)) {
                // Pass to kmbox library for additional parsing
                kmbox_process_serial_line(line, len, "\n", 1, now_ms);
            }
        }
    }
    
    // Update button states
    kmbox_update_states(now_ms);
}

//--------------------------------------------------------------------+
// Mouse Report
//--------------------------------------------------------------------+

bool kmbox_send_mouse_report(void) {
    if (!tud_hid_ready()) return false;
    
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    kmbox_update_states(now_ms);
    
    uint8_t buttons;
    int8_t x, y, wheel, pan;
    kmbox_get_mouse_report(&buttons, &x, &y, &wheel, &pan);
    
    return tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, x, y, wheel, pan);
}

//--------------------------------------------------------------------+
// Statistics & Compat Stubs
//--------------------------------------------------------------------+

void fast_cmd_get_stats(uint32_t *count, uint32_t *errors, uint32_t *overflows) {
    if (count) *count = fast_cmd_count;
    if (errors) *errors = fast_cmd_errors;
    if (overflows) *overflows = 0;
}

void kmbox_process_bridge_injections(void) {
    // No-op: movements go through shared accumulator
}

//--------------------------------------------------------------------+
// Send Info Packet to Bridge
//--------------------------------------------------------------------+

void kmbox_send_info_to_bridge(void) {
    int16_t temp = read_temperature_decidegrees();
    uint8_t info_pkt[8] = {
        FAST_CMD_INFO,
        (uint8_t)smooth_get_humanization_mode(),
        (uint8_t)smooth_get_inject_mode(),
        (uint8_t)smooth_get_max_per_frame(),
        (uint8_t)smooth_get_velocity_matching(),
        (uint8_t)(temp & 0xFF),
        (uint8_t)((temp >> 8) & 0xFF),
        0
    };
    uart_send_bytes(info_pkt, 8);
}
