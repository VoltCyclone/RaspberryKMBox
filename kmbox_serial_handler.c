/*
 * KMBox Serial Command Handler
 * 
 * Handles UART communication with RP2350 bridge for mouse/keyboard injection.
 * Supports text protocol (M<x>,<y>) and 8-byte binary protocol for low latency.
 */

#include "kmbox_serial_handler.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "wire_protocol.h"
#include "dma_uart.h"
#include "usb_hid.h"
#include "led_control.h"
#include "smooth_injection.h"
#include "uart_buffers.h"
#include "dcp_helpers.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

//--------------------------------------------------------------------+
// Configuration (UART_RX_BUFFER_SIZE defined in uart_buffers.h)
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Temperature Sensor (with caching for performance)
//--------------------------------------------------------------------+

static int16_t g_cached_temperature = 250;  // 25.0°C default
static uint32_t g_last_temp_read_ms = 0;
#define TEMP_CACHE_MS 1000

static inline int16_t read_temperature_decidegrees(void) {
    adc_select_input(4);
    uint16_t raw = adc_read();
    // Use DCP hardware acceleration for temperature conversion
    float temp_c = dcp_adc_to_temp(raw);
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

static __force_inline int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}

//--------------------------------------------------------------------+
// RX DMA Circular Buffer (zero-CPU reception)
// DMA writes continuously, main loop reads via write pointer comparison
//--------------------------------------------------------------------+

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE] __attribute__((aligned(UART_RX_BUFFER_SIZE)));
static volatile uint32_t uart_rx_read_pos = 0;
static int uart_rx_dma_chan = -1;

//--------------------------------------------------------------------+
// TX DMA (zero-CPU transmission)
// Stages data into a linear buffer, then fires a DMA transfer.
// Much faster than per-byte IRQ TX at 2 Mbaud.
//--------------------------------------------------------------------+
#define DMA_TX_BUFFER_SIZE  1024
static uint8_t dma_tx_buffer[DMA_TX_BUFFER_SIZE] __attribute__((aligned(4)));
static volatile uint16_t dma_tx_len = 0;
static int uart_tx_dma_chan = -1;
static volatile bool dma_tx_busy = false;

// Thin wrappers over shared dma_uart.h helpers, bound to our local buffer
static __force_inline uint32_t rx_get_write_pos(void) {
    return dma_ring_write_pos(uart_rx_dma_chan, uart_rx_buffer, UART_RX_BUFFER_MASK);
}

static __force_inline uint16_t rx_available(void) {
    return (uint16_t)dma_ring_available(uart_rx_read_pos, rx_get_write_pos(), UART_RX_BUFFER_MASK);
}

static __force_inline uint8_t rx_peek_at(uint16_t offset) {
    return dma_ring_peek(uart_rx_buffer, uart_rx_read_pos, offset, UART_RX_BUFFER_MASK);
}

static __force_inline void rx_consume(uint16_t count) {
    uart_rx_read_pos = (uart_rx_read_pos + count) & UART_RX_BUFFER_MASK;
}

//--------------------------------------------------------------------+
// TX Ring Buffer (Non-blocking IRQ-based transmission)
//--------------------------------------------------------------------+

static uart_tx_buffer_t g_uart_tx_buffer;
static volatile uint32_t g_uart_tx_dropped = 0;

// Forward declaration for DMA TX completion handler
static void __not_in_flash_func(on_dma_tx_complete)(void);

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
// Timed Move (hardware alarm — microsecond-accurate, zero polling)
//--------------------------------------------------------------------+

// Volatile struct for alarm callback to inject movement at exact time
static volatile struct {
    int16_t x, y;
    uint8_t mode;
    volatile bool pending;  // Set by alarm callback, cleared by main loop check
    volatile bool fired;    // Alarm fired, needs injection in main context
} pending_timed_move = {0};

static alarm_id_t g_timed_move_alarm_id = -1;

// Hardware alarm callback — fires at exact microsecond target time.
// Runs in IRQ context, so we just flag it for main loop injection.
// Returns 0 = don't reschedule.
static int64_t timed_move_alarm_callback(alarm_id_t id, void *user_data) {
    (void)id;
    (void)user_data;
    pending_timed_move.fired = true;
    g_timed_move_alarm_id = -1;
    return 0;  // Don't reschedule
}

// Process fired timed move in main loop context (safe for smooth_inject_movement)
static inline void process_pending_timed_move(void) {
    if (pending_timed_move.fired) {
        pending_timed_move.fired = false;
        pending_timed_move.pending = false;
        inject_mode_t mode = (pending_timed_move.mode <= 3) 
            ? (inject_mode_t)pending_timed_move.mode : INJECT_MODE_SMOOTH;
        smooth_inject_movement(pending_timed_move.x, pending_timed_move.y, mode);
    }
}

//--------------------------------------------------------------------+
// Fast Binary Command Stats
//--------------------------------------------------------------------+

static uint32_t fast_cmd_count = 0;
static uint32_t fast_cmd_errors = 0;

//--------------------------------------------------------------------+
// DMA TX Completion IRQ Handler
//--------------------------------------------------------------------+

static void __not_in_flash_func(on_dma_tx_complete)(void) {
    // Clear the interrupt
    if (uart_tx_dma_chan >= 0) {
        dma_hw->ints0 = (1u << uart_tx_dma_chan);
    }
    dma_tx_busy = false;
}

//--------------------------------------------------------------------+
// UART TX IRQ Handler (fallback before DMA TX is initialized)
//--------------------------------------------------------------------+

static void __not_in_flash_func(on_uart_irq)(void) {
    // Handle TX (only used before DMA TX is initialized)
    if (uart_is_writable(KMBOX_UART) && g_uart_tx_buffer.tx_active) {
        while (uart_is_writable(KMBOX_UART) && tx_buffer_has_data(&g_uart_tx_buffer)) {
            int byte = tx_buffer_get(&g_uart_tx_buffer);
            if (byte >= 0) {
                uart_putc_raw(KMBOX_UART, (uint8_t)byte);
            } else {
                break;
            }
        }
        if (!tx_buffer_has_data(&g_uart_tx_buffer)) {
            uart_set_irq_enables(KMBOX_UART, false, false);
            g_uart_tx_buffer.tx_active = false;
        }
    }
}

// Non-blocking TX: Use DMA if available, fall back to IRQ ring buffer
static bool uart_send_bytes(const uint8_t *data, size_t len) {
    if (!data || len == 0) return true;
    
    // DMA TX path: zero-CPU transmission
    if (uart_tx_dma_chan >= 0) {
        // Wait for previous DMA transfer to complete (should be very fast at 3Mbaud)
        // Reduced timeout to 500µs to prevent starving tud_task() under burst traffic
        // At 3Mbaud, 256 bytes takes ~0.85ms max; most packets are 8 bytes (~27µs)
        uint32_t timeout = time_us_32() + 500;  // 500µs safety timeout
        while (dma_tx_busy && time_us_32() < timeout) {
            tight_loop_contents();
        }
        if (dma_tx_busy) {
            // DMA still busy after timeout — drop the data to avoid blocking
            g_uart_tx_dropped += len;
            return false;
        }
        
        // Copy data to DMA TX staging buffer
        size_t to_send = (len > DMA_TX_BUFFER_SIZE) ? DMA_TX_BUFFER_SIZE : len;
        memcpy(dma_tx_buffer, data, to_send);
        dma_tx_len = to_send;
        dma_tx_busy = true;
        
        // Fire DMA transfer
        dma_channel_set_read_addr(uart_tx_dma_chan, dma_tx_buffer, false);
        dma_channel_set_trans_count(uart_tx_dma_chan, to_send, true);
        
        if (to_send < len) {
            g_uart_tx_dropped += (len - to_send);
            return false;
        }
        return true;
    }
    
    // Fallback: IRQ-based ring buffer TX (before DMA is initialized)
    int uart_irq = (KMBOX_UART == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(uart_irq, false);
    
    uint16_t written = tx_buffer_write(&g_uart_tx_buffer, data, len);
    
    if (written > 0 && !g_uart_tx_buffer.tx_active) {
        g_uart_tx_buffer.tx_active = true;
        uart_set_irq_enables(KMBOX_UART, false, true);
    }
    
    irq_set_enabled(uart_irq, true);
    
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
    uart_send_string(response);
    uart_send_string("\r\n");
    // Non-blocking: TX IRQ handles actual transmission
}

void kmbox_send_status(const char *message) {
    if (message) kmbox_send_response(message);
}

void kmbox_send_ping_to_bridge(void) {
    uint8_t resp[6];
    wire_build_response(resp, WIRE_RESP_PONG, 0, 0, 0, 0);
    uart_send_bytes(resp, 6);
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

static __force_inline void mark_activity(uint32_t now_ms) {
    g_last_data_time_ms = now_ms;
    if (__builtin_expect(!kmbox_is_connected(), 0)) set_connection_state(BRIDGE_STATE_CONNECTED);
    if (__builtin_expect(g_connection_state == BRIDGE_STATE_CONNECTED, 0)) set_connection_state(BRIDGE_STATE_ACTIVE);
}

//--------------------------------------------------------------------+
// Bridge Protocol Parser (variable-length binary, sync byte 0xBD)
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Wire Protocol v2 Processing (variable-length, LUT-based)
//--------------------------------------------------------------------+

extern void process_mouse_report(const hid_mouse_report_t *report);
extern void process_kbd_report(const hid_keyboard_report_t *report);

static bool __not_in_flash_func(process_wire_command)(const uint8_t *pkt, uint8_t len) {
    (void)len;
    switch (pkt[0]) {
        case WIRE_MOVE8: {
            int8_t x = (int8_t)pkt[1];
            int8_t y = (int8_t)pkt[2];
            kmbox_add_mouse_movement(x, y);
            neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
            fast_cmd_count++;
            return true;
        }
        case WIRE_MOVE16: {
            int16_t x = read_i16_le(pkt + 1);
            int16_t y = read_i16_le(pkt + 3);
            kmbox_add_mouse_movement(x, y);
            neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
            fast_cmd_count++;
            return true;
        }
        case WIRE_BUTTONS: {
            kmbox_update_physical_buttons(pkt[1]);
            fast_cmd_count++;
            return true;
        }
        case WIRE_WHEEL: {
            kmbox_add_wheel_movement((int8_t)pkt[1]);
            fast_cmd_count++;
            return true;
        }
        case WIRE_MOVE_ALL: {
            int16_t x = read_i16_le(pkt + 1);
            int16_t y = read_i16_le(pkt + 3);
            if (x || y) kmbox_add_mouse_movement(x, y);
            if (pkt[5]) kmbox_update_physical_buttons(pkt[5]);
            if (pkt[6]) kmbox_add_wheel_movement((int8_t)pkt[6]);
            neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
            fast_cmd_count++;
            return true;
        }
        case WIRE_MOVE8_BTN: {
            int8_t x = (int8_t)pkt[1];
            int8_t y = (int8_t)pkt[2];
            if (x || y) kmbox_add_mouse_movement(x, y);
            if (pkt[3]) kmbox_update_physical_buttons(pkt[3]);
            if (pkt[4]) kmbox_add_wheel_movement((int8_t)pkt[4]);
            neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
            fast_cmd_count++;
            return true;
        }
        case WIRE_SMOOTH16: {
            int16_t x = read_i16_le(pkt + 1);
            int16_t y = read_i16_le(pkt + 3);
            inject_mode_t mode = (pkt[5] <= 3) ? (inject_mode_t)pkt[5] : INJECT_MODE_SMOOTH;
            smooth_inject_movement(x, y, mode);
            fast_cmd_count++;
            return true;
        }
        case WIRE_KEYDOWN: {
            hid_keyboard_report_t report = {.modifier = pkt[2], .keycode = {pkt[1]}};
            process_kbd_report(&report);
            fast_cmd_count++;
            return true;
        }
        case WIRE_KEYUP: {
            hid_keyboard_report_t report = {0};
            process_kbd_report(&report);
            fast_cmd_count++;
            return true;
        }
        case WIRE_CLICK: {
            // Map button number to kmbox_button_t enum
            kmbox_button_t btn;
            switch (pkt[1]) {
                case 0: case 1: btn = KMBOX_BUTTON_LEFT;   break;
                case 2:         btn = KMBOX_BUTTON_RIGHT;  break;
                case 3:         btn = KMBOX_BUTTON_MIDDLE; break;
                case 4:         btn = KMBOX_BUTTON_SIDE1;  break;
                case 5:         btn = KMBOX_BUTTON_SIDE2;  break;
                default:        btn = KMBOX_BUTTON_LEFT;   break;
            }
            uint32_t click_now = time_us_32() / 1000;
            kmbox_start_button_click(btn, click_now);
            fast_cmd_count++;
            return true;
        }
        case WIRE_SMOOTH_CFG: {
            if (pkt[1] > 0) smooth_set_max_per_frame(pkt[1]);
            smooth_set_velocity_matching(pkt[2] != 0);
            fast_cmd_count++;
            return true;
        }
        case WIRE_SMOOTH_CLR:
            smooth_clear_queue();
            fast_cmd_count++;
            return true;
        case WIRE_INFO_REQ: {
            kmbox_send_info_to_bridge();
            // Also send binary info packets for TFT display (temp, flags, extended stats)
            {
                uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                int16_t temp = get_cached_temperature(now_ms);
                uint8_t queue_count = 0;
                smooth_get_stats(NULL, NULL, NULL, &queue_count);
                uint8_t flags = 0;
                humanization_mode_t hm = smooth_get_humanization_mode();
                if (hm >= HUMANIZATION_MICRO) flags |= 0x01;
                if (smooth_get_velocity_matching()) flags |= 0x02;
                uint8_t q3 = (queue_count > 28) ? 7 : (queue_count / 4);
                flags |= (q3 & 0x07) << 2;
                uint8_t info[8] = {
                    WIRE_BIN_INFO,
                    (uint8_t)hm,
                    (uint8_t)smooth_get_inject_mode(),
                    (uint8_t)smooth_get_max_per_frame(),
                    queue_count,
                    (uint8_t)(temp & 0xFF),
                    (uint8_t)((temp >> 8) & 0xFF),
                    flags
                };
                uart_send_bytes(info, 8);
            }
            {
                uint32_t total_injected = 0, frames_processed = 0, queue_overflows = 0;
                uint8_t queue_count = 0;
                smooth_get_stats(&total_injected, &frames_processed, &queue_overflows, &queue_count);
                uint8_t ext[8] = {
                    WIRE_BIN_EXT,
                    queue_count,
                    SMOOTH_QUEUE_SIZE,
                    (uint8_t)smooth_get_humanization_mode(),
                    (uint8_t)(total_injected & 0xFF),
                    (uint8_t)((total_injected >> 8) & 0xFF),
                    (uint8_t)(queue_overflows & 0xFF),
                    (uint8_t)((queue_overflows >> 8) & 0xFF)
                };
                uart_send_bytes(ext, 8);
            }
            fast_cmd_count++;
            return true;
        }
        case WIRE_CYCLE_HUM: {
            humanization_mode_t new_mode = smooth_cycle_humanization_mode();
            uint32_t mode_color;
            switch (new_mode) {
                case HUMANIZATION_OFF:   mode_color = COLOR_HUMANIZATION_OFF;   break;
                case HUMANIZATION_MICRO: mode_color = COLOR_HUMANIZATION_MICRO; break;
                case HUMANIZATION_FULL:  mode_color = COLOR_HUMANIZATION_FULL;  break;
                default:                 mode_color = COLOR_ERROR;              break;
            }
            neopixel_set_color(mode_color);
            neopixel_trigger_mode_flash(mode_color, 1500);
            kmbox_send_info_to_bridge();
            fast_cmd_count++;
            return true;
        }
        case WIRE_CONFIG: {
            uint8_t param = pkt[1];
            uint8_t value = pkt[2];
            if (param == WIRE_CFG_HUMANIZATION && value < HUMANIZATION_MODE_COUNT) {
                smooth_set_humanization_mode((humanization_mode_t)value);
                uint32_t mode_color;
                switch ((humanization_mode_t)value) {
                    case HUMANIZATION_OFF:   mode_color = COLOR_HUMANIZATION_OFF;   break;
                    case HUMANIZATION_MICRO: mode_color = COLOR_HUMANIZATION_MICRO; break;
                    case HUMANIZATION_FULL:  mode_color = COLOR_HUMANIZATION_FULL;  break;
                    default:                 mode_color = COLOR_ERROR;              break;
                }
                neopixel_set_color(mode_color);
                neopixel_trigger_mode_flash(mode_color, 1500);
                kmbox_send_info_to_bridge();
            }
            fast_cmd_count++;
            return true;
        }
        case WIRE_PING: {
            uint8_t resp[6];
            uint32_t ts = (uint32_t)(time_us_64() & 0xFFFFFFFF);
            wire_build_response(resp, WIRE_RESP_PONG,
                (uint8_t)(ts & 0xFF), (uint8_t)((ts >> 8) & 0xFF),
                (uint8_t)((ts >> 16) & 0xFF), (uint8_t)((ts >> 24) & 0xFF));
            uart_send_bytes(resp, 6);
            fast_cmd_count++;
            return true;
        }
        default:
            fast_cmd_errors++;
            return false;
    }
}

//--------------------------------------------------------------------+
// Fast Binary Command Processing (8-byte fixed packets, legacy)
//--------------------------------------------------------------------+

static __force_inline bool is_fast_cmd_start(uint8_t byte) {
    return byte == FAST_CMD_TIMED_MOVE;  // 0x0A — only remaining fast command
}

static bool __not_in_flash_func(process_fast_command)(const uint8_t *pkt) {
    // Only FAST_CMD_TIMED_MOVE (0x0A) reaches here — all other command bytes
    // are intercepted by the wire protocol LUT before this function is called.
    if (pkt[0] != FAST_CMD_TIMED_MOVE) {
        fast_cmd_errors++;
        return false;
    }

    const fast_cmd_timed_t *t = (const fast_cmd_timed_t *)pkt;
    if (clock_sync.synced && t->time_us > 0) {
        uint64_t target = clock_sync.device_base_us + t->time_us;
        uint64_t now = time_us_64();
        if (target > now && (target - now) < 100000) {
            if (g_timed_move_alarm_id >= 0) {
                cancel_alarm(g_timed_move_alarm_id);
                g_timed_move_alarm_id = -1;
            }
            pending_timed_move.x = t->x;
            pending_timed_move.y = t->y;
            pending_timed_move.mode = t->mode;
            pending_timed_move.pending = true;
            pending_timed_move.fired = false;
            uint64_t delay_us = target - now;
            g_timed_move_alarm_id = add_alarm_in_us(delay_us, timed_move_alarm_callback, NULL, true);
            fast_cmd_count++;
            return true;
        }
    }
    inject_mode_t mode = (t->mode <= 3) ? (inject_mode_t)t->mode : INJECT_MODE_SMOOTH;
    smooth_inject_movement(t->x, t->y, mode);
    fast_cmd_count++;
    return true;
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
        // Bail out early if value exceeds int16_t range (prevents int32_t overflow
        // on pathologically long digit strings)
        if (val > 32767) {
            val = 32767;
            // Skip remaining digits
            while (*(++p) >= '0' && *p <= '9') {}
            break;
        }
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
        uart_send_string("ECHO:");
        uart_send_string(line + 1);
        uart_send_string("\r\n");
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
        kmbox_send_response("KMBOX_PONG:v" KMBOX_PROTOCOL_VERSION);
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
        static const char *responses[] = {
            "KMBOX_STATE:WAITING",
            "KMBOX_STATE:CONNECTING",
            "KMBOX_STATE:CONNECTED",
            "KMBOX_STATE:ACTIVE",
            "KMBOX_STATE:DISCONNECTED"
        };
        if (g_connection_state < 5) {
            kmbox_send_response(responses[g_connection_state]);
        }
        return true;
    }
    
    // Protocol: KMBOX_INFO
    // Note: This is a diagnostic command, not hot path, but we still optimize it
    if (len >= 10 && strncmp(line, "KMBOX_INFO", 10) == 0) {
        char resp[128];
        uint16_t vid = get_attached_vid();
        uint16_t pid = get_attached_pid();
        const char *mfr = get_attached_manufacturer();
        const char *prod = get_attached_product();
        
        // Format hex values manually (faster than snprintf for simple hex)
        static const char hex[] = "0123456789ABCDEF";
        resp[0] = 'K'; resp[1] = 'M'; resp[2] = 'B'; resp[3] = 'O'; resp[4] = 'X'; resp[5] = '_';
        resp[6] = 'V'; resp[7] = 'I'; resp[8] = 'D'; resp[9] = ':';
        resp[10] = hex[(vid >> 12) & 0xF];
        resp[11] = hex[(vid >> 8) & 0xF];
        resp[12] = hex[(vid >> 4) & 0xF];
        resp[13] = hex[vid & 0xF];
        resp[14] = '\0';
        kmbox_send_response(resp);
        
        resp[6] = 'P'; resp[7] = 'I'; resp[8] = 'D';
        resp[10] = hex[(pid >> 12) & 0xF];
        resp[11] = hex[(pid >> 8) & 0xF];
        resp[12] = hex[(pid >> 4) & 0xF];
        resp[13] = hex[pid & 0xF];
        kmbox_send_response(resp);
        
        // Manufacturer and product need snprintf for string concatenation
        snprintf(resp, sizeof(resp), "KMBOX_MFR:%s", mfr[0] ? mfr : "Unknown");
        kmbox_send_response(resp);
        snprintf(resp, sizeof(resp), "KMBOX_PROD:%s", prod[0] ? prod : "Unknown");
        kmbox_send_response(resp);
        
        // Humanization info - still needs snprintf for multiple integers
        uint32_t total_inj = 0, frames_proc = 0, q_overflows = 0;
        uint8_t q_count = 0;
        smooth_get_stats(&total_inj, &frames_proc, &q_overflows, &q_count);
        snprintf(resp, sizeof(resp), "KMBOX_INFO:hmode=%d,imode=%d,max=%d,vel=%d,qd=%d,qc=%d,inj=%lu,ovf=%lu",
                 (int)smooth_get_humanization_mode(),
                 (int)smooth_get_inject_mode(),
                 (int)smooth_get_max_per_frame(),
                 (int)smooth_get_velocity_matching(),
                 (int)q_count,
                 (int)SMOOTH_QUEUE_SIZE,
                 (unsigned long)total_inj,
                 (unsigned long)q_overflows);
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
    uart_rx_read_pos = 0;
    
    // Initialize TX ring buffer
    tx_buffer_init(&g_uart_tx_buffer);
    g_uart_tx_dropped = 0;
    
    g_connection_state = BRIDGE_STATE_WAITING;
    g_last_data_time_ms = to_ms_since_boot(get_absolute_time());
    
    // Configure GPIO pins
    gpio_set_function(KMBOX_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(KMBOX_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(KMBOX_UART_RX_PIN);
    
    // Initialize UART
    uart_init(KMBOX_UART, KMBOX_UART_BAUDRATE);
    uart_set_format(KMBOX_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(KMBOX_UART, true);
    
    // Disable hardware flow control (no CTS/RTS)
    uart_set_hw_flow(KMBOX_UART, false, false);
    
    // Disable UART RX interrupts - we use DMA for RX
    // TX IRQ handler still needed for non-blocking TX
    uart_set_irq_enables(KMBOX_UART, false, false);
    
    // Set up TX-only IRQ handler
    int uart_irq = (KMBOX_UART == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, on_uart_irq);
    irq_set_priority(uart_irq, 0);  // Highest priority for TX
    irq_set_enabled(uart_irq, true);
    
    // Drain any garbage from UART FIFO
    while (uart_is_readable(KMBOX_UART)) {
        uart_getc(KMBOX_UART);
    }
    
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
    // Set up DMA circular buffer for UART RX (zero-CPU reception)
    uart_rx_dma_chan = dma_claim_unused_channel(false);
    if (uart_rx_dma_chan < 0) {
        printf("[KMBOX] WARNING: Failed to claim RX DMA channel, falling back to polling\n");
        return;
    }
    
    // Configure RX DMA with circular (ring) buffer
    dma_channel_config rx_cfg = dma_channel_get_default_config(uart_rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);   // Always read from UART DR
    channel_config_set_write_increment(&rx_cfg, true);    // Increment write pointer
    channel_config_set_dreq(&rx_cfg, uart_get_dreq(KMBOX_UART, false));  // RX DREQ
    channel_config_set_ring(&rx_cfg, true, __builtin_ctz(UART_RX_BUFFER_SIZE));  // Ring on write side
    channel_config_set_high_priority(&rx_cfg, true);      // High priority for low latency
    
    // Start continuous RX DMA - transfers indefinitely into circular buffer
    dma_channel_configure(
        uart_rx_dma_chan,
        &rx_cfg,
        uart_rx_buffer,                          // Write to ring buffer
        &uart_get_hw(KMBOX_UART)->dr,           // Read from UART data register
        0xFFFFFFFF,                              // Transfer "forever"
        true                                     // Start immediately
    );
    
    printf("[KMBOX] DMA RX initialized: ch=%d, buf=%u bytes\n", uart_rx_dma_chan, UART_RX_BUFFER_SIZE);
    
    //--------------------------------------------------------------------+
    // DMA TX: Zero-CPU UART transmission
    //--------------------------------------------------------------------+
    uart_tx_dma_chan = dma_claim_unused_channel(false);
    if (uart_tx_dma_chan < 0) {
        printf("[KMBOX] WARNING: Failed to claim TX DMA channel, using IRQ fallback\n");
        return;
    }
    
    dma_channel_config tx_cfg = dma_channel_get_default_config(uart_tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_cfg, true);     // Increment through TX buffer
    channel_config_set_write_increment(&tx_cfg, false);    // Always write to UART DR
    channel_config_set_dreq(&tx_cfg, uart_get_dreq(KMBOX_UART, true));  // TX DREQ
    channel_config_set_high_priority(&tx_cfg, true);       // High priority for low latency
    
    // Configure channel but don't start yet (will be triggered per-transfer)
    dma_channel_configure(
        uart_tx_dma_chan,
        &tx_cfg,
        &uart_get_hw(KMBOX_UART)->dr,   // Write to UART data register
        dma_tx_buffer,                   // Read from TX staging buffer (updated per-transfer)
        0,                               // Transfer count set per-transfer
        false                            // Don't start yet
    );
    
    // Set up DMA TX completion interrupt
    dma_channel_set_irq0_enabled(uart_tx_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, on_dma_tx_complete);
    irq_set_priority(DMA_IRQ_0, 1);  // High priority (just below UART)
    irq_set_enabled(DMA_IRQ_0, true);
    
    // Disable UART TX IRQ since DMA handles TX now
    uart_set_irq_enables(KMBOX_UART, false, false);
    g_uart_tx_buffer.tx_active = false;
    
    printf("[KMBOX] DMA TX initialized: ch=%d, buf=%u bytes\n", uart_tx_dma_chan, DMA_TX_BUFFER_SIZE);
}

//--------------------------------------------------------------------+
// Main Task
//--------------------------------------------------------------------+

void kmbox_serial_task(void) {
    // Use cheap microsecond timer — avoid to_ms_since_boot(get_absolute_time())
    // which has significant overhead.  Convert to ms only when needed.
    uint32_t now_us = time_us_32();
    uint32_t now_ms = now_us / 1000;
    
    // Periodic ping to bridge (every 5 seconds)
    if (now_ms - g_last_ping_time_ms > 5000) {
        g_last_ping_time_ms = now_ms;
        kmbox_send_ping_to_bridge();
    }
    
    // Check connection timeout
    check_connection_timeout(now_ms);
    
    // Process pending timed moves
    process_pending_timed_move();
    
    // Process RX buffer — drain up to 64 packets per call.
    // The 0x01 fast move path is just two atomic adds (~10ns each),
    // so we can process aggressively without starving tud_task().
    // At 3 Mbaud with 8-byte packets, 64 packets = ~1.7ms of buffer.
    uint16_t packets_processed = 0;
    const uint16_t MAX_PACKETS_PER_CALL = 64;
    
    while (rx_available() > 0 && packets_processed < MAX_PACKETS_PER_CALL) {
        uint8_t first = rx_peek_at(0);

        //--------------------------------------------------------------
        // Wire protocol v2 (command byte determines length via LUT)
        //--------------------------------------------------------------
        {
            uint8_t wire_len = wire_get_packet_len(first);
            if (wire_len > 0) {
                if (rx_available() >= wire_len) {
                    uint8_t pkt[WIRE_MAX_PACKET];
                    uint16_t tail = (uint16_t)(uart_rx_read_pos & UART_RX_BUFFER_MASK);
                    if (tail + wire_len <= UART_RX_BUFFER_SIZE) {
                        memcpy(pkt, &uart_rx_buffer[tail], wire_len);
                    } else {
                        for (uint8_t i = 0; i < wire_len; i++) pkt[i] = rx_peek_at(i);
                    }
                    rx_consume(wire_len);
                    process_wire_command(pkt, wire_len);
                    mark_activity(now_ms);
                    packets_processed++;
                    continue;
                }
                break;  // Incomplete packet, wait for more data
            }
        }

        //--------------------------------------------------------------
        // Fast binary commands — zero-copy when data is contiguous
        //--------------------------------------------------------------
        if (is_fast_cmd_start(first)) {
            if (rx_available() >= 8) {
                uint16_t tail = (uint16_t)(uart_rx_read_pos & UART_RX_BUFFER_MASK);
                const uint8_t *pkt;
                uint8_t pkt_stack[8];
                
                // Zero-copy: if all 8 bytes are contiguous in the circular buffer,
                // pass a pointer directly into the DMA buffer (no memcpy)
                if (tail + 8 <= UART_RX_BUFFER_SIZE) {
                    pkt = &uart_rx_buffer[tail];
                } else {
                    // Wraps around — must copy to stack (rare at 512-byte buffer)
                    for (int i = 0; i < 8; i++) pkt_stack[i] = rx_peek_at(i);
                    pkt = pkt_stack;
                }
                
                rx_consume(8);
                process_fast_command(pkt);
                mark_activity(now_ms);
                packets_processed++;
                continue;
            }
            break;  // Wait for complete packet
        }
        
        //--------------------------------------------------------------
        // Skip null bytes
        //--------------------------------------------------------------
        if (first == 0x00) {
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
    uint8_t queue_count = 0;
    smooth_get_stats(NULL, NULL, NULL, &queue_count);
    humanization_mode_t hm = smooth_get_humanization_mode();

    // Wire protocol v2 response (6 bytes)
    uint8_t wire_resp[6];
    wire_build_response(wire_resp, WIRE_RESP_INFO,
        (uint8_t)hm,
        (uint8_t)smooth_get_inject_mode(),
        (uint8_t)smooth_get_max_per_frame(),
        queue_count);
    uart_send_bytes(wire_resp, 6);
}
