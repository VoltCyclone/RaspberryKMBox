/*
 * KMBox Serial Command Handler
 * Integrates kmbox-commands library with the PIOKMBox firmware
 * Uses CP2110 HID-to-UART bridge via Feather Click Shield
 * Implements handshake protocol for reliable PC connection
 * 
 * FAST BINARY PROTOCOL:
 * For ultra-low latency, supports 8-byte binary packets that can be
 * processed without string parsing. At 2 Mbps, an 8-byte packet
 * takes only ~40µs to transmit.
 *
 * MEMORY-ALIGNED PROCESSING:
 * All command structures are 4-byte aligned and exactly 8 bytes, enabling:
 * - Single-instruction struct access via type-punning
 * - Zero-copy command parsing (cast buffer directly to struct)
 * - Optimal cache line utilization
 *
 * CLOCK SYNCHRONIZATION:
 * Supports timestamped commands for deterministic timing:
 * - SYNC command establishes time base with PC
 * - TIMED_MOVE executes at precise microsecond offsets
 * - Round-trip time measurement for latency compensation
 */

#include "kmbox_serial_handler.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "usb_hid.h"
#include "led_control.h"
#include "smooth_injection.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <string.h>

//--------------------------------------------------------------------+
// Memory-Aligned Fast Command Ring Buffer
//--------------------------------------------------------------------+
// 4-byte aligned ring buffer for zero-copy command processing
// Buffer size is power of 2 for fast modulo via bitmask

#define FAST_RING_SIZE      16          // Must be power of 2
#define FAST_RING_MASK      (FAST_RING_SIZE - 1)

// Ring buffer of aligned packets (128 bytes total, cache-friendly)
static fast_cmd_union_t __attribute__((aligned(8))) fast_ring[FAST_RING_SIZE];
static volatile uint8_t fast_ring_head = 0;  // Write position
static volatile uint8_t fast_ring_tail = 0;  // Read position

// Temporary accumulation buffer for incoming bytes
static uint8_t __attribute__((aligned(4))) __attribute__((unused)) fast_accum[8];
static uint8_t __attribute__((unused)) fast_accum_idx = 0;

// Statistics for fast commands
static uint32_t fast_cmd_count = 0;
static uint32_t fast_cmd_errors = 0;
static uint32_t fast_ring_overflows = 0;

//--------------------------------------------------------------------+
// Async/Non-blocking State for Timed Operations
//--------------------------------------------------------------------+
// These replace blocking sleep calls to avoid stalling USB tasks

typedef struct {
    uint64_t target_time_us;        // When to execute
    int16_t x;                      // Movement X
    int16_t y;                      // Movement Y
    uint8_t mode;                   // Injection mode
    bool pending;                   // True if waiting to execute
} timed_move_t;

static timed_move_t pending_timed_move = {0};

// Check and process pending timed move (called from main loop)
static inline void process_pending_timed_move(void) {
    if (pending_timed_move.pending) {
        if (time_us_64() >= pending_timed_move.target_time_us) {
            inject_mode_t inject_mode = (pending_timed_move.mode <= 3) 
                ? (inject_mode_t)pending_timed_move.mode : INJECT_MODE_SMOOTH;
            smooth_inject_movement(pending_timed_move.x, pending_timed_move.y, inject_mode);
            pending_timed_move.pending = false;
        }
    }
}

//--------------------------------------------------------------------+
// Clock Synchronization State
//--------------------------------------------------------------------+
// Enables precise timing between PC and device

typedef struct {
    uint64_t device_base_us;    // Device timestamp at last sync
    uint32_t pc_base_us;        // PC timestamp at last sync
    int32_t  offset_us;         // Estimated PC-to-device offset
    uint32_t rtt_us;            // Round-trip time estimate
    uint8_t  seq_num;           // Last sync sequence number
    bool     synced;            // True if sync established
} clock_sync_t;

static clock_sync_t clock_sync = {0};

// Ping tracking for bidirectional testing
static uint32_t kmbox_ping_count = 0;
static uint32_t last_kmbox_ping_time = 0;

// Get synchronized timestamp (device time adjusted for PC offset)
static inline uint64_t get_synced_time_us(void) {
    return time_us_64();
}

//--------------------------------------------------------------------+
// Fast Binary Command Processing (zero-copy, type-punned)
//--------------------------------------------------------------------+

// Forward declaration for process functions from usb_hid
extern void process_mouse_report(const hid_mouse_report_t *report);
extern void process_kbd_report(const hid_keyboard_report_t *report);

// Process a command using type-punned aligned access
// Returns true if processed successfully
static bool __not_in_flash_func(process_fast_command_aligned)(const fast_cmd_union_t *cmd) {
    
    switch (cmd->raw.bytes[0]) {
        case FAST_CMD_MOUSE_MOVE: {
            // Direct struct access - no byte shuffling needed
            const fast_cmd_move_t *m = &cmd->move;
            
            hid_mouse_report_t report = {0};
            report.buttons = m->buttons;
            report.x = (m->x > 127) ? 127 : ((m->x < -127) ? -127 : (int8_t)m->x);
            report.y = (m->y > 127) ? 127 : ((m->y < -127) ? -127 : (int8_t)m->y);
            report.wheel = m->wheel;
            process_mouse_report(&report);
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_MOUSE_CLICK: {
            const fast_cmd_click_t *c = &cmd->click;
            uint8_t count = c->count ? c->count : 1;
            
            // Map button number to bit
            static const uint8_t btn_map[] = {
                FAST_BTN_LEFT, FAST_BTN_LEFT, FAST_BTN_RIGHT,
                FAST_BTN_MIDDLE, FAST_BTN_BACK, FAST_BTN_FORWARD
            };
            uint8_t btn_mask = (c->button < 6) ? btn_map[c->button] : FAST_BTN_LEFT;
            
            // Non-blocking: Queue press/release reports
            // The HID polling rate will naturally space them out
            // Unroll for common cases (1-3 clicks)
            hid_mouse_report_t report = {0};
            if (count >= 1) {
                report.buttons = btn_mask;
                process_mouse_report(&report);
                report.buttons = 0;
                process_mouse_report(&report);
            }
            if (count >= 2) {
                report.buttons = btn_mask;
                process_mouse_report(&report);
                report.buttons = 0;
                process_mouse_report(&report);
            }
            if (count >= 3) {
                report.buttons = btn_mask;
                process_mouse_report(&report);
                report.buttons = 0;
                process_mouse_report(&report);
            }
            // Handle remaining clicks (rare)
            for (uint8_t i = 3; i < count && i < 10; i++) {
                report.buttons = btn_mask;
                process_mouse_report(&report);
                report.buttons = 0;
                process_mouse_report(&report);
            }
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_KEY_PRESS: {
            const fast_cmd_key_t *k = &cmd->key;
            
            // Non-blocking: Send key press, release will be sent next frame
            // The HID polling rate naturally provides the key press duration
            hid_keyboard_report_t report = {0};
            report.modifier = k->modifiers;
            report.keycode[0] = k->keycode;
            process_kbd_report(&report);
            
            // Immediate release (queued for next HID frame)
            memset(&report, 0, sizeof(report));
            process_kbd_report(&report);
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_KEY_COMBO: {
            const fast_cmd_combo_t *c = &cmd->combo;
            
            // Non-blocking: Send combo press, release queued for next frame
            hid_keyboard_report_t report = {0};
            report.modifier = c->modifiers;
            
            // Unroll key adding (4 keys max, most common cases are 1-2 keys)
            uint8_t key_count = 0;
            if (c->keys[0] != 0) report.keycode[key_count++] = c->keys[0];
            if (c->keys[1] != 0) report.keycode[key_count++] = c->keys[1];
            if (c->keys[2] != 0) report.keycode[key_count++] = c->keys[2];
            if (c->keys[3] != 0) report.keycode[key_count++] = c->keys[3];
            
            process_kbd_report(&report);
            
            // Immediate release (queued for next HID frame)
            memset(&report, 0, sizeof(report));
            process_kbd_report(&report);
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_MULTI_MOVE: {
            const fast_cmd_multi_t *m = &cmd->multi;
            
            // Process 3 moves from single packet
            if (m->x1 != 0 || m->y1 != 0) {
                hid_mouse_report_t report = {.x = m->x1, .y = m->y1};
                process_mouse_report(&report);
            }
            if (m->x2 != 0 || m->y2 != 0) {
                hid_mouse_report_t report = {.x = m->x2, .y = m->y2};
                process_mouse_report(&report);
            }
            if (m->x3 != 0 || m->y3 != 0) {
                hid_mouse_report_t report = {.x = m->x3, .y = m->y3};
                process_mouse_report(&report);
            }
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_MOUSE_ABS: {
            // TODO: Absolute positioning requires HID descriptor change
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_MOVE: {
            const fast_cmd_smooth_t *s = &cmd->smooth;
            
            // Map mode byte to inject_mode_t
            inject_mode_t inject_mode;
            switch (s->mode) {
                case 0: inject_mode = INJECT_MODE_IMMEDIATE; break;
                case 1: inject_mode = INJECT_MODE_SMOOTH; break;
                case 2: inject_mode = INJECT_MODE_VELOCITY_MATCHED; break;
                case 3: inject_mode = INJECT_MODE_MICRO; break;
                default: inject_mode = INJECT_MODE_SMOOTH; break;
            }
            
            smooth_inject_movement(s->x, s->y, inject_mode);
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_CONFIG: {
            const fast_cmd_config_t *c = &cmd->config;
            
            if (c->max_per_frame > 0) {
                smooth_set_max_per_frame(c->max_per_frame);
            }
            smooth_set_velocity_matching(c->vel_match != 0);
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_CLEAR: {
            smooth_clear_queue();
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_TIMED_MOVE: {
            const fast_cmd_timed_t *t = &cmd->timed;
            
            // Non-blocking: Schedule for later execution instead of busy-wait
            if (clock_sync.synced && t->time_us > 0) {
                uint64_t target_time = clock_sync.device_base_us + t->time_us;
                uint64_t now = time_us_64();
                
                if (target_time > now && (target_time - now) < 100000) {
                    // Schedule for async execution (checked in main loop)
                    pending_timed_move.target_time_us = target_time;
                    pending_timed_move.x = t->x;
                    pending_timed_move.y = t->y;
                    pending_timed_move.mode = t->mode;
                    pending_timed_move.pending = true;
                    fast_cmd_count++;
                    return true;
                }
            }
            
            // No delay or time already passed - execute immediately
            inject_mode_t inject_mode = (t->mode <= 3) ? (inject_mode_t)t->mode : INJECT_MODE_SMOOTH;
            smooth_inject_movement(t->x, t->y, inject_mode);
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SYNC: {
            const fast_cmd_sync_t *s = &cmd->sync;
            
            // Record sync point
            clock_sync.device_base_us = time_us_64();
            clock_sync.pc_base_us = s->timestamp;
            clock_sync.seq_num = s->seq_num;
            clock_sync.synced = true;
            
            // Send response with device timestamp
            fast_cmd_response_t response = {
                .cmd = FAST_CMD_RESPONSE,
                .status = 0x01,  // Sync ACK
                .timestamp = (uint32_t)(clock_sync.device_base_us & 0xFFFFFFFF)
            };
            
            // Send as raw bytes (8 bytes)
            const uint8_t *resp_bytes = (const uint8_t *)&response;
            for (int i = 0; i < 8; i++) {
                uart_putc_raw(KMBOX_UART, resp_bytes[i]);
            }
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_PING: {
            // Fast ping response with timestamp
            fast_cmd_response_t response = {
                .cmd = FAST_CMD_RESPONSE,
                .status = 0x00,  // Ping ACK
                .timestamp = (uint32_t)(time_us_64() & 0xFFFFFFFF)
            };
            
            const uint8_t *resp_bytes = (const uint8_t *)&response;
            for (int i = 0; i < 8; i++) {
                uart_putc_raw(KMBOX_UART, resp_bytes[i]);
            }
            
            fast_cmd_count++;
            return true;
        }
        
        default:
            fast_cmd_errors++;
            return false;
    }
}

// Legacy wrapper for non-aligned packet processing
static bool __not_in_flash_func(process_fast_command)(const uint8_t *packet) {
    // Copy to aligned union and process
    fast_cmd_union_t aligned_cmd;
    memcpy(&aligned_cmd, packet, 8);
    return process_fast_command_aligned(&aligned_cmd);
}

// Check if a byte could be the start of a fast command
static inline bool is_fast_cmd_start(uint8_t byte) {
    return (byte == FAST_CMD_MOUSE_MOVE ||
            byte == FAST_CMD_MOUSE_CLICK ||
            byte == FAST_CMD_KEY_PRESS ||
            byte == FAST_CMD_KEY_COMBO ||
            byte == FAST_CMD_MULTI_MOVE ||
            byte == FAST_CMD_MOUSE_ABS ||
            byte == FAST_CMD_SMOOTH_MOVE ||
            byte == FAST_CMD_SMOOTH_CONFIG ||
            byte == FAST_CMD_SMOOTH_CLEAR ||
            byte == FAST_CMD_TIMED_MOVE ||
            byte == FAST_CMD_SYNC ||
            byte == FAST_CMD_PING);
}

// Queue a complete packet into the ring buffer
static inline bool fast_ring_enqueue(const uint8_t *packet) {
    uint8_t next_head = (fast_ring_head + 1) & FAST_RING_MASK;
    
    if (next_head == fast_ring_tail) {
        // Ring buffer full
        fast_ring_overflows++;
        return false;
    }
    
    // Copy 8 bytes using 64-bit copy if possible
    fast_ring[fast_ring_head].raw.qword = *(const uint64_t *)packet;
    fast_ring_head = next_head;
    
    return true;
}

// Dequeue and process a packet from the ring buffer
static inline bool fast_ring_dequeue_and_process(void) {
    if (fast_ring_head == fast_ring_tail) {
        return false;  // Empty
    }
    
    // Process directly from ring buffer (zero-copy)
    process_fast_command_aligned(&fast_ring[fast_ring_tail]);
    fast_ring_tail = (fast_ring_tail + 1) & FAST_RING_MASK;
    
    return true;
}

// Process all pending commands in the ring buffer
void fast_cmd_process_pending(void) {
    while (fast_ring_dequeue_and_process()) {
        // Process until empty
    }
}

// Get fast command statistics
void fast_cmd_get_stats(uint32_t *count, uint32_t *errors, uint32_t *overflows) {
    if (count) *count = fast_cmd_count;
    if (errors) *errors = fast_cmd_errors;
    if (overflows) *overflows = fast_ring_overflows;
}

//--------------------------------------------------------------------+
// CP2110 Connection State Management
//--------------------------------------------------------------------+

static cp2110_connection_state_t g_connection_state = CP2110_STATE_WAITING;
static uint32_t g_last_data_time_ms = 0;
static uint32_t g_last_heartbeat_check_ms = 0;
static bool g_connection_led_updated = false;

// Ring buffer for non-blocking UART reception
// Use power-of-2 size for efficient modulo operation
// Memory is plentiful: use a large buffer to minimize overflow/drops
#define UART_RX_BUFFER_SIZE 2048
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

// UART RX interrupt handler for high-performance non-blocking reception
// Place in RAM to avoid XIP stalls during heavy serial traffic
static void __not_in_flash_func(on_uart_rx)(void) {
    while (uart_is_readable(KMBOX_UART)) {
        uint8_t ch = uart_getc(KMBOX_UART);
        
        // Calculate next head position using bitwise AND (faster than modulo)
        uint16_t next_head = (uart_rx_head + 1) & UART_RX_BUFFER_MASK;
        
        // Store character if buffer not full
        if (next_head != uart_rx_tail) {
            uart_rx_buffer[uart_rx_head] = ch;
            uart_rx_head = next_head;
        }
        // If buffer full, oldest data is discarded
    }
}

// Get character from ring buffer (non-blocking)
__attribute__((unused)) static int uart_rx_getchar(void) {
    if (uart_rx_head == uart_rx_tail) {
        return -1; // Buffer empty
    }
    
    uint8_t ch = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) & UART_RX_BUFFER_MASK;
    return ch;
}

// Peek for a full line in the ring buffer and copy it into dst (no terminator).
// Returns true if a full line was copied; out_len receives the length copied and
// term_len/term_buf are filled with the terminator bytes (if any).
static bool ringbuf_peek_line_and_copy(char *dst, size_t dst_size, size_t *out_len, char *term_buf, uint8_t *term_len)
{
    uint16_t head = uart_rx_head;
    uint16_t tail = uart_rx_tail;
    if (head == tail) return false; // empty

    // Scan from tail to head for a terminator
    uint16_t idx = tail;
    uint16_t found = UINT16_MAX;
    while (idx != head) {
        uint8_t ch = uart_rx_buffer[idx & UART_RX_BUFFER_MASK];
        if (ch == '\n' || ch == '\r') { found = idx; break; }
        idx = (idx + 1) & UART_RX_BUFFER_MASK;
    }
    if (found == UINT16_MAX) return false; // no full line

    // Determine terminator length (handle \r\n)
    uint8_t tlen = 1;
    char tbuf[2] = { (char)uart_rx_buffer[found & UART_RX_BUFFER_MASK], 0 };
    // if \r and next is \n, consider two-byte terminator
    if (tbuf[0] == '\r') {
        uint16_t next = (found + 1) & UART_RX_BUFFER_MASK;
        if (next != head && uart_rx_buffer[next] == '\n') {
            tbuf[1] = '\n';
            tlen = 2;
        }
    }

    // Compute line length (exclude terminator)
    size_t line_len = 0;
    uint16_t scan = tail;
    while (scan != found) { line_len++; scan = (scan + 1) & UART_RX_BUFFER_MASK; }

    // Truncate if necessary
    if (line_len >= dst_size) line_len = dst_size - 1;

    // Copy possibly wrapped data
    uint16_t first_chunk = UART_RX_BUFFER_SIZE - (tail & UART_RX_BUFFER_MASK);
    if (first_chunk > line_len) first_chunk = line_len;
    memcpy(dst, (const void *)&uart_rx_buffer[tail & UART_RX_BUFFER_MASK], first_chunk);
    if (line_len > first_chunk) {
        memcpy(dst + first_chunk, (const void *)&uart_rx_buffer[0], line_len - first_chunk);
    }
    dst[line_len] = '\0';

    // Advance tail past the line and its terminator
    uint16_t new_tail = (found + tlen) & UART_RX_BUFFER_MASK;
    // Update shared tail atomically (single word write is atomic on RP2040)
    uart_rx_tail = new_tail;

    if (out_len) *out_len = line_len;
    if (term_len) *term_len = tlen;
    if (term_buf && tlen > 0) { term_buf[0] = tbuf[0]; if (tlen == 2) term_buf[1] = tbuf[1]; }
    return true;
}

// Fast bulk read from ring buffer (no terminator scan). Returns number of bytes copied.
// Copies contiguous bytes up to maxlen from current tail to either head or buffer end.
static inline size_t ringbuf_read_chunk(uint8_t *dst, size_t maxlen) {
    uint16_t head = uart_rx_head;
    uint16_t tail = uart_rx_tail;
    if (head == tail || maxlen == 0) return 0;

    size_t available = (head - tail) & UART_RX_BUFFER_MASK;
    if (available == 0) return 0;

    size_t first_chunk = UART_RX_BUFFER_SIZE - (tail & UART_RX_BUFFER_MASK);
    if (first_chunk > available) first_chunk = available;
    if (first_chunk > maxlen) first_chunk = maxlen;

    memcpy(dst, (const void *)&uart_rx_buffer[tail & UART_RX_BUFFER_MASK], first_chunk);
    uart_rx_tail = (tail + (uint16_t)first_chunk) & UART_RX_BUFFER_MASK;
    return first_chunk;
}

// Initialize the serial handler
void kmbox_serial_init(void)
{
    // Reset UART RX ring buffer indices
    uart_rx_head = 0;
    uart_rx_tail = 0;

    // Initialize connection state
    g_connection_state = CP2110_STATE_WAITING;
    g_last_data_time_ms = 0;
    g_last_heartbeat_check_ms = 0;
    g_connection_led_updated = false;

    uart_init(KMBOX_UART, KMBOX_UART_BAUDRATE);
    // Standard 8N1 format and enable FIFO for throughput
    uart_set_format(KMBOX_UART, 8, 1, UART_PARITY_NONE);
    
    // Set up GPIO pins for selected UART (KMBOX_UART maps to pins below)
    gpio_set_function(KMBOX_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(KMBOX_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(KMBOX_UART_RX_PIN); // Help avoid spurious RX when line idle/floating
    
    // Enable UART FIFOs for better performance
    uart_set_fifo_enabled(KMBOX_UART, true);
    
    // Set up UART RX interrupt for non-blocking reception with higher priority
    int uart_irq = (KMBOX_UART == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, on_uart_rx);
    // Lower numerical value = higher priority on RP2040; 0 is highest. Use 0 for KMBox.
    irq_set_priority(uart_irq, 0);
    irq_set_enabled(uart_irq, true);
    
    // Enable UART RX interrupt
    uart_set_irq_enables(KMBOX_UART, true, false);
    // Initialize the kmbox commands module
    kmbox_commands_init();
    // Establish an initial timing baseline for the commands module
    uint32_t init_time_ms = to_ms_since_boot(get_absolute_time());
    kmbox_update_states(init_time_ms);
    g_last_data_time_ms = init_time_ms;
    
    // Set initial LED status to waiting for connection
    neopixel_set_status_override(STATUS_CP2110_WAITING);
    
    printf("KMBox serial handler initialized on UART%d via CP2110 (TX: GPIO%d, RX: GPIO%d) @ %d baud\n",
           (KMBOX_UART == uart0) ? 0 : 1, KMBOX_UART_TX_PIN, KMBOX_UART_RX_PIN, KMBOX_UART_BAUDRATE);
    printf("Waiting for CP2110 connection... (LED: breathing light blue)\n");
    
    // Give UART time to settle
    sleep_ms(100);
    
    // Send startup message to bridge
    kmbox_send_status("KMBox Ready");
    sleep_ms(10);
    kmbox_send_status("Waiting for USB devices...");
    sleep_ms(10);
    
    // Debug: send a simple test byte to verify UART TX is working
    uart_putc_raw(KMBOX_UART, 'T');
    uart_putc_raw(KMBOX_UART, 'E');
    uart_putc_raw(KMBOX_UART, 'S');
    uart_putc_raw(KMBOX_UART, 'T');
    uart_putc_raw(KMBOX_UART, '\n');
}

//--------------------------------------------------------------------+
// Connection State Management
//--------------------------------------------------------------------+

// Get current connection state
cp2110_connection_state_t kmbox_get_connection_state(void) {
    return g_connection_state;
}

// Check if connected and ready for commands
bool kmbox_is_connected(void) {
    return (g_connection_state == CP2110_STATE_CONNECTED || 
            g_connection_state == CP2110_STATE_ACTIVE);
}

//--------------------------------------------------------------------+
// Status Message Transmission to Bridge
//--------------------------------------------------------------------+

// Send status message to bridge via UART TX
// This allows the bridge to display KMBox status on its CDC interface
void kmbox_send_status(const char* message) {
    if (message == NULL) return;
    
    // Send message over UART (non-blocking)
    uart_puts(KMBOX_UART, message);
    uart_putc(KMBOX_UART, '\n');
}

// Send a response string back to the PC
void kmbox_send_response(const char *response) {
    if (!response) return;
    
    size_t len = strlen(response);
    for (size_t i = 0; i < len; i++) {
        uart_putc_raw(KMBOX_UART, response[i]);
    }
    uart_putc_raw(KMBOX_UART, '\n');
}

// Update connection state and LED indicator
static void set_connection_state(cp2110_connection_state_t new_state) {
    if (g_connection_state == new_state) return;
    
    cp2110_connection_state_t old_state = g_connection_state;
    g_connection_state = new_state;
    
    // Update LED based on new state
    switch (new_state) {
        case CP2110_STATE_WAITING:
            neopixel_set_status_override(STATUS_CP2110_WAITING);
            printf("CP2110: Waiting for connection (LED: breathing light blue)\n");
            break;
        case CP2110_STATE_CONNECTING:
            neopixel_set_status_override(STATUS_CP2110_CONNECTING);
            printf("CP2110: Handshake in progress (LED: yellow)\n");
            break;
        case CP2110_STATE_CONNECTED:
            neopixel_set_status_override(STATUS_CP2110_CONNECTED);
            printf("CP2110: Connected and ready (LED: green)\n");
            break;
        case CP2110_STATE_ACTIVE:
            neopixel_set_status_override(STATUS_CP2110_ACTIVE);
            // Don't spam logs for active state transitions
            break;
        case CP2110_STATE_DISCONNECTED:
            neopixel_set_status_override(STATUS_CP2110_DISCONNECTED);
            printf("CP2110: Connection lost (LED: breathing orange-red)\n");
            break;
    }
    
    (void)old_state;  // Suppress unused warning
}

// Check for connection timeout
static void check_connection_timeout(uint32_t current_time_ms) {
    // Only check periodically to avoid overhead
    if (current_time_ms - g_last_heartbeat_check_ms < CP2110_HEARTBEAT_CHECK_MS) {
        return;
    }
    g_last_heartbeat_check_ms = current_time_ms;
    
    // Check if we've timed out
    if (g_connection_state == CP2110_STATE_CONNECTED || 
        g_connection_state == CP2110_STATE_ACTIVE) {
        if (current_time_ms - g_last_data_time_ms > CP2110_HEARTBEAT_TIMEOUT_MS) {
            set_connection_state(CP2110_STATE_DISCONNECTED);
        }
    }
    
    // If disconnected for a while, go back to waiting
    if (g_connection_state == CP2110_STATE_DISCONNECTED) {
        if (current_time_ms - g_last_data_time_ms > CP2110_HEARTBEAT_TIMEOUT_MS * 2) {
            set_connection_state(CP2110_STATE_WAITING);
        }
    }
}

// Handle protocol commands (returns true if command was a protocol command)
static bool handle_protocol_command(const char *line, size_t len, uint32_t current_time_ms) {
    // Update last data time for any received data
    g_last_data_time_ms = current_time_ms;
    
    // Check for bridge sync request first (highest priority)
    if (len >= 17 && strncmp(line, "KMBOX_BRIDGE_SYNC", 17) == 0) {
        // Respond with ready message - bridge is looking for this
        kmbox_send_response("KMBOX_READY");
        set_connection_state(CP2110_STATE_CONNECTED);
        printf("Bridge sync: Connected via UART bridge\n");
        return true;
    }
    
    // Check for protocol commands
    if (len >= 10 && strncmp(line, "KMBOX_PING", 10) == 0) {
        // Respond with pong and version
        char response[64];
        snprintf(response, sizeof(response), "KMBOX_PONG:v%s", KMBOX_PROTOCOL_VERSION);
        kmbox_send_response(response);
        
        // If waiting, move to connecting state
        if (g_connection_state == CP2110_STATE_WAITING || 
            g_connection_state == CP2110_STATE_DISCONNECTED) {
            set_connection_state(CP2110_STATE_CONNECTING);
        }
        return true;
    }
    
    if (len >= 13 && strncmp(line, "KMBOX_CONNECT", 13) == 0) {
        // Confirm connection
        kmbox_send_response("KMBOX_READY");
        set_connection_state(CP2110_STATE_CONNECTED);
        return true;
    }
    
    if (len >= 16 && strncmp(line, "KMBOX_DISCONNECT", 16) == 0) {
        // Graceful disconnect
        kmbox_send_response("KMBOX_BYE");
        set_connection_state(CP2110_STATE_WAITING);
        return true;
    }
    
    if (len >= 12 && strncmp(line, "KMBOX_STATUS", 12) == 0) {
        // Report current status
        const char *state_names[] = {
            "WAITING", "CONNECTING", "CONNECTED", "ACTIVE", "DISCONNECTED"
        };
        char response[64];
        snprintf(response, sizeof(response), "KMBOX_STATE:%s", 
                 state_names[g_connection_state]);
        kmbox_send_response(response);
        return true;
    }
    
    return false;
}

// Process a line of input with protocol handling
static void process_line_with_protocol(const char *line, size_t len, 
                                        const char *terminator, uint8_t term_len,
                                        uint32_t current_time_ms) {
    // Update activity timestamp
    g_last_data_time_ms = current_time_ms;
    
    // Check if it's a protocol command
    if (handle_protocol_command(line, len, current_time_ms)) {
        return;
    }
    
    // For regular commands, must be connected
    if (!kmbox_is_connected()) {
        // If we receive non-protocol data while not connected, 
        // auto-connect for backwards compatibility
        if (g_connection_state == CP2110_STATE_WAITING ||
            g_connection_state == CP2110_STATE_DISCONNECTED) {
            // Auto-connect on any data (backwards compatibility mode)
            set_connection_state(CP2110_STATE_CONNECTED);
            printf("CP2110: Auto-connected (received data without handshake)\n");
        }
    }
    
    // Mark as active when receiving commands
    if (g_connection_state == CP2110_STATE_CONNECTED) {
        set_connection_state(CP2110_STATE_ACTIVE);
    }
    
    // Process the command through kmbox
    kmbox_process_serial_line(line, len, terminator, term_len, current_time_ms);
}

// Process any available serial input
void kmbox_serial_task(void)
{
    // Get current time
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    
    // Send periodic ping to bridge every 2 seconds for testing
    if (current_time_ms - last_kmbox_ping_time > 2000) {
        last_kmbox_ping_time = current_time_ms;
        kmbox_send_ping_to_bridge();
    }
    
    // Check for connection timeout
    check_connection_timeout(current_time_ms);
    
    // Process any pending timed moves (non-blocking scheduled execution)
    process_pending_timed_move();
    
    //------------------------------------------------------------------
    // FAST BINARY COMMAND PATH - Ultra-low latency
    //------------------------------------------------------------------
    // Check for fast binary commands first (highest priority)
    // These are 8-byte packets starting with specific command bytes
    
    uint16_t head = uart_rx_head;
    uint16_t tail = uart_rx_tail;
    
    while (head != tail) {
        uint8_t first_byte = uart_rx_buffer[tail];
        
        // Check if this could be a fast command
        if (is_fast_cmd_start(first_byte)) {
            // Check if we have a complete 8-byte packet
            uint16_t available = (head - tail) & UART_RX_BUFFER_MASK;
            
            if (available >= FAST_CMD_PACKET_SIZE) {
                // Read the 8-byte packet
                uint8_t packet[FAST_CMD_PACKET_SIZE];
                for (int i = 0; i < FAST_CMD_PACKET_SIZE; i++) {
                    packet[i] = uart_rx_buffer[(tail + i) & UART_RX_BUFFER_MASK];
                }
                
                // Advance tail past the packet
                uart_rx_tail = (tail + FAST_CMD_PACKET_SIZE) & UART_RX_BUFFER_MASK;
                
                // Process the fast command (in RAM for speed)
                process_fast_command(packet);
                
                // Update activity timestamp
                g_last_data_time_ms = current_time_ms;
                
                // Auto-connect on fast command
                if (!kmbox_is_connected()) {
                    set_connection_state(CP2110_STATE_CONNECTED);
                }
                if (g_connection_state == CP2110_STATE_CONNECTED) {
                    set_connection_state(CP2110_STATE_ACTIVE);
                }
                
                // Update head/tail for next iteration
                head = uart_rx_head;
                tail = uart_rx_tail;
                continue;
            } else {
                // Not enough data yet for a complete packet, wait
                break;
            }
        } else {
            // Not a fast command, fall through to text processing
            break;
        }
    }
    
    //------------------------------------------------------------------
    // TEXT COMMAND PATH - Full line parsing
    //------------------------------------------------------------------
    // Fast path: check for a full line and hand it to parser in one call
    char linebuf[KMBOX_CMD_BUFFER_SIZE];
    size_t line_len = 0;
    char termbuf[2];
    uint8_t termlen = 0;
    while (ringbuf_peek_line_and_copy(linebuf, sizeof(linebuf), &line_len, termbuf, &termlen)) {
        // Process with protocol handling
        process_line_with_protocol(linebuf, line_len, termbuf, termlen, current_time_ms);
    }

    // Fallback: process any remaining partial bytes in chunks to reduce per-byte overhead
    // Only process partial data if we're connected (protocol commands must be complete lines)
    if (kmbox_is_connected()) {
        uint8_t tmp[128];
        size_t n;
        while ((n = ringbuf_read_chunk(tmp, sizeof(tmp))) > 0) {
            // Skip if it looks like a partial fast command waiting for more bytes
            if (n > 0 && is_fast_cmd_start(tmp[0]) && n < FAST_CMD_PACKET_SIZE) {
                // Put bytes back by adjusting tail (can't easily do this, so just process them)
                // Actually, since fast commands should be complete above, this shouldn't happen often
            }
            
            // Update activity timestamp for any data
            g_last_data_time_ms = current_time_ms;
            
            // Manually unroll small inner loop for speed
            size_t i = 0;
            for (; i + 4 <= n; i += 4) {
                kmbox_process_serial_char((char)tmp[i + 0], current_time_ms);
                kmbox_process_serial_char((char)tmp[i + 1], current_time_ms);
                kmbox_process_serial_char((char)tmp[i + 2], current_time_ms);
                kmbox_process_serial_char((char)tmp[i + 3], current_time_ms);
            }
            for (; i < n; ++i) {
                kmbox_process_serial_char((char)tmp[i], current_time_ms);
            }
        }
    }
    
    // Update button states (handles timing for releases)
    kmbox_update_states(current_time_ms);
}

// Send mouse report with kmbox button states
bool kmbox_send_mouse_report(void)
{
    // Check if USB is ready
    if (!tud_hid_ready()) {
        return false;
    }
    
    // Ensure library state (timers/clicks) is up-to-date for this frame
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    kmbox_update_states(current_time_ms);

    // Get the report data from kmbox commands
    uint8_t buttons;
    int8_t x, y, wheel, pan;
    kmbox_get_mouse_report(&buttons, &x, &y, &wheel, &pan);
    
    // Send the report using TinyUSB
    bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, x, y, wheel, pan);
    
    if (success) {
        // Trigger rainbow effect periodically when KMBox commands are processed
        static uint32_t rainbow_counter = 0;
        if (++rainbow_counter % 50 == 0) {
            neopixel_trigger_rainbow_effect();
        }
    }
    
    return success;
}

// Send ping packet to bridge for bidirectional testing
void kmbox_send_ping_to_bridge(void) {
    fast_cmd_ping_t ping = {
        .cmd = FAST_CMD_PING,
        ._pad = {0}
    };
    
    // Send via UART TX (GPIO0) to bridge
    uart_write_blocking(KMBOX_UART, (const uint8_t*)&ping, sizeof(ping));
    kmbox_ping_count++;
}
