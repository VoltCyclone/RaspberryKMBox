/*
 * KMBox Serial Command Handler
 * Integrates kmbox-commands library with the PIOKMBox firmware
 * Uses RP2350 USB CDC-to-UART bridge for serial communication
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
#include "bridge_protocol.h"
#include "usb_hid.h"
#include "led_control.h"
#include "smooth_injection.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include <stdio.h>
#include <string.h>

//--------------------------------------------------------------------+
// Alignment-Safe Memory Access Helpers
//--------------------------------------------------------------------+
// ARM Cortex-M supports unaligned access but it's slower.
// These helpers ensure portable, efficient access.

static inline int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}

static inline uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

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

// Statistics for fast commands
static uint32_t fast_cmd_count = 0;
static uint32_t fast_cmd_errors = 0;
static uint32_t fast_ring_overflows = 0;

//--------------------------------------------------------------------+
// Async/Non-blocking State for Timed Operations
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

// Debug: count total UART RX bytes received
static volatile uint32_t uart_rx_byte_count = 0;

// Get synchronized timestamp (device time adjusted for PC offset)
static inline uint64_t get_synced_time_us(void) {
    return time_us_64();
}

//--------------------------------------------------------------------+
// Bridge Protocol Parser (Variable-Length Optimized)
//--------------------------------------------------------------------+

// Parse and process bridge protocol packets
// Returns number of bytes consumed (0 if incomplete packet)
static uint8_t process_bridge_packet(const uint8_t *data, size_t available) {
    if (available < 2) return 0;  // Need at least sync + cmd
    
    // Check for sync byte
    if (data[0] != BRIDGE_SYNC_BYTE) {
        return 1;  // Skip invalid sync byte
    }
    
    uint8_t cmd = data[1];
    
    switch (cmd) {
        case BRIDGE_CMD_MOUSE_MOVE: {
            // Move: [SYNC][CMD][x_lo][x_hi][y_lo][y_hi] = 6 bytes
            if (available < 6) return 0;
            int16_t x = read_i16_le(data + 2);
            int16_t y = read_i16_le(data + 4);
            // Add to accumulator - physical mouse callback will send combined result
            kmbox_add_mouse_movement(x, y);
            return 6;
        }
        
        case BRIDGE_CMD_MOUSE_WHEEL: {
            // Wheel: [SYNC][CMD][wheel] = 3 bytes
            if (available < 3) return 0;
            int8_t wheel = (int8_t)data[2];
            kmbox_add_wheel_movement(wheel);
            return 3;
        }
        
        case BRIDGE_CMD_BUTTON_SET: {
            // Button: [SYNC][CMD][mask][state] = 4 bytes
            if (available < 4) return 0;
            uint8_t mask = data[2];
            uint8_t state = data[3];
            // Update button state directly
            kmbox_update_physical_buttons(state ? mask : 0);
            return 4;
        }
        
        case BRIDGE_CMD_MOUSE_MOVE_WHEEL: {
            // Move+Wheel: [SYNC][CMD][x_lo][x_hi][y_lo][y_hi][wheel] = 7 bytes
            if (available < 7) return 0;
            int16_t x = *((int16_t*)(data + 2));
            int16_t y = *((int16_t*)(data + 4));
            int8_t wheel = (int8_t)data[6];
            // Add to accumulator - physical mouse callback will send combined result
            kmbox_add_mouse_movement(x, y);
            kmbox_add_wheel_movement(wheel);
            return 7;
        }
        
        case BRIDGE_CMD_PING: {
            // Ping: [SYNC][CMD] = 2 bytes
            return 2;
        }
        
        case BRIDGE_CMD_RESET: {
            // Reset: [SYNC][CMD] = 2 bytes
            // Clear all queued movement
            kmbox_add_mouse_movement(0, 0);
            kmbox_add_wheel_movement(0);
            kmbox_update_physical_buttons(0);
            return 2;
        }
        
        default:
            // Unknown command - skip sync byte and try again
            return 1;
    }
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
            report.x = kmbox_clamp_movement_i8(m->x);
            report.y = kmbox_clamp_movement_i8(m->y);
            report.wheel = m->wheel;
            process_mouse_report(&report);
            
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_MOUSE_CLICK: {
            const fast_cmd_click_t *c = &cmd->click;
            uint8_t count = c->count ? c->count : 1;
            
            // Map button number to HID bit mask using library function
            uint8_t btn_mask = kmbox_map_button_to_hid_mask(c->button);
            
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
// NOTE: 0x0A is FAST_CMD_TIMED_MOVE but also '\n' (newline)
// We exclude 0x0A and 0x0D here to prevent line terminators from being
// mistaken for fast command starts when using text protocol
static inline bool is_fast_cmd_start(uint8_t byte) {
    // Exclude newline (0x0A) and carriage return (0x0D) - they're line terminators
    if (byte == 0x0A || byte == 0x0D) {
        return false;
    }
    return (byte == FAST_CMD_MOUSE_MOVE ||
            byte == FAST_CMD_MOUSE_CLICK ||
            byte == FAST_CMD_KEY_PRESS ||
            byte == FAST_CMD_KEY_COMBO ||
            byte == FAST_CMD_MULTI_MOVE ||
            byte == FAST_CMD_MOUSE_ABS ||
            byte == FAST_CMD_SMOOTH_MOVE ||
            byte == FAST_CMD_SMOOTH_CONFIG ||
            byte == FAST_CMD_SMOOTH_CLEAR ||
            // FAST_CMD_TIMED_MOVE (0x0A) excluded above - conflicts with '\n'
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
// Bridge Connection State Management
//--------------------------------------------------------------------+

static bridge_connection_state_t g_connection_state = BRIDGE_STATE_WAITING;
static uint32_t g_last_data_time_ms = 0;
static uint32_t g_last_heartbeat_check_ms = 0;
static bool g_connection_led_updated = false;

//--------------------------------------------------------------------+
// UART TX DMA Configuration (Post-Boot Optimization)
//--------------------------------------------------------------------+

// DMA state - only initialized after USB is running
static bool uart_tx_dma_initialized = false;
static int uart_tx_dma_chan = -1;
static volatile bool tx_dma_busy = false;

static bool uart_rx_dma_initialized = false;
static int uart_rx_dma_chan = -1;

// Double-buffered TX DMA for zero-wait non-blocking transmission
#define UART_TX_DMA_BUFFER_SIZE 512
static uint8_t __attribute__((aligned(4))) uart_tx_dma_buffer_a[UART_TX_DMA_BUFFER_SIZE];
static uint8_t __attribute__((aligned(4))) uart_tx_dma_buffer_b[UART_TX_DMA_BUFFER_SIZE];
static volatile uint16_t tx_buffer_a_len = 0;
static volatile uint16_t tx_buffer_b_len = 0;
static volatile uint8_t active_tx_buffer = 0;  // 0=A, 1=B

// RX DMA buffer for continuous circular reception
// CRITICAL: Buffer MUST be aligned to its size for DMA ring mode!
#define UART_RX_DMA_BUFFER_SIZE 1024
static uint8_t __attribute__((aligned(UART_RX_DMA_BUFFER_SIZE))) uart_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE];
static volatile uint16_t uart_rx_dma_read_pos = 0;

//--------------------------------------------------------------------+
// UART RX Ring Buffer
//--------------------------------------------------------------------+

// Ring buffer for non-blocking UART reception
// Use power-of-2 size for efficient modulo operation
// Memory is plentiful: use a large buffer to minimize overflow/drops
#define UART_RX_BUFFER_SIZE 2048
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

//--------------------------------------------------------------------+
// UART TX DMA Interrupt Handler (Post-Boot)
//--------------------------------------------------------------------+

// DMA completion handler for UART TX (placed in RAM for low latency)
static void __not_in_flash_func(uart_tx_dma_handler)(void) {
    // Clear interrupt for our channel on IRQ_1
    if (uart_tx_dma_chan >= 0) {
        dma_hw->ints1 = 1u << uart_tx_dma_chan;
    }
    
    // Mark TX as no longer busy - next transmission can proceed
    tx_dma_busy = false;
}

//--------------------------------------------------------------------+
// UART RX Interrupt Handler
//--------------------------------------------------------------------+

// UART RX interrupt handler for high-performance non-blocking reception
// Place in RAM to avoid XIP stalls during heavy serial traffic
static void __not_in_flash_func(on_uart_rx)(void) {
    while (uart_is_readable(KMBOX_UART)) {
        uint8_t ch = uart_getc(KMBOX_UART);
        uart_rx_byte_count++;  // Debug counter
        
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
    
    if (found == UINT16_MAX) {
        return false; // no full line yet
    }

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
    g_connection_state = BRIDGE_STATE_WAITING;
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
    neopixel_set_status_override(STATUS_BRIDGE_WAITING);
}

//--------------------------------------------------------------------+
// Post-Boot DMA Initialization
//--------------------------------------------------------------------+

// Initialize UART TX and RX DMA after USB is fully running
// This avoids boot-time conflicts with USB/PIO initialization
void kmbox_serial_init_dma(void) {
    if (uart_tx_dma_initialized && uart_rx_dma_initialized) {
        return;
    }
    
    // Try to claim DMA channel without panicking
    uart_tx_dma_chan = dma_claim_unused_channel(false); // false = don't panic
    if (uart_tx_dma_chan < 0) {
        return;
    }
    
    dma_channel_config c = dma_channel_get_default_config(uart_tx_dma_chan);
    
    // Configure for 8-bit transfers from memory to UART TX FIFO
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);   // Increment read address
    channel_config_set_write_increment(&c, false); // Fixed write to UART DR
    channel_config_set_dreq(&c, uart_get_dreq(KMBOX_UART, true)); // TX DREQ
    
    // Set high priority for low latency
    channel_config_set_high_priority(&c, true);
    
    // Apply configuration (but don't start yet)
    dma_channel_configure(
        uart_tx_dma_chan,
        &c,
        &uart_get_hw(KMBOX_UART)->dr,  // Write to UART data register
        NULL,  // Read address set per transfer
        0,     // Transfer count set per transfer
        false  // Don't start yet
    );
    
    // Enable DMA completion interrupt on DMA_IRQ_1 (avoid IRQ_0 conflicts)
    dma_channel_set_irq1_enabled(uart_tx_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, uart_tx_dma_handler);
    irq_set_priority(DMA_IRQ_1, 0x80); // Lower priority
    irq_set_enabled(DMA_IRQ_1, true);
    
    uart_tx_dma_initialized = true;
    printf("UART TX DMA initialized on channel %d\n", uart_tx_dma_chan);
    
    // Initialize RX DMA for continuous circular reception
    uart_rx_dma_chan = dma_claim_unused_channel(false);
    if (uart_rx_dma_chan < 0) {
        printf("No DMA channel for RX, using interrupt mode\n");
        kmbox_send_status("DMA TX enabled");
        return;
    }
    
    dma_channel_config rx_config = dma_channel_get_default_config(uart_rx_dma_chan);
    
    // Configure for 8-bit transfers from UART RX FIFO to memory
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);  // Fixed read from UART DR
    channel_config_set_write_increment(&rx_config, true);  // Increment write address
    channel_config_set_dreq(&rx_config, uart_get_dreq(KMBOX_UART, false)); // RX DREQ
    
    // Enable ring buffer mode for circular reception
    channel_config_set_ring(&rx_config, true, 10); // 2^10 = 1024 bytes ring on write
    
    // High priority for low latency
    channel_config_set_high_priority(&rx_config, true);
    
    // Configure and start continuous circular DMA
    // Use maximum transfer count for continuous operation
    // Ring mode wraps the address, but we need infinite transfers
    dma_channel_configure(
        uart_rx_dma_chan,
        &rx_config,
        uart_rx_dma_buffer,                    // Write to DMA buffer
        &uart_get_hw(KMBOX_UART)->dr,         // Read from UART data register
        0xFFFFFFFF,                            // Maximum transfers for continuous reception
        true                                   // Start immediately
    );
    
    // Disable UART RX interrupt since DMA is handling it
    uart_set_irq_enables(KMBOX_UART, false, false);
    
    uart_rx_dma_initialized = true;
    printf("UART RX DMA initialized on channel %d (circular buffer)\n", uart_rx_dma_chan);
    kmbox_send_status("DMA TX+RX enabled");
}

//--------------------------------------------------------------------+
// Connection State Management
//--------------------------------------------------------------------+

// Get current connection state
bridge_connection_state_t kmbox_get_connection_state(void) {
    return g_connection_state;
}

// Check if connected and ready for commands
bool kmbox_is_connected(void) {
    return (g_connection_state == BRIDGE_STATE_CONNECTED || 
            g_connection_state == BRIDGE_STATE_ACTIVE);
}

//--------------------------------------------------------------------+
// Status Message Transmission to Bridge
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Non-blocking UART TX using DMA (Post-Boot only)
//--------------------------------------------------------------------+

// Send data via UART using DMA with double-buffering
// Returns true if queued successfully, false if both buffers full
static bool uart_send_dma(const uint8_t* data, uint16_t len) {
    // Check if DMA is initialized
    if (!uart_tx_dma_initialized || uart_tx_dma_chan < 0 || len == 0 || len > UART_TX_DMA_BUFFER_SIZE) {
        // Fallback to blocking TX if DMA not ready or data too large
        for (uint16_t i = 0; i < len; i++) {
            uart_putc_raw(KMBOX_UART, data[i]);
        }
        return true;
    }
    
    // If DMA is busy, queue to inactive buffer (double-buffering)
    if (tx_dma_busy) {
        uint8_t inactive_buffer = 1 - active_tx_buffer;
        volatile uint16_t* inactive_len = (inactive_buffer == 0) ? &tx_buffer_a_len : &tx_buffer_b_len;
        
        // Check if inactive buffer has space
        if (*inactive_len + len > UART_TX_DMA_BUFFER_SIZE) {
            return false; // Both buffers full, drop data
        }
        
        // Queue to inactive buffer
        uint8_t* inactive_buf = (inactive_buffer == 0) ? uart_tx_dma_buffer_a : uart_tx_dma_buffer_b;
        memcpy(inactive_buf + *inactive_len, data, len);
        *inactive_len += len;
        return true;
    }
    
    // DMA idle, start immediate transfer
    uint8_t* active_buf = (active_tx_buffer == 0) ? uart_tx_dma_buffer_a : uart_tx_dma_buffer_b;
    memcpy(active_buf, data, len);
    
    tx_dma_busy = true;
    
    // Start DMA transfer
    dma_channel_transfer_from_buffer_now(uart_tx_dma_chan, active_buf, len);
    
    return true;
}

// Process queued TX buffer (called from main loop)
static inline void process_uart_tx_queue(void) {
    if (!uart_tx_dma_initialized || tx_dma_busy || uart_tx_dma_chan < 0) return;
    
    // Check if inactive buffer has pending data
    uint8_t inactive_buffer = 1 - active_tx_buffer;
    volatile uint16_t* inactive_len = (inactive_buffer == 0) ? &tx_buffer_a_len : &tx_buffer_b_len;
    
    if (*inactive_len > 0) {
        // Swap buffers and transmit queued data
        active_tx_buffer = inactive_buffer;
        uint8_t* active_buf = (active_tx_buffer == 0) ? uart_tx_dma_buffer_a : uart_tx_dma_buffer_b;
        uint16_t len = *inactive_len;
        
        *inactive_len = 0; // Clear queued length
        tx_dma_busy = true;
        
        dma_channel_transfer_from_buffer_now(uart_tx_dma_chan, active_buf, len);
    }
}

// Send status message to bridge via UART TX
// This allows the bridge to display KMBox status on its CDC interface
void kmbox_send_status(const char* message) {
    if (message == NULL) return;
    
    // Try DMA if initialized, otherwise use blocking
    if (uart_tx_dma_initialized) {
        size_t len = strlen(message);
        if (len > UART_TX_DMA_BUFFER_SIZE - 2) {
            len = UART_TX_DMA_BUFFER_SIZE - 2;
        }
        
        uint8_t buffer[UART_TX_DMA_BUFFER_SIZE];
        memcpy(buffer, message, len);
        buffer[len++] = '\n';
        
        if (!uart_send_dma(buffer, len)) {
            // Fallback to blocking if DMA buffers full
            uart_puts(KMBOX_UART, message);
            uart_putc(KMBOX_UART, '\n');
        }
    } else {
        // DMA not initialized yet, use blocking
        uart_puts(KMBOX_UART, message);
        uart_putc(KMBOX_UART, '\n');
    }
}

// Send a response string back to the PC
void kmbox_send_response(const char *response) {
    if (!response) return;
    
    size_t len = strlen(response);
    
    // Try DMA if initialized
    if (uart_tx_dma_initialized && len < UART_TX_DMA_BUFFER_SIZE - 2) {
        uint8_t buffer[UART_TX_DMA_BUFFER_SIZE];
        memcpy(buffer, response, len);
        buffer[len++] = '\n';
        
        if (!uart_send_dma(buffer, len)) {
            // Fallback to blocking if DMA buffers full
            for (size_t i = 0; i < len - 1; i++) {
                uart_putc_raw(KMBOX_UART, response[i]);
            }
            uart_putc_raw(KMBOX_UART, '\n');
        }
    } else {
        // DMA not initialized or message too long, use blocking
        for (size_t i = 0; i < len; i++) {
            uart_putc_raw(KMBOX_UART, response[i]);
        }
        uart_putc_raw(KMBOX_UART, '\n');
    }
}

// Update connection state and LED indicator
static void set_connection_state(bridge_connection_state_t new_state) {
    if (g_connection_state == new_state) return;
    
    bridge_connection_state_t old_state = g_connection_state;
    g_connection_state = new_state;
    
    // Update LED based on new state
    switch (new_state) {
        case BRIDGE_STATE_WAITING:
            neopixel_set_status_override(STATUS_BRIDGE_WAITING);
            printf("Bridge: Waiting for connection (LED: breathing light blue)\n");
            break;
        case BRIDGE_STATE_CONNECTING:
            neopixel_set_status_override(STATUS_BRIDGE_CONNECTING);
            printf("Bridge: Handshake in progress (LED: yellow)\n");
            break;
        case BRIDGE_STATE_CONNECTED:
            neopixel_set_status_override(STATUS_BRIDGE_CONNECTED);
            printf("Bridge: Connected and ready (LED: green)\n");
            break;
        case BRIDGE_STATE_ACTIVE:
            neopixel_set_status_override(STATUS_BRIDGE_ACTIVE);
            // Don't spam logs for active state transitions
            break;
        case BRIDGE_STATE_DISCONNECTED:
            neopixel_set_status_override(STATUS_BRIDGE_DISCONNECTED);
            printf("Bridge: Connection lost (LED: breathing orange-red)\n");
            break;
    }
    
    (void)old_state;  // Suppress unused warning
}

// Check for connection timeout
static void check_connection_timeout(uint32_t current_time_ms) {
    // Only check periodically to avoid overhead
    if (current_time_ms - g_last_heartbeat_check_ms < BRIDGE_HEARTBEAT_CHECK_MS) {
        return;
    }
    g_last_heartbeat_check_ms = current_time_ms;
    
    // Check if we've timed out
    if (g_connection_state == BRIDGE_STATE_CONNECTED || 
        g_connection_state == BRIDGE_STATE_ACTIVE) {
        if (current_time_ms - g_last_data_time_ms > BRIDGE_HEARTBEAT_TIMEOUT_MS) {
            set_connection_state(BRIDGE_STATE_DISCONNECTED);
        }
    }
    
    // If disconnected for a while, go back to waiting
    if (g_connection_state == BRIDGE_STATE_DISCONNECTED) {
        if (current_time_ms - g_last_data_time_ms > BRIDGE_HEARTBEAT_TIMEOUT_MS * 2) {
            set_connection_state(BRIDGE_STATE_WAITING);
        }
    }
}

// Handle protocol commands (returns true if command was a protocol command)
static bool handle_protocol_command(const char *line, size_t len, uint32_t current_time_ms) {
    // Update last data time for any received data
    g_last_data_time_ms = current_time_ms;
    
    // ============================================================
    // Simple Text Protocol Commands (for reliable bridge communication)
    // Format: M<x>,<y>  - Mouse move
    //         W<delta>  - Wheel scroll
    //         B<mask>   - Button state
    // ============================================================
    
    // Simple mouse move: M<x>,<y>
    if (len >= 3 && line[0] == 'M') {
        int x, y;
        if (sscanf(line, "M%d,%d", &x, &y) == 2) {
            // Debug: flash green to show M command received
            neopixel_trigger_activity_flash_color(0x0000FF00);  // Green = M command
            
            // Queue movement through kmbox API - this adds to the accumulator
            // The next physical mouse report will automatically include this
            // movement combined with the physical movement, achieving the counter effect
            kmbox_add_mouse_movement((int16_t)x, (int16_t)y);
            // Auto-connect on command
            if (!kmbox_is_connected()) {
                set_connection_state(BRIDGE_STATE_CONNECTED);
            }
            if (g_connection_state == BRIDGE_STATE_CONNECTED) {
                set_connection_state(BRIDGE_STATE_ACTIVE);
            }
            return true;
        }
    }
    
    // Simple wheel: W<delta>
    if (len >= 2 && line[0] == 'W') {
        int delta;
        if (sscanf(line, "W%d", &delta) == 1) {
            kmbox_add_wheel_movement((int8_t)delta);
            return true;
        }
    }
    
    // Simple buttons: B<mask>
    if (len >= 2 && line[0] == 'B') {
        int mask;
        if (sscanf(line, "B%d", &mask) == 1) {
            kmbox_update_physical_buttons((uint8_t)mask);
            return true;
        }
    }
    
    // Simple ping: P (keepalive from bridge in text mode)
    if (len >= 1 && line[0] == 'P') {
        // Just a keepalive - update activity timestamp and auto-connect
        if (!kmbox_is_connected()) {
            set_connection_state(BRIDGE_STATE_CONNECTED);
        }
        if (g_connection_state == BRIDGE_STATE_CONNECTED) {
            set_connection_state(BRIDGE_STATE_ACTIVE);
        }
        // Send pong response so bridge knows we're alive
        kmbox_send_response("KMBOX_PONG");
        return true;
    }
    
    // Echo test: E<data> - echo back the data to confirm UART works
    // Bridge sends "ETEST123", KMBox responds "ECHO:TEST123"
    if (len >= 2 && line[0] == 'E') {
        // Flash LED to show we received something
        neopixel_trigger_activity_flash_color(0x00FFFF00);  // Yellow = echo received
        
        // Echo back the data
        char response[64];
        snprintf(response, sizeof(response), "ECHO:%s", line + 1);
        
        // Send via UART
        for (const char* p = response; *p; p++) {
            uart_putc_raw(KMBOX_UART, *p);
        }
        uart_putc_raw(KMBOX_UART, '\n');
        
        printf("Echo test: received '%s', sent '%s'\n", line, response);
        return true;
    }
    
    // ============================================================
    // KMBox Protocol Commands (handshake, status, etc.)
    // ============================================================
    
    // Check for bridge sync request first (highest priority)
    if (len >= 17 && strncmp(line, "KMBOX_BRIDGE_SYNC", 17) == 0) {
        // Respond with ready message - bridge is looking for this
        kmbox_send_response("KMBOX_READY");
        set_connection_state(BRIDGE_STATE_CONNECTED);
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
        if (g_connection_state == BRIDGE_STATE_WAITING || 
            g_connection_state == BRIDGE_STATE_DISCONNECTED) {
            set_connection_state(BRIDGE_STATE_CONNECTING);
        }
        return true;
    }
    
    if (len >= 13 && strncmp(line, "KMBOX_CONNECT", 13) == 0) {
        // Confirm connection
        kmbox_send_response("KMBOX_READY");
        set_connection_state(BRIDGE_STATE_CONNECTED);
        return true;
    }
    
    if (len >= 16 && strncmp(line, "KMBOX_DISCONNECT", 16) == 0) {
        // Graceful disconnect
        kmbox_send_response("KMBOX_BYE");
        set_connection_state(BRIDGE_STATE_WAITING);
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
    
    // Device info command - sends attached mouse VID/PID and name
    if (len >= 10 && strncmp(line, "KMBOX_INFO", 10) == 0) {
        char response[128];
        uint16_t vid = get_attached_vid();
        uint16_t pid = get_attached_pid();
        const char *mfr = get_attached_manufacturer();
        const char *prod = get_attached_product();
        
        // Send VID:PID first
        snprintf(response, sizeof(response), "KMBOX_VID:%04X", vid);
        kmbox_send_response(response);
        
        snprintf(response, sizeof(response), "KMBOX_PID:%04X", pid);
        kmbox_send_response(response);
        
        // Send manufacturer (may be empty)
        snprintf(response, sizeof(response), "KMBOX_MFR:%s", mfr[0] ? mfr : "Unknown");
        kmbox_send_response(response);
        
        // Send product name (may be empty)
        snprintf(response, sizeof(response), "KMBOX_PROD:%s", prod[0] ? prod : "Unknown");
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
        if (g_connection_state == BRIDGE_STATE_WAITING ||
            g_connection_state == BRIDGE_STATE_DISCONNECTED) {
            // Auto-connect on any data (backwards compatibility mode)
            set_connection_state(BRIDGE_STATE_CONNECTED);
            printf("Bridge: Auto-connected (received data without handshake)\n");
        }
    }
    
    // Mark as active when receiving commands
    if (g_connection_state == BRIDGE_STATE_CONNECTED) {
        set_connection_state(BRIDGE_STATE_ACTIVE);
    }
    
    // Process the command through kmbox
    kmbox_process_serial_line(line, len, terminator, term_len, current_time_ms);
}

// Process any available serial input
void kmbox_serial_task(void)
{
    // Get current time
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    
    // Process UART TX DMA queue if initialized
    if (uart_tx_dma_initialized) {
        process_uart_tx_queue();
    }
    
    // Read from DMA buffer if RX DMA is active
    if (uart_rx_dma_initialized) {
        // Get current DMA write position from write_addr register (correct for ring mode)
        uintptr_t current_addr = (uintptr_t)dma_channel_hw_addr(uart_rx_dma_chan)->write_addr;
        uintptr_t base_addr = (uintptr_t)uart_rx_dma_buffer;
        uint16_t dma_write_pos = (uint16_t)((current_addr - base_addr) & (UART_RX_DMA_BUFFER_SIZE - 1));
        
        // Debug: track if we receive any UART data at all
        static uint32_t uart_rx_byte_count = 0;
        static uint32_t last_debug_flash_count = 0;
        
        // Process all available bytes in circular buffer
        while (uart_rx_dma_read_pos != dma_write_pos) {
            uint8_t byte = uart_rx_dma_buffer[uart_rx_dma_read_pos];
            uart_rx_byte_count++;
            
            // Flash YELLOW every 100 bytes received to show UART RX is working
            if (uart_rx_byte_count - last_debug_flash_count >= 100) {
                neopixel_trigger_activity_flash_color(0x00FFFF00);  // Yellow = UART RX active
                last_debug_flash_count = uart_rx_byte_count;
            }
            
            // Copy to ring buffer for command processing
            uint16_t next_head = (uart_rx_head + 1) & UART_RX_BUFFER_MASK;
            if (next_head != uart_rx_tail) {
                uart_rx_buffer[uart_rx_head] = byte;
                uart_rx_head = next_head;
            }
            
            uart_rx_dma_read_pos = (uart_rx_dma_read_pos + 1) & (UART_RX_DMA_BUFFER_SIZE - 1);
        }
    }
    
    // Send periodic ping to bridge every 5 seconds for keepalive
    if (current_time_ms - last_kmbox_ping_time > 5000) {
        last_kmbox_ping_time = current_time_ms;
        kmbox_send_ping_to_bridge();
    }
    
    // Check for connection timeout
    check_connection_timeout(current_time_ms);
    
    // Process any pending timed moves (non-blocking scheduled execution)
    process_pending_timed_move();
    
    //------------------------------------------------------------------
    // BRIDGE PROTOCOL PATH - Optimized variable-length packets
    //------------------------------------------------------------------
    // Check for bridge protocol packets first (SYNC byte = 0xBD)
    // These are 2-7 byte variable-length packets
    
    uint16_t head = uart_rx_head;
    uint16_t tail = uart_rx_tail;
    
    while (head != tail) {
        uint8_t first_byte = uart_rx_buffer[tail];
        
        // Check for bridge protocol sync byte
        if (first_byte == BRIDGE_SYNC_BYTE) {
            // Re-read head to get latest DMA position
            head = uart_rx_head;
            uint16_t available = (head - tail) & UART_RX_BUFFER_MASK;
            
            // Need at least 6 bytes for smallest variable packet (mouse move)
            if (available < 6) {
                break;  // Wait for more data
            }
            
            // Build contiguous buffer for parser
            uint8_t packet[8];  // Max bridge packet size
            size_t copy_len = (available < 8) ? available : 8;
            for (size_t i = 0; i < copy_len; i++) {
                packet[i] = uart_rx_buffer[(tail + i) & UART_RX_BUFFER_MASK];
            }
            
            // Try to parse bridge packet
            uint8_t consumed = process_bridge_packet(packet, copy_len);
            
            if (consumed >= 2) {
                // Successfully parsed a valid packet
                // Advance tail past the parsed packet
                tail = (tail + consumed) & UART_RX_BUFFER_MASK;
                uart_rx_tail = tail;
                
                // Update activity timestamp
                g_last_data_time_ms = current_time_ms;
                
                // Auto-connect on bridge command
                if (!kmbox_is_connected()) {
                    set_connection_state(BRIDGE_STATE_CONNECTED);
                }
                if (g_connection_state == BRIDGE_STATE_CONNECTED) {
                    set_connection_state(BRIDGE_STATE_ACTIVE);
                }
                
                // Check if there's enough for another packet already
                head = uart_rx_head;
                uint16_t remaining = (head - tail) & UART_RX_BUFFER_MASK;
                if (remaining < 6) {
                    break;  // Wait for more data
                }
                continue;  // Parse next packet if we have enough
            } else if (consumed == 1) {
                // Bad command byte, skip past sync and try again
                tail = (tail + 1) & UART_RX_BUFFER_MASK;
                uart_rx_tail = tail;
                head = uart_rx_head;
                continue;
            } else {
                // Incomplete packet
                break;
            }
        }
        
        // Not a bridge sync byte - let text command path handle it
        break;
    }
    
    //------------------------------------------------------------------
    // FAST BINARY COMMAND PATH - Ultra-low latency (Legacy)
    //------------------------------------------------------------------
    // Check for fast binary commands (8-byte fixed packets)
    
    head = uart_rx_head;
    tail = uart_rx_tail;
    
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
                    set_connection_state(BRIDGE_STATE_CONNECTED);
                }
                if (g_connection_state == BRIDGE_STATE_CONNECTED) {
                    set_connection_state(BRIDGE_STATE_ACTIVE);
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
    // Skip any stray binary bytes (0xFE ping, 0xFF response markers) that got into text stream
    // These can happen during mode transitions or from legacy binary pings
    {
        uint16_t skip_head = uart_rx_head;
        uint16_t skip_tail = uart_rx_tail;
        uint16_t skipped = 0;
        while (skip_tail != skip_head) {
            uint8_t b = uart_rx_buffer[skip_tail & UART_RX_BUFFER_MASK];
            // Skip binary protocol markers and NULL bytes
            if (b == 0xFE || b == 0xFF || b == 0x00) {
                skip_tail = (skip_tail + 1) & UART_RX_BUFFER_MASK;
                uart_rx_tail = skip_tail;  // Update global
                skipped++;
            } else {
                break;  // Found a valid text byte
            }
        }
    }
    
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
    // IMPORTANT: Don't consume text command data that's waiting for a newline terminator!
    // Check if first byte looks like a text command start (M, W, B, P, K, etc.)
    {
        uint16_t fb_head = uart_rx_head;
        uint16_t fb_tail = uart_rx_tail;
        if (fb_head != fb_tail) {
            uint8_t first = uart_rx_buffer[fb_tail & UART_RX_BUFFER_MASK];
            // If it looks like the start of a text command, DON'T consume it
            // Wait for the newline to arrive so ringbuf_peek_line_and_copy can process it
            if (first == 'M' || first == 'W' || first == 'B' || first == 'P' || 
                first == 'K' || first == 'm' || first == 'k' ||
                first >= 0x20) {  // Any printable ASCII - likely text command
                // Skip the fallback - let text accumulate until we get a newline
                goto skip_fallback;
            }
        }
    }
    if (kmbox_is_connected()) {
        uint8_t tmp[128];
        size_t n;
        while ((n = ringbuf_read_chunk(tmp, sizeof(tmp))) > 0) {
            // Skip if it looks like a partial fast command waiting for more bytes
            if (n > 0 && is_fast_cmd_start(tmp[0]) && n < FAST_CMD_PACKET_SIZE) {
                // Re-insert bytes back into ring buffer for next time
                for (size_t i = 0; i < n; i++) {
                    // Calculate next head position
                    uint16_t next_head = (uart_rx_head + 1) & UART_RX_BUFFER_MASK;
                    if (next_head != uart_rx_tail) {
                        uart_rx_buffer[uart_rx_head] = tmp[i];
                        uart_rx_head = next_head;
                    } else {
                        // Buffer full, drop remaining bytes
                        break;
                    }
                }
                break; // Exit processing loop
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
    
skip_fallback:
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
    
    return success;
}

// Process pending bridge command injections
// NOTE: This function is no longer needed - bridge movements are added to the
// shared accumulator and the physical mouse callback sends combined reports.
// Kept for API compatibility.
void kmbox_process_bridge_injections(void) {
    // No-op: bridge movements go through shared accumulator
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
