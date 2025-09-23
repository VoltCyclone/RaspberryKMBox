/*
 * KMBox Serial Command Handler
 * Integrates kmbox-commands library with the PIOKMBox firmware
 * Uses dedicated UART1 for KMBox serial input (separate from debug UART0)
 */

#include "kmbox_serial_handler.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "usb_hid.h"
#include "led_control.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <stdio.h>

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
static int uart_rx_getchar(void) {
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
    
    printf("KMBox serial handler initialized on UART1 (TX: GPIO%d, RX: GPIO%d) @ %d baud\n",
           KMBOX_UART_TX_PIN, KMBOX_UART_RX_PIN, KMBOX_UART_BAUDRATE);
}

// Process any available serial input
void kmbox_serial_task(void)
{
    // Get current time
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    // Fast path: check for a full line and hand it to parser in one call
    char linebuf[KMBOX_CMD_BUFFER_SIZE];
    size_t line_len = 0;
    char termbuf[2];
    uint8_t termlen = 0;
    while (ringbuf_peek_line_and_copy(linebuf, sizeof(linebuf), &line_len, termbuf, &termlen)) {
        kmbox_process_serial_line(linebuf, line_len, termbuf, termlen, current_time_ms);
    }

    // Fallback: process any remaining partial bytes in chunks to reduce per-byte overhead
    uint8_t tmp[128];
    size_t n;
    while ((n = ringbuf_read_chunk(tmp, sizeof(tmp))) > 0) {
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
