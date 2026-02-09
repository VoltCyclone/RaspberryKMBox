/**
 * UART Buffer Configuration for KMBox
 * 
 * Recommended buffer sizes and ring buffer implementation
 * for high-performance UART communication at 921600 baud.
 */

#ifndef UART_BUFFERS_H
#define UART_BUFFERS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/uart.h"

//=============================================================================
// BUFFER SIZE CONFIGURATION
//=============================================================================

// Buffer sizes MUST be power of 2 for efficient modulo via bitmask
// At 921600 baud: ~92KB/sec = ~92 bytes/ms

// RX Buffer: Holds incoming commands from Bridge
// - Largest command is 8 bytes (fast binary protocol)
// - At 2 Mbaud: ~200KB/sec = ~200 bytes/ms
// - 512 bytes gives ~2.5ms of buffer time (safe margin at 2 Mbaud)
#define UART_RX_BUFFER_SIZE     512
#define UART_RX_BUFFER_MASK     (UART_RX_BUFFER_SIZE - 1)

// TX Buffer: Holds outgoing responses to Bridge  
// - Longest response ~64 bytes (status messages)
// - Usually just short ACKs (8-16 bytes)
// - 128 bytes is plenty
#define UART_TX_BUFFER_SIZE     128
#define UART_TX_BUFFER_MASK     (UART_TX_BUFFER_SIZE - 1)

//=============================================================================
// RING BUFFER STRUCTURES
//=============================================================================

// RX Ring Buffer (IRQ fills, main loop drains)
typedef struct {
    uint8_t data[UART_RX_BUFFER_SIZE] __attribute__((aligned(UART_RX_BUFFER_SIZE)));
    volatile uint16_t head;  // Write position (IRQ updates)
    volatile uint16_t tail;  // Read position (main loop updates)
} uart_rx_buffer_t;

// TX Ring Buffer (main loop fills, DMA/IRQ drains)
typedef struct {
    uint8_t data[UART_TX_BUFFER_SIZE] __attribute__((aligned(4)));
    volatile uint16_t head;  // Write position (main loop updates)
    volatile uint16_t tail;  // Read position (TX IRQ/DMA updates)
    volatile bool tx_active; // DMA/IRQ transmission in progress
} uart_tx_buffer_t;

//=============================================================================
// RING BUFFER API - RX (Lock-free, single producer/single consumer)
//=============================================================================

static inline void rx_buffer_init(uart_rx_buffer_t* buf) {
    buf->head = 0;
    buf->tail = 0;
}

// Called from UART RX IRQ - adds byte to buffer
static inline bool rx_buffer_put(uart_rx_buffer_t* buf, uint8_t byte) {
    uint16_t next_head = (buf->head + 1) & UART_RX_BUFFER_MASK;
    if (next_head == buf->tail) {
        return false;  // Buffer full, drop byte
    }
    buf->data[buf->head] = byte;
    buf->head = next_head;
    return true;
}

// Called from main loop - gets byte from buffer
static inline int rx_buffer_get(uart_rx_buffer_t* buf) {
    if (buf->head == buf->tail) {
        return -1;  // Buffer empty
    }
    uint8_t byte = buf->data[buf->tail];
    buf->tail = (buf->tail + 1) & UART_RX_BUFFER_MASK;
    return byte;
}

// Check if data available
static inline bool rx_buffer_available(const uart_rx_buffer_t* buf) {
    return buf->head != buf->tail;
}

// Get number of bytes available
static inline uint16_t rx_buffer_count(const uart_rx_buffer_t* buf) {
    return (buf->head - buf->tail) & UART_RX_BUFFER_MASK;
}

// Peek at next byte without removing it
static inline int rx_buffer_peek(const uart_rx_buffer_t* buf) {
    if (buf->head == buf->tail) {
        return -1;
    }
    return buf->data[buf->tail];
}

// Read multiple bytes (returns count read)
static inline uint16_t rx_buffer_read(uart_rx_buffer_t* buf, uint8_t* dest, uint16_t max_len) {
    uint16_t count = 0;
    while (count < max_len && buf->head != buf->tail) {
        dest[count++] = buf->data[buf->tail];
        buf->tail = (buf->tail + 1) & UART_RX_BUFFER_MASK;
    }
    return count;
}

//=============================================================================
// RING BUFFER API - TX (with blocking send option)
//=============================================================================

static inline void tx_buffer_init(uart_tx_buffer_t* buf) {
    buf->head = 0;
    buf->tail = 0;
    buf->tx_active = false;
}

// Add byte to TX buffer (non-blocking)
static inline bool tx_buffer_put(uart_tx_buffer_t* buf, uint8_t byte) {
    uint16_t next_head = (buf->head + 1) & UART_TX_BUFFER_MASK;
    if (next_head == buf->tail) {
        return false;  // Buffer full
    }
    buf->data[buf->head] = byte;
    buf->head = next_head;
    return true;
}

// Add multiple bytes to TX buffer
static inline uint16_t tx_buffer_write(uart_tx_buffer_t* buf, const uint8_t* src, uint16_t len) {
    uint16_t written = 0;
    while (written < len) {
        uint16_t next_head = (buf->head + 1) & UART_TX_BUFFER_MASK;
        if (next_head == buf->tail) {
            break;  // Buffer full
        }
        buf->data[buf->head] = src[written++];
        buf->head = next_head;
    }
    return written;
}

// Check if TX buffer has data
static inline bool tx_buffer_has_data(const uart_tx_buffer_t* buf) {
    return buf->head != buf->tail;
}

// Get next byte for transmission (called from TX IRQ)
static inline int tx_buffer_get(uart_tx_buffer_t* buf) {
    if (buf->head == buf->tail) {
        return -1;  // Empty
    }
    uint8_t byte = buf->data[buf->tail];
    buf->tail = (buf->tail + 1) & UART_TX_BUFFER_MASK;
    return byte;
}

//=============================================================================
// DIRECT UART SEND (No buffering, for guaranteed delivery)
//=============================================================================

// Blocking send - guaranteed to transmit all bytes
static inline void uart_send_blocking(uart_inst_t* uart, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        while (!uart_is_writable(uart)) {
            tight_loop_contents();
        }
        uart_putc_raw(uart, data[i]);
    }
}

// Blocking send string
static inline void uart_send_string_blocking(uart_inst_t* uart, const char* str) {
    while (*str) {
        while (!uart_is_writable(uart)) {
            tight_loop_contents();
        }
        uart_putc_raw(uart, *str++);
    }
}

#endif // UART_BUFFERS_H
