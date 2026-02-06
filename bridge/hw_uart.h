/**
 * Hardware UART Driver for KMBox Bridge
 * 
 * High-performance hardware UART implementation with DMA support.
 * Replaces PIO UART for board-to-board communication after rewiring
 * TX/RX lines to be crossed at the hardware level.
 * 
 * Features:
 * - Hardware UART0 for reliable, high-speed communication
 * - DMA TX for non-blocking, zero-CPU transmission
 * - DMA RX with circular buffer for continuous reception
 * - IRQ-based fallback for compatibility
 */

#ifndef HW_UART_H
#define HW_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pico/types.h"

// DMA buffer sizes (must be power of 2 for ring buffer)
#define HW_UART_TX_BUFFER_SIZE  256
#define HW_UART_RX_BUFFER_SIZE  PICO_STDIO_STACK_BUFFER_SIZE

/**
 * Initialize hardware UART with DMA support
 * 
 * @param tx_pin TX GPIO pin
 * @param rx_pin RX GPIO pin  
 * @param baud Baud rate
 * @return true if successful
 */
bool hw_uart_init(uint tx_pin, uint rx_pin, uint baud);

/**
 * Send data via UART TX (non-blocking with DMA)
 * 
 * @param data Data to send
 * @param len Length of data
 * @return true if queued successfully
 */
bool hw_uart_send(const uint8_t *data, size_t len);

/**
 * Send a single byte (blocking if necessary)
 * 
 * @param c Byte to send
 */
void hw_uart_putc(uint8_t c);

/**
 * Send a null-terminated string
 * 
 * @param str String to send
 */
void hw_uart_puts(const char *str);

/**
 * Check if RX data is available
 * 
 * @return true if bytes available
 */
bool hw_uart_rx_available(void);

/**
 * Get number of bytes available in RX buffer
 * 
 * @return Number of bytes available
 */
size_t hw_uart_rx_count(void);

/**
 * Read a single byte from RX buffer (non-blocking)
 * 
 * @return Byte value, or -1 if empty
 */
int hw_uart_getc(void);

/**
 * Read multiple bytes from RX buffer
 * 
 * @param buf Destination buffer
 * @param maxlen Maximum bytes to read
 * @return Number of bytes read
 */
size_t hw_uart_read(uint8_t *buf, size_t maxlen);

/**
 * Check if TX is ready for more data
 * 
 * @return true if TX DMA is idle
 */
bool hw_uart_tx_ready(void);

/**
 * Wait for TX to complete
 */
void hw_uart_tx_wait(void);

/**
 * Get UART statistics
 * 
 * @param tx_bytes Total bytes transmitted
 * @param rx_bytes Total bytes received
 * @param rx_overflows Number of RX buffer overflows
 */
void hw_uart_get_stats(uint32_t *tx_bytes, uint32_t *rx_bytes, uint32_t *rx_overflows);

/**
 * Get UART RX IRQ count (debug)
 * 
 * @return Number of times RX IRQ handler was called
 */


#endif // HW_UART_H
