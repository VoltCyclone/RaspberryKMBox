/**
 * PIO UART with DMA acceleration
 * 
 * Provides zero-overhead UART TX/RX using DMA transfers directly to/from PIO FIFOs.
 * Eliminates CPU involvement in byte-by-byte transfers for maximum throughput.
 */

#ifndef PIO_DMA_H
#define PIO_DMA_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include <stdbool.h>

/**
 * Initialize PIO UART TX with DMA support
 * 
 * @param pio PIO instance
 * @param sm State machine
 * @param pin TX pin
 * @param baud Baud rate
 * @param dma_chan Pointer to receive allocated DMA channel
 * @return true if successful
 */
bool pio_uart_tx_dma_init(PIO pio, uint sm, uint pin, uint baud, int *dma_chan);

/**
 * Initialize PIO UART RX with DMA support
 * 
 * @param pio PIO instance
 * @param sm State machine
 * @param pin RX pin
 * @param baud Baud rate
 * @param dma_chan Pointer to receive allocated DMA channel
 * @param buffer Circular RX buffer (must be power of 2)
 * @param buffer_size Size of RX buffer (must be power of 2)
 * @return true if successful
 */
bool pio_uart_rx_dma_init(PIO pio, uint sm, uint pin, uint baud, int *dma_chan, 
                         uint8_t *buffer, size_t buffer_size);

/**
 * Send data via DMA to PIO UART TX (non-blocking if space available)
 * 
 * @param dma_chan TX DMA channel
 * @param pio PIO instance
 * @param sm State machine
 * @param data Data to send
 * @param len Length of data
 * @return true if transfer started, false if DMA busy
 */
bool pio_uart_tx_dma_send(int dma_chan, PIO pio, uint sm, const uint8_t *data, size_t len);

/**
 * Check if TX DMA transfer is complete
 * 
 * @param dma_chan TX DMA channel
 * @return true if idle (ready for next transfer)
 */
bool pio_uart_tx_dma_ready(int dma_chan);

/**
 * Get current RX DMA write position in circular buffer
 * 
 * @param dma_chan RX DMA channel
 * @param buffer_size Size of circular buffer
 * @return Current write position
 */
uint32_t pio_uart_rx_dma_pos(int dma_chan, size_t buffer_size);

/**
 * Force abort TX DMA transfer (use with caution)
 * 
 * @param dma_chan TX DMA channel
 */
void pio_uart_tx_dma_abort(int dma_chan);

#endif // PIO_DMA_H
