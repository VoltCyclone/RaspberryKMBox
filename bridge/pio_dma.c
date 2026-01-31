/**
 * PIO UART with DMA acceleration
 */

#include "pio_dma.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// Track RX buffer base addresses for position calculation
static uint8_t *rx_buffer_base[12] = {NULL};  // Max 12 DMA channels

// Initialize PIO UART TX with DMA
bool pio_uart_tx_dma_init(PIO pio, uint sm, uint pin, uint baud, int *dma_chan) {
    // Add PIO program and claim state machine
    uint offset = pio_add_program(pio, &uart_tx_program);
    uart_tx_program_init(pio, sm, offset, pin, baud);
    
    // Claim DMA channel
    *dma_chan = dma_claim_unused_channel(false);
    if (*dma_chan < 0) {
        return false;
    }
    
    // Configure DMA channel for PIO TX
    dma_channel_config c = dma_channel_get_default_config(*dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);   // Read from buffer
    channel_config_set_write_increment(&c, false); // Fixed write to PIO FIFO
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true)); // TX DREQ
    channel_config_set_high_priority(&c, true);
    
    // Apply config (don't start yet)
    dma_channel_configure(
        *dma_chan,
        &c,
        &pio->txf[sm],  // Write to PIO TX FIFO
        NULL,           // Source set per transfer
        0,              // Count set per transfer
        false           // Don't start
    );
    
    return true;
}

// Initialize PIO UART RX with DMA
bool pio_uart_rx_dma_init(PIO pio, uint sm, uint pin, uint baud, int *dma_chan,
                         uint8_t *buffer, size_t buffer_size) {
    // Add PIO program and claim state machine
    uint offset = pio_add_program(pio, &uart_rx_program);
    uart_rx_program_init(pio, sm, offset, pin, baud);
    
    // Claim DMA channel
    *dma_chan = dma_claim_unused_channel(false);
    if (*dma_chan < 0) {
        return false;
    }
    
    // Verify buffer size is power of 2
    if ((buffer_size & (buffer_size - 1)) != 0) {
        return false;
    }
    
    // Configure DMA channel for PIO RX with circular buffer
    dma_channel_config c = dma_channel_get_default_config(*dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);  // Fixed read from PIO FIFO
    channel_config_set_write_increment(&c, true);  // Write to buffer
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false)); // RX DREQ
    channel_config_set_ring(&c, true, __builtin_ctz(buffer_size)); // Circular buffer on write
    channel_config_set_high_priority(&c, true);
    
    // Apply config and start continuous transfer
    // NOTE: PIO RX FIFO with right-shift puts 8-bit data in bits 31:24 (MSB)
    // For 8-bit DMA reads, we read from rxf[sm] + 3 to get the MSB byte
    dma_channel_configure(
        *dma_chan,
        &c,
        buffer,                          // Write to circular buffer
        ((uint8_t*)&pio->rxf[sm]) + 3,   // Read from PIO RX FIFO MSB (where data lands)
        0xFFFFFFFF,                      // Transfer count (continuous)
        true                             // Start immediately
    );
    
    // Store buffer base for position calculation
    rx_buffer_base[*dma_chan] = buffer;
    
    return true;
}

// Send data via TX DMA (non-blocking if ready)
bool pio_uart_tx_dma_send(int dma_chan, PIO pio, uint sm, const uint8_t *data, size_t len) {
    // Check if DMA is busy
    if (dma_channel_is_busy(dma_chan)) {
        return false;
    }
    
    // Start transfer
    dma_channel_transfer_from_buffer_now(dma_chan, data, len);
    return true;
}

// Check if TX DMA is ready
bool pio_uart_tx_dma_ready(int dma_chan) {
    return !dma_channel_is_busy(dma_chan);
}

// Get RX DMA write position
uint32_t pio_uart_rx_dma_pos(int dma_chan, size_t buffer_size) {
    // Read the current write address from DMA
    uintptr_t current_addr = (uintptr_t)dma_channel_hw_addr(dma_chan)->write_addr;
    uintptr_t base_addr = (uintptr_t)rx_buffer_base[dma_chan];
    
    if (base_addr == 0) {
        return 0;  // Not initialized
    }
    
    // Calculate offset from base (wraps due to ring buffer)
    return (uint32_t)((current_addr - base_addr) & (buffer_size - 1));
}

// Abort TX DMA transfer
void pio_uart_tx_dma_abort(int dma_chan) {
    dma_channel_abort(dma_chan);
}
