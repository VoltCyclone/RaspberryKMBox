/**
 * Hardware UART Driver for KMBox Bridge
 * 
 * High-performance hardware UART implementation with DMA support.
 * Uses UART0 with DMA for both TX and RX operations.
 */

#include "hw_uart.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include <string.h>

// UART instance
#define HW_UART uart0

// DMA channels
static int tx_dma_chan = -1;
static int rx_dma_chan = -1;

// TX double-buffer for continuous transmission
static uint8_t tx_buffer_a[HW_UART_TX_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t tx_buffer_b[HW_UART_TX_BUFFER_SIZE] __attribute__((aligned(4)));
static volatile bool using_buffer_a = true;
static volatile bool tx_busy = false;
static volatile size_t tx_pending_len = 0;

// RX circular buffer (must be aligned to size for DMA ring mode)
static uint8_t rx_buffer[HW_UART_RX_BUFFER_SIZE] __attribute__((aligned(HW_UART_RX_BUFFER_SIZE)));
static volatile uint32_t rx_read_pos = 0;

// Statistics
static volatile uint32_t total_tx_bytes = 0;
static volatile uint32_t total_rx_bytes = 0;
static volatile uint32_t rx_overflows = 0;

// IRQ handler for TX DMA completion
static void tx_dma_irq_handler(void) {
    if (tx_dma_chan >= 0 && dma_channel_get_irq0_status(tx_dma_chan)) {
        dma_channel_acknowledge_irq0(tx_dma_chan);
        tx_busy = false;
    }
}

bool hw_uart_init(uint tx_pin, uint rx_pin, uint baud) {
    // Initialize UART
    uart_init(HW_UART, baud);
    
    // Configure format: 8N1
    uart_set_format(HW_UART, 8, 1, UART_PARITY_NONE);
    
    // Enable FIFOs for better throughput
    uart_set_fifo_enabled(HW_UART, true);
    
    // Disable flow control
    uart_set_hw_flow(HW_UART, false, false);
    
    // Set up GPIO pins
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin);  // Prevent floating when disconnected
    
    // Claim TX DMA channel
    tx_dma_chan = dma_claim_unused_channel(false);
    if (tx_dma_chan < 0) {
        return false;
    }
    
    // Configure TX DMA
    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, uart_get_dreq(HW_UART, true));
    channel_config_set_high_priority(&tx_cfg, true);
    
    dma_channel_configure(
        tx_dma_chan,
        &tx_cfg,
        &uart_get_hw(HW_UART)->dr,
        NULL,  // Set per transfer
        0,     // Set per transfer
        false  // Don't start
    );
    
    // Set up TX DMA interrupt
    dma_channel_set_irq0_enabled(tx_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, tx_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // Claim RX DMA channel
    rx_dma_chan = dma_claim_unused_channel(false);
    if (rx_dma_chan < 0) {
        dma_channel_unclaim(tx_dma_chan);
        tx_dma_chan = -1;
        return false;
    }
    
    // Configure RX DMA with circular buffer
    dma_channel_config rx_cfg = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    channel_config_set_dreq(&rx_cfg, uart_get_dreq(HW_UART, false));
    channel_config_set_ring(&rx_cfg, true, __builtin_ctz(HW_UART_RX_BUFFER_SIZE));
    channel_config_set_high_priority(&rx_cfg, true);
    
    // Start continuous RX DMA
    dma_channel_configure(
        rx_dma_chan,
        &rx_cfg,
        rx_buffer,
        &uart_get_hw(HW_UART)->dr,
        0xFFFFFFFF,  // Continuous transfer
        true         // Start now
    );
    
    return true;
}

bool hw_uart_send(const uint8_t *data, size_t len) {
    if (len == 0 || len > HW_UART_TX_BUFFER_SIZE) {
        return false;
    }
    
    // If DMA not available, use blocking send
    if (tx_dma_chan < 0) {
        for (size_t i = 0; i < len; i++) {
            uart_putc_raw(HW_UART, data[i]);
        }
        total_tx_bytes += len;
        return true;
    }
    
    // Wait for previous DMA to complete
    while (tx_busy) {
        tight_loop_contents();
    }
    
    // Copy to inactive buffer
    uint8_t *buf = using_buffer_a ? tx_buffer_a : tx_buffer_b;
    memcpy(buf, data, len);
    using_buffer_a = !using_buffer_a;
    
    // Start DMA transfer
    tx_busy = true;
    dma_channel_set_read_addr(tx_dma_chan, buf, false);
    dma_channel_set_trans_count(tx_dma_chan, len, true);
    
    total_tx_bytes += len;
    return true;
}

void hw_uart_putc(uint8_t c) {
    // For single bytes, direct write is often faster than DMA overhead
    while (!uart_is_writable(HW_UART)) {
        tight_loop_contents();
    }
    uart_putc_raw(HW_UART, c);
    total_tx_bytes++;
}

void hw_uart_puts(const char *str) {
    size_t len = strlen(str);
    if (len <= HW_UART_TX_BUFFER_SIZE) {
        hw_uart_send((const uint8_t *)str, len);
    } else {
        // Fall back to per-character for long strings
        while (*str) {
            hw_uart_putc(*str++);
        }
    }
}

// Get current DMA write position in circular buffer
static inline uint32_t get_rx_write_pos(void) {
    if (rx_dma_chan < 0) return 0;
    
    uintptr_t write_addr = (uintptr_t)dma_channel_hw_addr(rx_dma_chan)->write_addr;
    uintptr_t base_addr = (uintptr_t)rx_buffer;
    return (uint32_t)((write_addr - base_addr) & (HW_UART_RX_BUFFER_SIZE - 1));
}

bool hw_uart_rx_available(void) {
    return get_rx_write_pos() != rx_read_pos;
}

size_t hw_uart_rx_count(void) {
    uint32_t write_pos = get_rx_write_pos();
    return (write_pos - rx_read_pos) & (HW_UART_RX_BUFFER_SIZE - 1);
}

int hw_uart_getc(void) {
    uint32_t write_pos = get_rx_write_pos();
    if (write_pos == rx_read_pos) {
        return -1;  // Buffer empty
    }
    
    uint8_t c = rx_buffer[rx_read_pos];
    rx_read_pos = (rx_read_pos + 1) & (HW_UART_RX_BUFFER_SIZE - 1);
    total_rx_bytes++;
    return c;
}

size_t hw_uart_read(uint8_t *buf, size_t maxlen) {
    if (maxlen == 0) return 0;
    
    uint32_t write_pos = get_rx_write_pos();
    size_t available = (write_pos - rx_read_pos) & (HW_UART_RX_BUFFER_SIZE - 1);
    
    if (available == 0) return 0;
    
    size_t to_read = (available < maxlen) ? available : maxlen;
    
    // Copy from circular buffer, handling wrap-around
    size_t first_chunk = HW_UART_RX_BUFFER_SIZE - rx_read_pos;
    if (first_chunk > to_read) first_chunk = to_read;
    
    memcpy(buf, &rx_buffer[rx_read_pos], first_chunk);
    
    if (to_read > first_chunk) {
        memcpy(buf + first_chunk, rx_buffer, to_read - first_chunk);
    }
    
    rx_read_pos = (rx_read_pos + to_read) & (HW_UART_RX_BUFFER_SIZE - 1);
    total_rx_bytes += to_read;
    
    return to_read;
}

bool hw_uart_tx_ready(void) {
    return !tx_busy;
}

void hw_uart_tx_wait(void) {
    while (tx_busy) {
        tight_loop_contents();
    }
}

void hw_uart_get_stats(uint32_t *tx_bytes, uint32_t *rx_bytes, uint32_t *overflows) {
    if (tx_bytes) *tx_bytes = total_tx_bytes;
    if (rx_bytes) *rx_bytes = total_rx_bytes;
    if (overflows) *overflows = rx_overflows;
}
