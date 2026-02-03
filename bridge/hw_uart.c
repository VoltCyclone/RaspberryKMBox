/**
 * Hardware UART Driver for KMBox Bridge
 * 
 * High-performance hardware UART implementation with DMA support.
 * 
 * Clock Configuration for Overclocking:
 * When CPU is overclocked above 133MHz, clk_peri is switched to 48MHz USB PLL
 * to ensure stable UART timing. At 240MHz system clock with 240MHz clk_peri,
 * UART baudrate divisors become fractional causing timing drift.
 * 
 * At 48MHz: 48000000 / 921600 = 52.08 (acceptable error)
 * At 240MHz: 240000000 / 921600 = 260.4 (excessive error)
 */

#include "hw_uart.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/uart.h"
#include <stdio.h>
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

/**
 * Configure peripheral clock for stable UART operation
 * 
 * After set_sys_clock_khz() is called, clk_peri is automatically attached to
 * USB PLL (48MHz). For maximum UART performance, we attach it directly to the
 * system PLL instead, keeping it at the full system clock speed.
 * 
 * CRITICAL: Must be called AFTER set_sys_clock_khz() and BEFORE uart_init()!
 */
static void configure_stable_peri_clock(void) {
    uint32_t peri_freq_before = clock_get_hz(clk_peri);
    uint32_t sys_freq = clock_get_hz(clk_sys);
    
    printf("[HW_UART] clk_sys=%luHz, clk_peri=%luHz (before)\n", sys_freq, peri_freq_before);
    
    // If overclocked, switch clk_peri to stable 48MHz USB PLL
    // This avoids fractional baudrate divisors that cause timing drift
    if (sys_freq > 133000000) {
        printf("[HW_UART] Overclock detected! Switching clk_peri to 48MHz USB PLL\n");
        clock_configure(
            clk_peri,
            0,                                                // No glitchless mux
            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, // USB PLL (48MHz)
            48000000,                                         // 48MHz input
            48000000                                          // 48MHz output
        );
    }
    
    uint32_t peri_freq_after = clock_get_hz(clk_peri);
    printf("[HW_UART] clk_peri now %luHz (%.2f MHz)\n", peri_freq_after, peri_freq_after / 1000000.0f);
}

bool hw_uart_init(uint tx_pin, uint rx_pin, uint baud) {
    // CRITICAL: Configure stable peripheral clock BEFORE UART init
    // This must happen first so uart_init() calculates correct divisor
    configure_stable_peri_clock();
    
    // Log the peripheral clock frequency that uart_init will use for baud calculation
    uint32_t peri_freq = clock_get_hz(clk_peri);
    printf("[HW_UART] Using clk_peri: %lu Hz (%.2f MHz) for UART baud calculation\n", 
           peri_freq, peri_freq / 1000000.0f);
    
    // Set up GPIO pins with correct UART function select BEFORE uart_init
    // CRITICAL: Must use UART_FUNCSEL_NUM to get correct function select
    // for this specific UART instance and pin combination
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(HW_UART, tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(HW_UART, rx_pin));
    gpio_pull_up(rx_pin);
    
    // Initialize UART - this automatically enables FIFOs and sets 8N1 format
    uint actual_baud = uart_init(HW_UART, baud);
    
    // Calculate and display baud rate accuracy
    int32_t baud_error_ppm = ((int32_t)actual_baud - (int32_t)baud) * 1000000 / (int32_t)baud;
    printf("[HW_UART] Requested: %u, Actual: %u, Error: %ld ppm\n", baud, actual_baud, baud_error_ppm);
    
    // Warn if baud rate error is too high (>2% is problematic)
    if (baud_error_ppm > 20000 || baud_error_ppm < -20000) {
        printf("[HW_UART] WARNING: Baud rate error >2%%, communication may be unreliable!\n");
    }
    
    // Set RX FIFO trigger to 1 byte (1/8 threshold) for responsive DMA
    uart_hw_t *uart_hw = uart_get_hw(HW_UART);
    uart_hw->ifls = (uart_hw->ifls & ~UART_UARTIFLS_RXIFLSEL_BITS) | (0 << UART_UARTIFLS_RXIFLSEL_LSB);
    
    // Disable flow control
    uart_set_hw_flow(HW_UART, false, false);
    
    // Disable UART interrupts (we use DMA)
    uart_set_irq_enables(HW_UART, false, false);
    
    // Drain any garbage
    while (uart_is_readable(HW_UART)) {
        uart_getc(HW_UART);
    }
    
    // Claim TX DMA channel
    tx_dma_chan = dma_claim_unused_channel(false);
    if (tx_dma_chan < 0) {
        printf("[HW_UART] Failed to claim TX DMA channel\n");
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
        NULL,
        0,
        false
    );
    
    // Set up TX DMA interrupt
    dma_channel_set_irq0_enabled(tx_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, tx_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // Claim RX DMA channel
    rx_dma_chan = dma_claim_unused_channel(false);
    if (rx_dma_chan < 0) {
        printf("[HW_UART] Failed to claim RX DMA channel\n");
        dma_channel_unclaim(tx_dma_chan);
        tx_dma_chan = -1;
        return false;
    }
    
    // Configure RX DMA with circular buffer
    dma_channel_config rx_cfg = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    
    uint dreq_num = uart_get_dreq(HW_UART, false);
    channel_config_set_dreq(&rx_cfg, dreq_num);
    
    uint ring_size_bits = __builtin_ctz(HW_UART_RX_BUFFER_SIZE);
    channel_config_set_ring(&rx_cfg, true, ring_size_bits);
    channel_config_set_high_priority(&rx_cfg, true);
    
    printf("[HW_UART] RX DMA: ch=%d dreq=%u ring_bits=%u buf_size=%u\n", 
           rx_dma_chan, dreq_num, ring_size_bits, HW_UART_RX_BUFFER_SIZE);
    
    // Print UART hardware register addresses for debugging
    printf("[HW_UART] UART base=%p DR=%p\n", uart_hw, &uart_hw->dr);
    
    // Start continuous RX DMA
    dma_channel_configure(
        rx_dma_chan,
        &rx_cfg,
        rx_buffer,
        &uart_hw->dr,  // Read from UART data register
        0xFFFFFFFF,
        true
    );
    
    // Verify DMA is actually configured and running
    printf("[HW_UART] DMA configured: read_addr=%p write_addr=%p count=%lu\n",
           (void*)dma_channel_hw_addr(rx_dma_chan)->read_addr,
           (void*)dma_channel_hw_addr(rx_dma_chan)->write_addr,
           dma_channel_hw_addr(rx_dma_chan)->transfer_count);
    
    printf("[HW_UART] Init complete: TX_DMA=%d, RX_DMA=%d\n", tx_dma_chan, rx_dma_chan);
    
    return true;
}

bool hw_uart_send(const uint8_t *data, size_t len) {
    if (len == 0 || len > HW_UART_TX_BUFFER_SIZE) {
        return false;
    }
    
    if (tx_dma_chan < 0) {
        for (size_t i = 0; i < len; i++) {
            uart_putc_raw(HW_UART, data[i]);
        }
        total_tx_bytes += len;
        return true;
    }
    
    while (tx_busy) {
        tight_loop_contents();
    }
    
    uint8_t *buf = using_buffer_a ? tx_buffer_a : tx_buffer_b;
    memcpy(buf, data, len);
    using_buffer_a = !using_buffer_a;
    
    tx_busy = true;
    dma_channel_set_read_addr(tx_dma_chan, buf, false);
    dma_channel_set_trans_count(tx_dma_chan, len, true);
    
    total_tx_bytes += len;
    return true;
}

void hw_uart_putc(uint8_t c) {
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
        while (*str) {
            hw_uart_putc(*str++);
        }
    }
}

static inline uint32_t get_rx_write_pos(void) {
    if (rx_dma_chan < 0) return 0;
    uintptr_t write_addr = (uintptr_t)dma_channel_hw_addr(rx_dma_chan)->write_addr;
    uintptr_t base_addr = (uintptr_t)rx_buffer;
    return (uint32_t)((write_addr - base_addr) & (HW_UART_RX_BUFFER_SIZE - 1));
}

bool hw_uart_rx_available(void) {
    uint32_t write_pos = get_rx_write_pos();
    bool has_data = (write_pos != rx_read_pos);
    
    static uint32_t last_debug = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_debug > 5000) {
        bool dma_busy = (rx_dma_chan >= 0) ? dma_channel_is_busy(rx_dma_chan) : false;
        
        // Get DMA transfer count to see if it's actually transferring
        uint32_t transfer_count = 0;
        if (rx_dma_chan >= 0) {
            transfer_count = dma_channel_hw_addr(rx_dma_chan)->transfer_count;
        }
        
        // Check if UART has data in hardware FIFO
        bool uart_readable = uart_is_readable(HW_UART);
        
        printf("[RX] wr=%u rd=%u has_data=%d dma_busy=%d xfer_count=%u uart_readable=%d total=%u\n", 
               write_pos, (uint32_t)rx_read_pos, has_data, dma_busy, transfer_count, uart_readable, total_rx_bytes);
        last_debug = now;
    }
    
    return has_data;
}

size_t hw_uart_rx_count(void) {
    uint32_t write_pos = get_rx_write_pos();
    size_t available = (write_pos - rx_read_pos) & (HW_UART_RX_BUFFER_SIZE - 1);
    if (available >= HW_UART_RX_BUFFER_SIZE - 16) {
        rx_overflows++;
    }
    return available;
}

int hw_uart_getc(void) {
    uint32_t write_pos = get_rx_write_pos();
    if (write_pos == rx_read_pos) {
        return -1;
    }
    
    uint8_t c = rx_buffer[rx_read_pos];
    rx_read_pos = (rx_read_pos + 1) & (HW_UART_RX_BUFFER_SIZE - 1);
    total_rx_bytes++;
    
    // Flash neopixel red on RX for debug (brief flash)
    extern void ws2812_put_rgb(uint8_t r, uint8_t g, uint8_t b);
    static uint32_t rx_flash_count = 0;
    if ((rx_flash_count++ & 0x1F) == 0) {  // Flash every 32 bytes to avoid overwhelming
        ws2812_put_rgb(255, 0, 0);  // Red flash for RX
    } else if ((rx_flash_count & 0x1F) == 1) {
        ws2812_put_rgb(0, 0, 0);  // Turn off after flash
    }
    
    return c;
}

size_t hw_uart_read(uint8_t *buf, size_t maxlen) {
    if (maxlen == 0) return 0;
    
    uint32_t write_pos = get_rx_write_pos();
    size_t available = (write_pos - rx_read_pos) & (HW_UART_RX_BUFFER_SIZE - 1);
    if (available == 0) return 0;
    
    size_t to_read = (available < maxlen) ? available : maxlen;
    
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
