/**
 * SPI Slave - FPGA-clocked packet reception for KMBox
 *
 * Receives 8-byte KMBox fast binary command packets from the iCE40 FPGA
 * SPI master. The FPGA drives SCK at 12 MHz; the RP2350's PIO2 samples
 * MOSI on rising edges and autopushes bytes into the RX FIFO. A DMA
 * channel drains 8 bytes per packet continuously.
 *
 * Data flow:
 *   FPGA SPI master → PIO2 SM0 (sample) → DMA → spi_rx_buf
 *   → DMA IRQ copies to spi_process_buf → main loop
 *   → process_fast_command() → mouse/keyboard/smooth injection
 *
 * Synchronization strategy:
 *   The PIO program samples MOSI on SCK rising edges with autopush at
 *   8 bits. Between SPI transactions, SCK idles low — PIO blocks at
 *   "wait 1 pin 1" with an empty ISR, naturally aligned for the next
 *   transaction. No CS_N GPIO IRQ is needed for framing.
 *
 *   DMA runs continuously: after each 8-byte transfer completes, the
 *   DMA IRQ handler copies the packet to a process buffer and
 *   immediately restarts DMA. The PIO RX FIFO (4 words = 16 bytes)
 *   buffers data during the ~200-500 ns DMA restart latency — well
 *   within the 10.67 µs budget (16 bytes at 12 MHz SPI).
 *
 * IRQ usage:
 *   DMA_IRQ_1: 8-byte transfer complete → copy packet + restart DMA
 */

#include "spi_slave.h"
#include "spi_slave.pio.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#include <string.h>
#include <stdio.h>

// process_fast_command() is defined in kmbox_serial_handler.c
// Must be made non-static for SPI slave integration
extern bool process_fast_command(const uint8_t *pkt);

// ============================================================================
// Module state
// ============================================================================

static bool spi_initialized = false;
static int spi_dma_chan = -1;
static uint spi_pio_offset;

// DMA writes into spi_rx_buf; ISR copies to spi_process_buf for main loop
static uint8_t spi_rx_buf[8] __attribute__((aligned(4)));
static uint8_t spi_process_buf[8] __attribute__((aligned(4)));

// Flag set by DMA IRQ, consumed by spi_slave_task()
static volatile bool spi_packet_ready = false;

// ============================================================================
// DMA IRQ handler: 8-byte transfer complete
//
// Copies the received packet to the process buffer, then immediately
// restarts DMA for the next packet. The PIO FIFO (16 bytes) absorbs
// any data arriving during the ~200-500 ns handler latency.
//
// Uses DMA_IRQ_1 to avoid conflicts with UART DMA on DMA_IRQ_0.
// ============================================================================

static void spi_dma_irq_handler(void) {
    if (dma_channel_get_irq1_status(spi_dma_chan)) {
        dma_channel_acknowledge_irq1(spi_dma_chan);

        // Copy completed packet to process buffer
        memcpy(spi_process_buf, spi_rx_buf, 8);
        spi_packet_ready = true;

        // Restart DMA immediately for next packet
        dma_channel_set_write_addr(spi_dma_chan, spi_rx_buf, false);
        dma_channel_set_trans_count(spi_dma_chan, 8, true);
    }
}

// ============================================================================
// Initialization
// ============================================================================

void spi_slave_init(void) {
    // Validate pin constraint: SCK must be MOSI + 1 for PIO wait instruction
    if (KM_SPI_SCK_PIN != KM_SPI_MOSI_PIN + 1) {
        printf("[SPI_SLAVE] ERROR: SCK (GPIO %d) must be MOSI (GPIO %d) + 1\n",
               KM_SPI_SCK_PIN, KM_SPI_MOSI_PIN);
        return;
    }

    // ----------------------------------------------------------------
    // Load PIO program into PIO2
    // ----------------------------------------------------------------
    if (!pio_can_add_program(KM_SPI_PIO, &spi_slave_rx_program)) {
        printf("[SPI_SLAVE] ERROR: No space in PIO2 for SPI slave program\n");
        return;
    }

    spi_pio_offset = pio_add_program(KM_SPI_PIO, &spi_slave_rx_program);

    spi_slave_rx_program_init(KM_SPI_PIO, KM_SPI_SM, spi_pio_offset,
                               KM_SPI_MOSI_PIN, KM_SPI_SCK_PIN);

    // ----------------------------------------------------------------
    // Configure CS_N as input with pull-up (idle high when disconnected)
    // Not used for IRQ — kept for diagnostic reads and pull-up bias
    // ----------------------------------------------------------------
    gpio_init(KM_SPI_CS_PIN);
    gpio_set_dir(KM_SPI_CS_PIN, GPIO_IN);
    gpio_pull_up(KM_SPI_CS_PIN);

    // ----------------------------------------------------------------
    // DMA channel: PIO2 RX FIFO → spi_rx_buf (8 bytes per transfer)
    // ----------------------------------------------------------------
    spi_dma_chan = dma_claim_unused_channel(false);
    if (spi_dma_chan < 0) {
        printf("[SPI_SLAVE] ERROR: No free DMA channel\n");
        return;
    }

    dma_channel_config cfg = dma_channel_get_default_config(spi_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);   // Read from fixed PIO RX FIFO addr
    channel_config_set_write_increment(&cfg, true);    // Write incrementally to buffer
    channel_config_set_dreq(&cfg, pio_get_dreq(KM_SPI_PIO, KM_SPI_SM, false));

    dma_channel_configure(spi_dma_chan, &cfg,
                          spi_rx_buf,                           // Write dest
                          &KM_SPI_PIO->rxf[KM_SPI_SM],        // Read src (PIO RX FIFO)
                          8,                                    // 8 bytes per transfer
                          false);                               // Don't start yet

    // DMA completion IRQ on DMA_IRQ_1 (DMA_IRQ_0 may be used by UART)
    dma_channel_set_irq1_enabled(spi_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, spi_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    // Clear any stale PIO data
    pio_sm_clear_fifos(KM_SPI_PIO, KM_SPI_SM);

    memset(spi_rx_buf, 0, sizeof(spi_rx_buf));
    memset(spi_process_buf, 0, sizeof(spi_process_buf));
    spi_packet_ready = false;
    spi_initialized = true;

    // Start DMA immediately — PIO naturally blocks at "wait 1 pin 1"
    // between transactions, so DMA won't receive spurious data
    dma_channel_set_trans_count(spi_dma_chan, 8, true);

    printf("[SPI_SLAVE] Ready (MOSI=GPIO%d, SCK=GPIO%d, CS=GPIO%d, DMA=%d)\n",
           KM_SPI_MOSI_PIN, KM_SPI_SCK_PIN, KM_SPI_CS_PIN, spi_dma_chan);
}

// ============================================================================
// Main loop task
// ============================================================================

void spi_slave_task(void) {
    if (!spi_packet_ready) return;
    spi_packet_ready = false;

    // Skip NOP packets (opcode 0x00 is status-read only on bridge side)
    if (spi_process_buf[0] == 0x00) return;

    // Forward to the existing fast command processor
    process_fast_command(spi_process_buf);
}
