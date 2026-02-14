/**
 * FPGA SPI Communication Driver Implementation
 *
 * After CRAM bitstream loading completes (using SPI0 on GPIO 4-7),
 * this module reconfigures those pins as a PIO-based SPI master for
 * full-duplex 8-byte transactions with the iCE40 FPGA bridge engine.
 *
 * PIO2 SM0 runs the fpga_spi program (Mode 0, MSB-first, autopull/push
 * at 32 bits). Two DMA channels transfer 32-bit words to/from PIO FIFOs.
 * 2 words × 32 bits = 64 bits per transaction.
 *
 * Each transaction:
 *   1. Assert CS (GPIO 5 low)
 *   2. Start TX and RX DMA (8 bytes each)
 *   3. Wait for RX DMA completion (blocking)
 *   4. Deassert CS (GPIO 5 high)
 *   5. Parse 8-byte MISO response into fpga_response_t
 */

#include "fpga_spi.h"
#include "fpga_spi.pio.h"  // Generated PIO header

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"   // For spi_deinit()
#include "hardware/clocks.h"

#include <string.h>
#include <stdio.h>

// ============================================================================
// Module state
// ============================================================================

static bool spi_initialized = false;

static int dma_tx_chan = -1;
static int dma_rx_chan = -1;

// Cached last response for fpga_is_connected() without a new transaction
static fpga_response_t last_response;

// PIO program offset (set during init)
static uint pio_offset;

// ============================================================================
// Initialization
// ============================================================================

bool fpga_spi_init(void) {
    // ----------------------------------------------------------------
    // Step 1: Deinit SPI0 — it was used for CRAM bitstream loading.
    // After ice_cram_close(), the FPGA is running and GPIO 4-7 become
    // available for reuse.
    // ----------------------------------------------------------------
    spi_deinit(spi0);

    // Release all four CRAM pins from SPI function
    gpio_set_function(FPGA_SPI_MISO_PIN, GPIO_FUNC_NULL);  // GPIO 4
    gpio_set_function(FPGA_SPI_CS_PIN,   GPIO_FUNC_NULL);  // GPIO 5
    gpio_set_function(FPGA_SPI_SCK_PIN,  GPIO_FUNC_NULL);  // GPIO 6
    gpio_set_function(FPGA_SPI_MOSI_PIN, GPIO_FUNC_NULL);  // GPIO 7

    // ----------------------------------------------------------------
    // Step 2: Configure CS as software-controlled GPIO output (idle high)
    // The PIO program does not manage CS — we assert/deassert it around
    // each DMA transfer so the FPGA sees clean transaction boundaries.
    // ----------------------------------------------------------------
    gpio_init(FPGA_SPI_CS_PIN);
    gpio_set_dir(FPGA_SPI_CS_PIN, GPIO_OUT);
    gpio_put(FPGA_SPI_CS_PIN, 1);  // Deasserted (idle high)

    // ----------------------------------------------------------------
    // Step 3: Load PIO program into PIO2 and configure SM0
    // The fpga_spi PIO program uses 2 instructions with side-set on SCK.
    // 4 PIO clocks per SPI bit: 2 instructions x [1] delay each.
    // ----------------------------------------------------------------
    if (!pio_can_add_program(FPGA_SPI_PIO, &fpga_spi_program)) {
        printf("[FPGA_SPI] ERROR: No space in PIO2 for SPI program\n");
        return false;
    }

    pio_offset = pio_add_program(FPGA_SPI_PIO, &fpga_spi_program);

    // Clock divider: PIO runs at 4x the SPI frequency (4 PIO clocks per bit)
    float clkdiv = (float)clock_get_hz(clk_sys) / (4.0f * FPGA_SPI_FREQ_HZ);

    printf("[FPGA_SPI] PIO clkdiv=%.2f (sys=%lu Hz, spi=%u Hz)\n",
           clkdiv, clock_get_hz(clk_sys), FPGA_SPI_FREQ_HZ);

    fpga_spi_program_init(FPGA_SPI_PIO, FPGA_SPI_SM, pio_offset,
                          FPGA_SPI_MOSI_PIN, FPGA_SPI_MISO_PIN,
                          FPGA_SPI_SCK_PIN, clkdiv);

    // ----------------------------------------------------------------
    // Step 4: Claim two DMA channels for TX and RX
    // TX: feeds command bytes into PIO TX FIFO (MOSI)
    // RX: drains response bytes from PIO RX FIFO (MISO)
    // ----------------------------------------------------------------
    dma_tx_chan = dma_claim_unused_channel(false);
    if (dma_tx_chan < 0) {
        printf("[FPGA_SPI] ERROR: No free DMA channel for TX\n");
        return false;
    }

    dma_rx_chan = dma_claim_unused_channel(false);
    if (dma_rx_chan < 0) {
        printf("[FPGA_SPI] ERROR: No free DMA channel for RX\n");
        dma_channel_unclaim(dma_tx_chan);
        dma_tx_chan = -1;
        return false;
    }

    // ---- TX DMA config ----
    // Read 32-bit words from memory, write to PIO TX FIFO.
    // Each 32-bit word holds 4 bytes packed big-endian (MSB in bits [31:24]).
    // The PIO shifts left (MSB first), so bit 31 goes out on MOSI first.
    // 2 transfers × 32 bits = 64 bits per transaction.
    dma_channel_config tx_cfg = dma_channel_get_default_config(dma_tx_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, pio_get_dreq(FPGA_SPI_PIO, FPGA_SPI_SM, true));

    dma_channel_configure(dma_tx_chan, &tx_cfg,
                          &FPGA_SPI_PIO->txf[FPGA_SPI_SM],  // Write to PIO TX FIFO
                          NULL,                               // Read address set per-transaction
                          2,                                  // 2 × 32-bit words = 8 bytes
                          false);                             // Don't start yet

    // ---- RX DMA config ----
    // Read 32-bit words from PIO RX FIFO, write to memory.
    // Each word contains 4 received bytes packed big-endian.
    // 2 transfers × 32 bits = 64 bits per transaction.
    dma_channel_config rx_cfg = dma_channel_get_default_config(dma_rx_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    channel_config_set_dreq(&rx_cfg, pio_get_dreq(FPGA_SPI_PIO, FPGA_SPI_SM, false));

    dma_channel_configure(dma_rx_chan, &rx_cfg,
                          NULL,                               // Write address set per-transaction
                          &FPGA_SPI_PIO->rxf[FPGA_SPI_SM],  // Read from PIO RX FIFO
                          2,                                  // 2 × 32-bit words = 8 bytes
                          false);                             // Don't start yet

    // Clear any stale data in PIO FIFOs
    pio_sm_clear_fifos(FPGA_SPI_PIO, FPGA_SPI_SM);

    memset(&last_response, 0, sizeof(last_response));
    spi_initialized = true;

    printf("[FPGA_SPI] Ready (MOSI=GPIO%d, MISO=GPIO%d, SCK=GPIO%d, CS=GPIO%d)\n",
           FPGA_SPI_MOSI_PIN, FPGA_SPI_MISO_PIN, FPGA_SPI_SCK_PIN, FPGA_SPI_CS_PIN);
    printf("[FPGA_SPI] DMA channels: TX=%d, RX=%d\n", dma_tx_chan, dma_rx_chan);
    return true;
}

// ============================================================================
// Response parsing
// ============================================================================

/**
 * Parse 8 raw MISO bytes into an fpga_response_t structure.
 *
 * Wire format (MSB-first, 8 bytes):
 *   [0] status     - FPGA_STATUS_* flags
 *   [1] rx_count   - Number of valid KMBox response bytes (0-4)
 *   [2] rx_data[0] - KMBox response byte 0
 *   [3] rx_data[1] - KMBox response byte 1
 *   [4] rx_data[2] - KMBox response byte 2
 *   [5] rx_data[3] - KMBox response byte 3
 *   [6] tx_free    - Free slots in FPGA->KMBox TX FIFO
 *   [7] seq        - Rolling transaction sequence counter
 */
static fpga_response_t parse_response(const uint8_t raw[8]) {
    fpga_response_t resp;
    resp.status   = raw[0];
    resp.rx_count = raw[1];
    resp.rx_data[0] = raw[2];
    resp.rx_data[1] = raw[3];
    resp.rx_data[2] = raw[4];
    resp.rx_data[3] = raw[5];
    resp.tx_free  = raw[6];
    resp.seq      = raw[7];
    return resp;
}

// ============================================================================
// Low-level SPI transaction
// ============================================================================

fpga_response_t fpga_spi_transact(const uint8_t cmd[8]) {
    uint8_t rx_buf[8] = {0};

    if (!spi_initialized) {
        return parse_response(rx_buf);
    }

    // Pack 8 command bytes into 2 big-endian 32-bit words.
    // PIO shifts left (MSB first), so bit 31 goes out first.
    // Each 32-bit word holds 4 bytes in network byte order.
    uint32_t tx_words[2] = {
        ((uint32_t)cmd[0] << 24) | ((uint32_t)cmd[1] << 16) |
        ((uint32_t)cmd[2] <<  8) | (uint32_t)cmd[3],
        ((uint32_t)cmd[4] << 24) | ((uint32_t)cmd[5] << 16) |
        ((uint32_t)cmd[6] <<  8) | (uint32_t)cmd[7]
    };
    uint32_t rx_words[2] = {0, 0};

    // Drain any stale data from PIO RX FIFO before starting
    pio_sm_clear_fifos(FPGA_SPI_PIO, FPGA_SPI_SM);

    // ---- Assert CS (active low) ----
    gpio_put(FPGA_SPI_CS_PIN, 0);

    // ---- Configure and start both DMA channels ----
    dma_channel_set_write_addr(dma_rx_chan, rx_words, false);
    dma_channel_set_trans_count(dma_rx_chan, 2, false);

    dma_channel_set_read_addr(dma_tx_chan, tx_words, false);
    dma_channel_set_trans_count(dma_tx_chan, 2, false);

    // Start RX first so it is ready to accept data as soon as PIO shifts in
    dma_channel_start(dma_rx_chan);
    dma_channel_start(dma_tx_chan);

    // ---- Wait for RX DMA to complete (all 64 bits received) ----
    dma_channel_wait_for_finish_blocking(dma_rx_chan);
    dma_channel_wait_for_finish_blocking(dma_tx_chan);

    // ---- Deassert CS (idle high) ----
    gpio_put(FPGA_SPI_CS_PIN, 1);

    // Unpack 2 big-endian 32-bit words into 8 response bytes
    rx_buf[0] = (rx_words[0] >> 24) & 0xFF;
    rx_buf[1] = (rx_words[0] >> 16) & 0xFF;
    rx_buf[2] = (rx_words[0] >>  8) & 0xFF;
    rx_buf[3] =  rx_words[0]        & 0xFF;
    rx_buf[4] = (rx_words[1] >> 24) & 0xFF;
    rx_buf[5] = (rx_words[1] >> 16) & 0xFF;
    rx_buf[6] = (rx_words[1] >>  8) & 0xFF;
    rx_buf[7] =  rx_words[1]        & 0xFF;

    // Parse and cache the response
    last_response = parse_response(rx_buf);
    return last_response;
}

// ============================================================================
// Command convenience functions
// ============================================================================

/**
 * Build an 8-byte command buffer with the given opcode and zeroed payload,
 * then fill in specific fields as needed.
 */
static inline void cmd_init(uint8_t buf[8], uint8_t opcode) {
    memset(buf, 0, 8);
    buf[0] = opcode;
}

fpga_response_t fpga_send_mouse_move(int16_t dx, int16_t dy) {
    // Format: [0x01, dx_lo, dx_hi, dy_lo, dy_hi, buttons, wheel, reserved]
    uint8_t cmd[8];
    cmd_init(cmd, FPGA_CMD_MOUSE_MOVE);
    cmd[1] = (uint8_t)(dx & 0xFF);
    cmd[2] = (uint8_t)((dx >> 8) & 0xFF);
    cmd[3] = (uint8_t)(dy & 0xFF);
    cmd[4] = (uint8_t)((dy >> 8) & 0xFF);
    cmd[5] = 0;  // buttons
    cmd[6] = 0;  // wheel
    cmd[7] = 0;  // reserved
    return fpga_spi_transact(cmd);
}

fpga_response_t fpga_send_mouse_wheel(int8_t wheel) {
    // Uses opcode 0x01 (mouse move) with dx=0, dy=0, wheel field set.
    // The KMBox's process_fast_command() reads the wheel byte from pkt[6].
    uint8_t cmd[8];
    cmd_init(cmd, FPGA_CMD_MOUSE_MOVE);
    cmd[1] = 0;  // dx_lo
    cmd[2] = 0;  // dx_hi
    cmd[3] = 0;  // dy_lo
    cmd[4] = 0;  // dy_hi
    cmd[5] = 0;  // buttons
    cmd[6] = (uint8_t)wheel;
    cmd[7] = 0;  // reserved
    return fpga_spi_transact(cmd);
}

fpga_response_t fpga_send_button_set(uint8_t mask, uint8_t state) {
    // Uses opcode 0x01 (mouse move) with dx=0, dy=0, buttons field set.
    // The KMBox's process_fast_command() reads button flags from pkt[5].
    // If state != 0 (press), set the bits from mask; if 0 (release), clear them.
    // Since the KMBox interprets pkt[5] as the full button state byte,
    // the caller should manage cumulative state. Here we apply mask+state
    // for simple single-call usage.
    uint8_t buttons = state ? mask : 0;
    uint8_t cmd[8];
    cmd_init(cmd, FPGA_CMD_MOUSE_MOVE);
    cmd[1] = 0;        // dx_lo
    cmd[2] = 0;        // dx_hi
    cmd[3] = 0;        // dy_lo
    cmd[4] = 0;        // dy_hi
    cmd[5] = buttons;  // button flags
    cmd[6] = 0;        // wheel
    cmd[7] = 0;        // reserved
    return fpga_spi_transact(cmd);
}

fpga_response_t fpga_send_mouse_move_wheel(int16_t dx, int16_t dy, int8_t wheel) {
    // Full mouse move packet with wheel field populated.
    uint8_t cmd[8];
    cmd_init(cmd, FPGA_CMD_MOUSE_MOVE);
    cmd[1] = (uint8_t)(dx & 0xFF);
    cmd[2] = (uint8_t)((dx >> 8) & 0xFF);
    cmd[3] = (uint8_t)(dy & 0xFF);
    cmd[4] = (uint8_t)((dy >> 8) & 0xFF);
    cmd[5] = 0;  // buttons
    cmd[6] = (uint8_t)wheel;
    cmd[7] = 0;  // reserved
    return fpga_spi_transact(cmd);
}

fpga_response_t fpga_send_ping(void) {
    uint8_t cmd[8];
    cmd_init(cmd, FPGA_CMD_PING);
    return fpga_spi_transact(cmd);
}

fpga_response_t fpga_send_reset(void) {
    uint8_t cmd[8];
    cmd_init(cmd, FPGA_CMD_RESET);
    return fpga_spi_transact(cmd);
}

// ============================================================================
// Status interface
// ============================================================================

fpga_response_t fpga_read_status(void) {
    // Send NOP (0x00) — FPGA does not relay NOP to KMBox, but still
    // returns the current status/response on MISO.
    uint8_t cmd[8];
    cmd_init(cmd, FPGA_CMD_NOP);
    return fpga_spi_transact(cmd);
}

bool fpga_is_connected(void) {
    return (last_response.status & FPGA_STATUS_CONNECTED) != 0;
}

size_t fpga_drain_rx(uint8_t *buf, size_t maxlen) {
    if (!spi_initialized || !buf || maxlen == 0) return 0;

    size_t total = 0;

    while (total < maxlen) {
        fpga_response_t resp = fpga_read_status();

        // No more data available from FPGA
        if (resp.rx_count == 0) break;

        // Clamp to what we can actually copy
        uint8_t count = resp.rx_count;
        if (count > 4) count = 4;  // Max 4 bytes per transaction

        size_t remaining = maxlen - total;
        if (count > remaining) count = (uint8_t)remaining;

        memcpy(buf + total, resp.rx_data, count);
        total += count;
    }

    return total;
}
