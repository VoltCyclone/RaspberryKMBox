/**
 * FPGA SPI Communication Driver
 *
 * Provides high-level command interface to the iCE40 FPGA bridge engine
 * via PIO-based SPI master on PIO2 SM0. Replaces the UART driver with
 * full-duplex 8-byte SPI transactions at 12 MHz.
 *
 * After FPGA bitstream is loaded via CRAM (SPI0 on GPIO 4-7), those pins
 * are released from SPI0 and reconfigured as a PIO SPI master:
 *   GPIO 7 (MOSI) -> iCE40 pin 14   (was SPI0 TX / CRAM data)
 *   GPIO 4 (MISO) <- iCE40 pin 17   (was SPI0 RX)
 *   GPIO 6 (SCK)  -> iCE40 pin 15   (was SPI0 SCK)
 *   GPIO 5 (CS)   -> iCE40 pin 16   (was SPI0 CS, now software GPIO)
 *
 * Each SPI transaction exchanges 8 bytes simultaneously:
 *   MOSI: 8-byte command packet (opcode + payload)
 *   MISO: 8-byte status/response from FPGA
 *
 * The FPGA's spi_target module receives the 64-bit command on CS deassertion
 * and simultaneously shifts out the prepared 64-bit response. The bridge_engine
 * processes received commands and relays them to the KMBox via UART.
 *
 * Usage:
 *   1. Load FPGA bitstream via CRAM (ice_cram_open/write/close)
 *   2. Call fpga_spi_init() to switch GPIO 4-7 from SPI0 to PIO SPI
 *   3. Use fpga_send_*() convenience functions for common commands
 *   4. Use fpga_spi_transact() for custom 8-byte packets
 *   5. Use fpga_is_connected() to check KMBox connection status
 *   6. Use fpga_drain_rx() to read KMBox response bytes from FPGA
 */

#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Pin definitions (pico2-ice CRAM traces, reused for SPI after bitstream load)
#define FPGA_SPI_MOSI_PIN   7    // GPIO 7 -> iCE40 pin 14
#define FPGA_SPI_MISO_PIN   4    // GPIO 4 <- iCE40 pin 17
#define FPGA_SPI_SCK_PIN    6    // GPIO 6 -> iCE40 pin 15
#define FPGA_SPI_CS_PIN     5    // GPIO 5 -> iCE40 pin 16

#define FPGA_SPI_FREQ_HZ    12000000  // 12 MHz SPI clock

// PIO allocation
#define FPGA_SPI_PIO        pio2
#define FPGA_SPI_SM         0

// SPI command opcodes (matches bridge protocol / KMBox fast binary format)
// The FPGA relays all non-NOP bytes to KMBox; it does not interpret opcodes.
// The KMBox's process_fast_command() handles 0x01, 0xFE, etc.
#define FPGA_CMD_NOP             0x00  // No-op: FPGA does not relay (used for status poll)
#define FPGA_CMD_MOUSE_MOVE      0x01  // Mouse move (+optional buttons/wheel in payload)
#define FPGA_CMD_MOUSE_WHEEL     0x02  // Alias: maps to 0x01 with dx=0, dy=0, wheel field set
#define FPGA_CMD_BUTTON_SET      0x03  // Alias: maps to 0x01 with dx=0, dy=0, buttons field set
#define FPGA_CMD_PING            0xFE  // Keepalive ping
#define FPGA_CMD_RESET           0xFF  // Reset / clear state (treated as NOP by KMBox)

// MISO status byte bit positions (byte 0 of 8-byte FPGA response)
#define FPGA_STATUS_CONNECTED    (1 << 0)  // KMBox UART link is alive
#define FPGA_STATUS_TX_FULL      (1 << 1)  // FPGA->KMBox TX FIFO is full
#define FPGA_STATUS_RX_AVAIL     (1 << 2)  // KMBox response bytes available
#define FPGA_STATUS_UART_ERR     (1 << 3)  // UART framing error detected
#define FPGA_STATUS_CMD_OK       (1 << 4)  // Last command was accepted

// ============================================================================
// Response structure (parsed from 8-byte MISO data)
// ============================================================================

typedef struct {
    uint8_t status;       // Byte 0: status flags (FPGA_STATUS_*)
    uint8_t rx_count;     // Byte 1: number of valid bytes in rx_data[]
    uint8_t rx_data[4];   // Bytes 2-5: KMBox response data (up to 4 bytes per txn)
    uint8_t tx_free;      // Byte 6: free slots in FPGA->KMBox TX FIFO
    uint8_t seq;          // Byte 7: transaction sequence counter
} fpga_response_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize PIO SPI master for FPGA communication.
 * Deinits SPI0 (used for CRAM loading), releases GPIO 4-7, configures
 * PIO2 SM0 with the SPI program, and claims two DMA channels for
 * full-duplex 8-byte transfers.
 *
 * Must be called AFTER FPGA bitstream is loaded via CRAM.
 *
 * @return true if successful, false on PIO/DMA resource allocation failure
 */
bool fpga_spi_init(void);

// ============================================================================
// Low-level transaction
// ============================================================================

/**
 * Execute a single 8-byte SPI transaction with the FPGA.
 * Sends cmd[0..7] on MOSI while simultaneously receiving 8 bytes on MISO.
 * Uses DMA for both directions; blocks until RX DMA completes.
 *
 * @param cmd  8-byte command buffer to send
 * @return     Parsed response from FPGA MISO data
 */
fpga_response_t fpga_spi_transact(const uint8_t cmd[8]);

// ============================================================================
// Command convenience functions
// ============================================================================

/** Send a mouse move command (opcode 0x01). */
fpga_response_t fpga_send_mouse_move(int16_t dx, int16_t dy);

/** Send a mouse wheel command (opcode 0x01, dx=0, dy=0, wheel field set). */
fpga_response_t fpga_send_mouse_wheel(int8_t wheel);

/**
 * Send a button state command (opcode 0x01, dx=0, dy=0, buttons field set).
 * The buttons byte is the full HID button state (caller manages press/release).
 */
fpga_response_t fpga_send_button_set(uint8_t mask, uint8_t state);

/** Send a combined mouse move + wheel command (opcode 0x01, all fields). */
fpga_response_t fpga_send_mouse_move_wheel(int16_t dx, int16_t dy, int8_t wheel);

/** Send a keepalive ping (opcode 0xFE). */
fpga_response_t fpga_send_ping(void);

/** Send a reset command (opcode 0xFF). */
fpga_response_t fpga_send_reset(void);

// ============================================================================
// Status interface
// ============================================================================

/**
 * Read FPGA status without sending a command (sends NOP 0x00).
 * Useful for polling rx_data and status flags.
 */
fpga_response_t fpga_read_status(void);

/**
 * Check if KMBox is connected.
 * Returns bit 0 (FPGA_STATUS_CONNECTED) of the last cached response status.
 * Does NOT perform an SPI transaction; use fpga_read_status() to refresh.
 */
bool fpga_is_connected(void);

/**
 * Drain KMBox response bytes from FPGA.
 * Repeatedly sends NOP transactions to pull rx_data bytes until
 * rx_count == 0 or maxlen bytes have been read.
 *
 * @param buf     Destination buffer
 * @param maxlen  Maximum bytes to read
 * @return        Number of bytes actually read
 */
size_t fpga_drain_rx(uint8_t *buf, size_t maxlen);

#endif // FPGA_SPI_H
