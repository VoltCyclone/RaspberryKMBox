/**
 * FPGA UART Communication Driver
 *
 * Provides high-level command interface to the iCE40 FPGA bridge engine
 * via UART1. After FPGA bitstream is loaded via SPI (CRAM), the SPI
 * pins (GPIO 4/5) are reconfigured as UART1 for protocol communication.
 *
 * The FPGA relays bridge protocol packets between RP2350 and KMBox:
 *   RP2350 UART1 TX (GPIO 4) → FPGA → KMBox UART TX
 *   KMBox UART RX → FPGA → RP2350 UART1 RX (GPIO 5)
 *
 * Usage:
 *   1. Load FPGA bitstream via CRAM (ice_cram_open/write/close)
 *   2. Call fpga_uart_init() to switch GPIO 4/5 from SPI to UART1
 *   3. Use fpga_send_*() to submit commands (builds bridge protocol packets)
 *   4. Use fpga_read_rx() to read KMBox responses
 *   5. Use fpga_is_connected() to check KMBox connection status via GPIO 7
 */

#ifndef FPGA_UART_H
#define FPGA_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Pin definitions
#define FPGA_UART_INST      uart1
#define FPGA_UART_TX_PIN    4       // RP2350 GPIO 4 → iCE40 pin 17
#define FPGA_UART_RX_PIN    5       // RP2350 GPIO 5 ← iCE40 pin 16
#define FPGA_UART_BAUD      3000000 // 3 Mbaud (exact at 48 MHz peripheral clock)
#define FPGA_CONNECTED_PIN  7       // RP2350 GPIO 7 ← iCE40 pin 14

/**
 * Initialize UART1 for FPGA communication.
 * Reconfigures GPIO 4/5 from SPI to UART1 and sets up GPIO 7 as
 * connected status input. Must be called AFTER FPGA bitstream is loaded.
 *
 * @return true if successful
 */
bool fpga_uart_init(void);

// ============================================================================
// Command Interface (builds bridge protocol packets and sends via UART1)
// ============================================================================

/**
 * Send a mouse move command.
 * Builds bridge protocol packet [0xBD, 0x01, x_lo, x_hi, y_lo, y_hi]
 * and sends via UART1 → FPGA → KMBox.
 */
bool fpga_send_mouse_move(int16_t x, int16_t y);

/** Send a mouse wheel command. */
bool fpga_send_mouse_wheel(int8_t wheel);

/** Send a button set command. */
bool fpga_send_button_set(uint8_t mask, uint8_t state);

/** Send a combined mouse move + wheel command. */
bool fpga_send_mouse_move_wheel(int16_t x, int16_t y, int8_t wheel);

/** Send a ping command (normally auto-handled by FPGA). */
bool fpga_send_ping(void);

/** Send a reset command to KMBox. */
bool fpga_send_reset(void);

// ============================================================================
// Status Interface
// ============================================================================

/**
 * Non-blocking read of KMBox response bytes from UART1 RX.
 * Returns bytes forwarded by FPGA from KMBox UART.
 *
 * @param buf    Destination buffer
 * @param maxlen Maximum bytes to read
 * @return Number of bytes actually read (0 if none available)
 */
size_t fpga_read_rx(uint8_t *buf, size_t maxlen);

/**
 * Check if KMBox is connected.
 * Reads the FPGA connected status output on GPIO 7.
 * High = connected (FPGA received KMBox response within 5s).
 */
bool fpga_is_connected(void);

#endif // FPGA_UART_H
