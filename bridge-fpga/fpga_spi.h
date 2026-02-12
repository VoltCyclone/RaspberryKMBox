/**
 * FPGA SPI Communication Driver
 *
 * Provides register-level read/write access to the iCE40 FPGA
 * via SPI, and high-level command submission for the bridge engine.
 *
 * The FPGA handles all UART communication with the KMBox device,
 * including auto-ping keepalive and protocol packet construction.
 *
 * Usage:
 *   1. Call fpga_spi_init() after FPGA bitstream is loaded
 *   2. Use fpga_send_mouse_move() etc. to submit commands
 *   3. Use fpga_poll_status() to read connection state
 *   4. Use fpga_drain_rx_fifo() to read KMBox responses
 */

#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * Initialize SPI master for FPGA register access.
 * Must be called after FPGA bitstream is loaded and running.
 * Uses spi0 (the same bus used for CRAM loading, but at higher speed).
 *
 * @return true if successful
 */
bool fpga_spi_init(void);

/**
 * Write a single register on the FPGA.
 * @param addr 7-bit register address
 * @param value 8-bit data value
 */
void fpga_reg_write(uint8_t addr, uint8_t value);

/**
 * Read a single register from the FPGA.
 * @param addr 7-bit register address
 * @return 8-bit data value
 */
uint8_t fpga_reg_read(uint8_t addr);

/**
 * Write multiple consecutive registers (burst write).
 * @param start_addr Starting register address
 * @param data Data buffer
 * @param len Number of bytes to write
 */
void fpga_reg_write_burst(uint8_t start_addr, const uint8_t *data, size_t len);

/**
 * Read multiple consecutive registers (burst read).
 * @param start_addr Starting register address
 * @param data Data buffer
 * @param len Number of bytes to read
 */
void fpga_reg_read_burst(uint8_t start_addr, uint8_t *data, size_t len);

// ============================================================================
// High-Level Command Interface
// ============================================================================

/**
 * Send a mouse move command via FPGA → UART → KMBox
 * @param x Relative X movement (signed 16-bit)
 * @param y Relative Y movement (signed 16-bit)
 * @return true if command was accepted (FPGA TX not busy)
 */
bool fpga_send_mouse_move(int16_t x, int16_t y);

/**
 * Send a mouse wheel command
 * @param wheel Wheel delta (signed 8-bit)
 * @return true if command was accepted
 */
bool fpga_send_mouse_wheel(int8_t wheel);

/**
 * Send a button set command
 * @param mask Button mask (which buttons to affect)
 * @param state Button state (0=release, 1=press)
 * @return true if command was accepted
 */
bool fpga_send_button_set(uint8_t mask, uint8_t state);

/**
 * Send a combined mouse move + wheel command
 * @param x Relative X movement
 * @param y Relative Y movement
 * @param wheel Wheel delta
 * @return true if command was accepted
 */
bool fpga_send_mouse_move_wheel(int16_t x, int16_t y, int8_t wheel);

/**
 * Send a ping command (normally auto-handled by FPGA)
 * @return true if command was accepted
 */
bool fpga_send_ping(void);

/**
 * Send a reset command to KMBox
 * @return true if command was accepted
 */
bool fpga_send_reset(void);

// ============================================================================
// Status Interface
// ============================================================================

/** FPGA bridge status snapshot */
typedef struct {
    bool     connected;      // KMBox is responding
    uint8_t  conn_state;     // Connection state byte
    uint16_t tx_pkt_count;   // Total TX packets sent
    uint16_t rx_pkt_count;   // Total RX packets received
    uint16_t rx_err_count;   // RX errors (framing, timeout)
    uint8_t  last_rx_cmd;    // Last received command/response byte
    uint8_t  fpga_version;   // FPGA RTL version
    bool     tx_busy;        // TX FSM is currently sending
    uint8_t  rx_fifo_count;  // Bytes waiting in RX FIFO
} fpga_bridge_status_t;

/**
 * Read all status registers from FPGA (single burst read).
 * @param status Output structure
 */
void fpga_poll_status(fpga_bridge_status_t *status);

/**
 * Quick check: is KMBox connected?
 * @return true if FPGA reports active UART communication
 */
bool fpga_is_connected(void);

/**
 * Read bytes from the FPGA RX FIFO (raw KMBox responses).
 * @param buf Destination buffer
 * @param maxlen Maximum bytes to read
 * @return Number of bytes actually read
 */
size_t fpga_drain_rx_fifo(uint8_t *buf, size_t maxlen);

#endif // FPGA_SPI_H
