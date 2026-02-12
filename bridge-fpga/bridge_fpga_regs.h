/**
 * Bridge FPGA Register Map - C Header
 *
 * Mirror of rtl/bridge_regs.vh for the RP2350 firmware.
 * Keep in sync with the Verilog definitions.
 */

#ifndef BRIDGE_FPGA_REGS_H
#define BRIDGE_FPGA_REGS_H

#include <stdint.h>

// --- Status Registers (Read-Only) ---
#define FPGA_REG_STATUS         0x00  // [0]=connected
#define FPGA_REG_CONN_STATE     0x01  // Connection state: 0=disconn, 1=conn
#define FPGA_REG_TX_COUNT_LO    0x02  // TX packet count low byte
#define FPGA_REG_TX_COUNT_HI    0x03  // TX packet count high byte
#define FPGA_REG_RX_COUNT_LO    0x04  // RX packet count low byte
#define FPGA_REG_RX_COUNT_HI    0x05  // RX packet count high byte
#define FPGA_REG_RX_ERR_LO     0x06  // RX error count low byte
#define FPGA_REG_RX_ERR_HI     0x07  // RX error count high byte
#define FPGA_REG_LAST_RX_CMD   0x08  // Last received command byte
#define FPGA_REG_VERSION       0x09  // RTL version number
#define FPGA_REG_TX_BUSY       0x0A  // [0]=TX FSM busy

// --- Command Registers (Write) ---
#define FPGA_REG_CMD_MOUSE_XL  0x10  // Mouse X low byte (i16)
#define FPGA_REG_CMD_MOUSE_XH  0x11  // Mouse X high byte
#define FPGA_REG_CMD_MOUSE_YL  0x12  // Mouse Y low byte (i16)
#define FPGA_REG_CMD_MOUSE_YH  0x13  // Mouse Y high byte
#define FPGA_REG_CMD_WHEEL     0x14  // Wheel delta (i8)
#define FPGA_REG_CMD_BTN_MASK  0x15  // Button mask (u8)
#define FPGA_REG_CMD_BTN_STATE 0x16  // Button state (u8)
#define FPGA_REG_CMD_TYPE      0x17  // Command type → triggers send

// --- RX FIFO Registers ---
#define FPGA_REG_RX_FIFO_CNT  0x20  // Bytes available in RX FIFO
#define FPGA_REG_RX_FIFO_DATA 0x21  // Read pops one byte from RX FIFO

// --- Protocol Command Types (same as bridge_protocol.h) ---
#define FPGA_CMD_MOUSE_MOVE       0x01
#define FPGA_CMD_MOUSE_WHEEL      0x02
#define FPGA_CMD_BUTTON_SET       0x03
#define FPGA_CMD_MOUSE_MOVE_WHEEL 0x04
#define FPGA_CMD_PING             0xFE
#define FPGA_CMD_RESET            0xFF

// --- SPI Transaction Helpers ---
// First byte: [R/W(1)] [ADDR(7)]
// R/W: 1=read, 0=write
#define FPGA_SPI_READ(addr)   (0x80 | ((addr) & 0x7F))
#define FPGA_SPI_WRITE(addr)  (0x00 | ((addr) & 0x7F))

#endif // BRIDGE_FPGA_REGS_H
