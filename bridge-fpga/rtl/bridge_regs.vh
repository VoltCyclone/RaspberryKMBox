/**
 * Bridge Register Map Definitions
 *
 * Shared between Verilog RTL and C firmware via `include / #include
 *
 * Register addresses (7-bit, 0x00-0x7F):
 *   0x00-0x0F: Status / read-only registers
 *   0x10-0x1F: Command write registers
 *   0x20-0x2F: RX FIFO interface
 *   0x30-0x3F: Configuration
 */

// --- Status Registers (Read-Only) ---
localparam REG_STATUS       = 7'h00;  // [0]=connected
localparam REG_CONN_STATE   = 7'h01;  // Connection state: 0=disconn, 1=conn
localparam REG_TX_COUNT_LO  = 7'h02;  // TX packet count low byte
localparam REG_TX_COUNT_HI  = 7'h03;  // TX packet count high byte
localparam REG_RX_COUNT_LO  = 7'h04;  // RX packet count low byte
localparam REG_RX_COUNT_HI  = 7'h05;  // RX packet count high byte
localparam REG_RX_ERR_LO    = 7'h06;  // RX error count low byte
localparam REG_RX_ERR_HI    = 7'h07;  // RX error count high byte
localparam REG_LAST_RX_CMD  = 7'h08;  // Last received command byte
localparam REG_VERSION      = 7'h09;  // RTL version number
localparam REG_TX_BUSY      = 7'h0A;  // [0]=TX busy

// --- Command Registers (Write) ---
localparam REG_CMD_MOUSE_XL = 7'h10;  // Mouse X low byte (i16)
localparam REG_CMD_MOUSE_XH = 7'h11;  // Mouse X high byte
localparam REG_CMD_MOUSE_YL = 7'h12;  // Mouse Y low byte (i16)
localparam REG_CMD_MOUSE_YH = 7'h13;  // Mouse Y high byte
localparam REG_CMD_WHEEL    = 7'h14;  // Wheel delta (i8)
localparam REG_CMD_BTN_MASK = 7'h15;  // Button mask (u8)
localparam REG_CMD_BTN_STATE = 7'h16; // Button state (u8)
localparam REG_CMD_TYPE     = 7'h17;  // Command type → triggers send (u8)

// --- RX FIFO Registers ---
localparam REG_RX_FIFO_CNT  = 7'h20; // Bytes available in RX FIFO
localparam REG_RX_FIFO_DATA = 7'h21; // Read pops one byte from RX FIFO

// --- Protocol Constants ---
localparam [7:0] SYNC_BYTE          = 8'hBD;
localparam [7:0] CMD_MOUSE_MOVE     = 8'h01;
localparam [7:0] CMD_MOUSE_WHEEL    = 8'h02;
localparam [7:0] CMD_BUTTON_SET     = 8'h03;
localparam [7:0] CMD_MOUSE_MOVE_WHEEL = 8'h04;
localparam [7:0] CMD_PING           = 8'hFE;
localparam [7:0] CMD_RESET          = 8'hFF;
