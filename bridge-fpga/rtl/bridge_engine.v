/**
 * Bridge Protocol Engine
 *
 * Handles the KMBox bridge protocol:
 *   - Encodes commands (mouse move, wheel, button, ping) into UART packets
 *   - Decodes incoming UART responses (ACK, NACK, PONG, info packets)
 *   - Auto-generates keepalive pings
 *   - Maintains connection state and statistics
 *
 * Register Map (directly accessible via SPI):
 *   See bridge_regs.vh for full register definitions.
 *
 * Command Submission:
 *   1. Write mouse X/Y or wheel/button data to CMD_DATA registers
 *   2. Write command type to CMD_REG → packet is built and queued for TX
 *
 * Status Readback:
 *   Read STATUS_REG, counters, and RX info registers via SPI.
 */
module bridge_engine #(
    parameter CLK_FREQ      = 48_000_000,
    parameter PING_INTERVAL = 48_000_000 * 2  // 2 seconds at 48MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- Register bus from SPI target ---
    input  wire [6:0]  reg_addr,
    input  wire [7:0]  reg_wdata,
    input  wire        reg_wen,
    output reg  [7:0]  reg_rdata,
    input  wire        reg_ren,

    // --- UART TX interface ---
    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready,

    // --- UART RX interface ---
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,

    // --- Status ---
    output reg         connected,
    output wire [7:0]  tx_fifo_count,
    output wire [7:0]  rx_fifo_count
);

    `include "bridge_regs.vh"

    // ================================================================
    // Internal registers
    // ================================================================

    // Command data registers (written by RP2350 via SPI)
    reg signed [15:0] cmd_mouse_x;
    reg signed [15:0] cmd_mouse_y;
    reg signed [7:0]  cmd_wheel;
    reg [7:0]         cmd_btn_mask;
    reg [7:0]         cmd_btn_state;
    reg [7:0]         cmd_type;      // Written last → triggers send
    reg               cmd_pending;

    // Status / connection
    reg [7:0]  conn_state;    // 0=disconnected, 1=connected
    reg [15:0] tx_pkt_count;
    reg [15:0] rx_pkt_count;
    reg [15:0] rx_err_count;
    reg [7:0]  last_rx_cmd;
    reg [7:0]  last_rx_payload [0:5];

    // Ping auto-timer
    reg [31:0] ping_timer;
    reg        ping_needed;

    // TX packet builder state machine
    localparam [2:0]
        TX_IDLE   = 3'd0,
        TX_SYNC   = 3'd1,
        TX_CMD    = 3'd2,
        TX_DATA   = 3'd3,
        TX_DONE   = 3'd4;

    reg [2:0]  tx_state;
    reg [7:0]  tx_packet [0:6];  // Max 7 bytes: sync + cmd + 5 payload
    reg [2:0]  tx_pkt_len;
    reg [2:0]  tx_byte_idx;

    // RX parser state machine
    localparam [1:0]
        RX_IDLE    = 2'd0,
        RX_CMD     = 2'd1,
        RX_PAYLOAD = 2'd2;

    reg [1:0]  rx_state;
    reg [7:0]  rx_cmd;
    reg [2:0]  rx_bytes_needed;
    reg [2:0]  rx_byte_idx;
    reg [7:0]  rx_payload [0:7]; // Up to 8 byte payload for binary responses

    // RX binary packet parser (for 8-byte fixed packets from KMBox)
    reg [7:0]  rx_bin_buf [0:7];
    reg [2:0]  rx_bin_idx;
    reg        rx_bin_active;
    reg [23:0] rx_timeout_cnt;  // Timeout counter for stuck binary packets

    // RX FIFO for forwarding raw bytes to RP2350
    reg [7:0]  rx_fifo_mem [0:63];
    reg [6:0]  rx_fifo_wr;
    reg [6:0]  rx_fifo_rd;
    wire [6:0] rx_fifo_cnt = rx_fifo_wr - rx_fifo_rd;

    assign rx_fifo_count = {1'b0, rx_fifo_cnt};
    assign tx_fifo_count = {5'b0, tx_byte_idx};

    // Connection timeout counter
    reg [31:0] rx_activity_timer;
    localparam TIMEOUT_CLKS = CLK_FREQ * 5;  // 5 second timeout

    // ================================================================
    // Register read logic
    // ================================================================
    always @(*) begin
        case (reg_addr)
            REG_STATUS:       reg_rdata = {7'b0, connected};
            REG_CONN_STATE:   reg_rdata = conn_state;
            REG_TX_COUNT_LO:  reg_rdata = tx_pkt_count[7:0];
            REG_TX_COUNT_HI:  reg_rdata = tx_pkt_count[15:8];
            REG_RX_COUNT_LO:  reg_rdata = rx_pkt_count[7:0];
            REG_RX_COUNT_HI:  reg_rdata = rx_pkt_count[15:8];
            REG_RX_ERR_LO:   reg_rdata = rx_err_count[7:0];
            REG_RX_ERR_HI:   reg_rdata = rx_err_count[15:8];
            REG_LAST_RX_CMD:  reg_rdata = last_rx_cmd;
            REG_RX_FIFO_CNT: reg_rdata = {1'b0, rx_fifo_cnt};
            REG_RX_FIFO_DATA: reg_rdata = (rx_fifo_wr != rx_fifo_rd) ? rx_fifo_mem[rx_fifo_rd[5:0]] : 8'h00;
            REG_VERSION:      reg_rdata = 8'h01;  // Firmware version 1.0
            REG_TX_BUSY:      reg_rdata = {7'b0, (tx_state != TX_IDLE)};
            default:          reg_rdata = 8'h00;
        endcase
    end

    // ================================================================
    // Register write logic
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_mouse_x  <= 0;
            cmd_mouse_y  <= 0;
            cmd_wheel    <= 0;
            cmd_btn_mask <= 0;
            cmd_btn_state <= 0;
            cmd_type     <= 0;
            cmd_pending  <= 0;
        end else begin
            // Clear pending once TX FSM picks it up
            if (tx_state == TX_SYNC && cmd_pending)
                cmd_pending <= 1'b0;

            if (reg_wen) begin
                case (reg_addr)
                    REG_CMD_MOUSE_XL: cmd_mouse_x[7:0]  <= reg_wdata;
                    REG_CMD_MOUSE_XH: cmd_mouse_x[15:8] <= reg_wdata;
                    REG_CMD_MOUSE_YL: cmd_mouse_y[7:0]  <= reg_wdata;
                    REG_CMD_MOUSE_YH: cmd_mouse_y[15:8] <= reg_wdata;
                    REG_CMD_WHEEL:    cmd_wheel          <= reg_wdata;
                    REG_CMD_BTN_MASK: cmd_btn_mask       <= reg_wdata;
                    REG_CMD_BTN_STATE: cmd_btn_state     <= reg_wdata;
                    REG_CMD_TYPE: begin
                        cmd_type    <= reg_wdata;
                        cmd_pending <= 1'b1;
                    end
                    default: ;
                endcase
            end

            // Pop RX FIFO on read
            if (reg_ren && reg_addr == REG_RX_FIFO_DATA) begin
                if (rx_fifo_wr != rx_fifo_rd)
                    rx_fifo_rd <= rx_fifo_rd + 1;
            end
        end
    end

    // ================================================================
    // TX Packet Builder FSM
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            tx_valid    <= 0;
            tx_data     <= 0;
            tx_byte_idx <= 0;
            tx_pkt_len  <= 0;
            tx_pkt_count <= 0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx_valid <= 1'b0;
                    if (cmd_pending) begin
                        // Build packet based on command type
                        tx_packet[0] <= SYNC_BYTE;
                        case (cmd_type)
                            CMD_MOUSE_MOVE: begin
                                tx_packet[1] <= CMD_MOUSE_MOVE;
                                tx_packet[2] <= cmd_mouse_x[7:0];
                                tx_packet[3] <= cmd_mouse_x[15:8];
                                tx_packet[4] <= cmd_mouse_y[7:0];
                                tx_packet[5] <= cmd_mouse_y[15:8];
                                tx_pkt_len   <= 6;
                            end
                            CMD_MOUSE_WHEEL: begin
                                tx_packet[1] <= CMD_MOUSE_WHEEL;
                                tx_packet[2] <= cmd_wheel;
                                tx_pkt_len   <= 3;
                            end
                            CMD_BUTTON_SET: begin
                                tx_packet[1] <= CMD_BUTTON_SET;
                                tx_packet[2] <= cmd_btn_mask;
                                tx_packet[3] <= cmd_btn_state;
                                tx_pkt_len   <= 4;
                            end
                            CMD_MOUSE_MOVE_WHEEL: begin
                                tx_packet[1] <= CMD_MOUSE_MOVE_WHEEL;
                                tx_packet[2] <= cmd_mouse_x[7:0];
                                tx_packet[3] <= cmd_mouse_x[15:8];
                                tx_packet[4] <= cmd_mouse_y[7:0];
                                tx_packet[5] <= cmd_mouse_y[15:8];
                                tx_packet[6] <= cmd_wheel;
                                tx_pkt_len   <= 7;
                            end
                            CMD_PING: begin
                                tx_packet[1] <= CMD_PING;
                                tx_pkt_len   <= 2;
                            end
                            CMD_RESET: begin
                                tx_packet[1] <= CMD_RESET;
                                tx_pkt_len   <= 2;
                            end
                            default: begin
                                // Unknown command - send as raw 8-byte packet
                                tx_packet[1] <= cmd_type;
                                tx_pkt_len   <= 2;
                            end
                        endcase
                        tx_byte_idx <= 0;
                        tx_state    <= TX_SYNC;
                    end else if (ping_needed) begin
                        // Auto-ping
                        tx_packet[0] <= SYNC_BYTE;
                        tx_packet[1] <= CMD_PING;
                        tx_pkt_len   <= 2;
                        tx_byte_idx  <= 0;
                        tx_state     <= TX_SYNC;
                        ping_needed  <= 1'b0;
                    end
                end

                TX_SYNC: begin
                    // Start sending packet bytes
                    if (tx_ready) begin
                        tx_data  <= tx_packet[tx_byte_idx];
                        tx_valid <= 1'b1;
                        tx_state <= TX_CMD;
                    end
                end

                TX_CMD: begin
                    tx_valid <= 1'b0;
                    if (tx_ready) begin
                        tx_byte_idx <= tx_byte_idx + 1;
                        if (tx_byte_idx + 1 >= tx_pkt_len) begin
                            tx_state    <= TX_DONE;
                            tx_pkt_count <= tx_pkt_count + 1;
                        end else begin
                            tx_data  <= tx_packet[tx_byte_idx + 1];
                            tx_valid <= 1'b1;
                        end
                    end
                end

                TX_DONE: begin
                    tx_valid <= 1'b0;
                    tx_state <= TX_IDLE;
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // ================================================================
    // RX Parser - handles both bridge protocol and 8-byte binary packets
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_bin_active    <= 0;
            rx_bin_idx       <= 0;
            rx_timeout_cnt   <= 0;
            rx_pkt_count     <= 0;
            rx_err_count     <= 0;
            last_rx_cmd      <= 0;
            rx_fifo_wr       <= 0;
            rx_fifo_rd       <= 0;
        end else begin
            // Timeout: reset binary state if no byte for ~1ms
            if (rx_bin_active) begin
                if (rx_timeout_cnt >= (CLK_FREQ / 1000)) begin
                    rx_bin_active  <= 0;
                    rx_bin_idx     <= 0;
                    rx_timeout_cnt <= 0;
                end else begin
                    rx_timeout_cnt <= rx_timeout_cnt + 1;
                end
            end

            if (rx_valid) begin
                rx_timeout_cnt <= 0;

                // Push all received bytes into RX FIFO for RP2350 readback
                if (rx_fifo_cnt < 7'd63) begin
                    rx_fifo_mem[rx_fifo_wr[5:0]] <= rx_data;
                    rx_fifo_wr <= rx_fifo_wr + 1;
                end

                // Try to detect known binary response headers
                if (!rx_bin_active && 
                    (rx_data == 8'hFF || rx_data == 8'hFE ||
                     rx_data == 8'h0C || rx_data == 8'h0E ||
                     rx_data == 8'hA0 || rx_data == 8'hA2)) begin
                    rx_bin_active     <= 1;
                    rx_bin_buf[0]     <= rx_data;
                    rx_bin_idx        <= 1;
                end else if (rx_bin_active) begin
                    rx_bin_buf[rx_bin_idx] <= rx_data;
                    rx_bin_idx <= rx_bin_idx + 1;

                    if (rx_bin_idx == 3'd7) begin
                        // Complete 8-byte packet received
                        rx_pkt_count  <= rx_pkt_count + 1;
                        last_rx_cmd   <= rx_bin_buf[0];
                        rx_bin_active <= 0;
                        rx_bin_idx    <= 0;

                        // Update connection state
                        rx_activity_timer <= 0;
                    end
                end
            end
        end
    end

    // ================================================================
    // Connection state & auto-ping timer
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            connected        <= 0;
            conn_state       <= 0;
            ping_timer       <= 0;
            ping_needed      <= 0;
            rx_activity_timer <= 0;
        end else begin
            // Activity timer
            if (rx_valid) begin
                rx_activity_timer <= 0;
                if (!connected) begin
                    connected  <= 1;
                    conn_state <= 8'h01;
                end
            end else begin
                if (rx_activity_timer < TIMEOUT_CLKS) begin
                    rx_activity_timer <= rx_activity_timer + 1;
                end else begin
                    connected  <= 0;
                    conn_state <= 8'h00;
                end
            end

            // Ping timer
            if (ping_timer >= PING_INTERVAL) begin
                ping_timer  <= 0;
                ping_needed <= 1'b1;
            end else begin
                ping_timer <= ping_timer + 1;
            end

            // Clear ping_needed when consumed
            if (tx_state == TX_SYNC && ping_needed && !cmd_pending)
                ping_needed <= 1'b0;
        end
    end

endmodule
