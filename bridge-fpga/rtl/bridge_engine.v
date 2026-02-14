/**
 * Bridge Engine - UART-to-UART Relay with Auto-Ping
 *
 * Relays bytes bidirectionally between the CMD UART (RP2350) and
 * the KMBox UART, with autonomous keepalive ping generation.
 *
 * Forward path:  CMD UART RX → 16-byte FIFO → KMBox UART TX
 * Return path:   KMBox UART RX → 16-byte FIFO → CMD UART TX
 * Auto-ping:     If no CMD traffic for 2s, inject PING (0xBD 0xFE) on KMBox TX
 *
 * Connection tracking:
 *   - `connected` goes high on any KMBox RX activity
 *   - `connected` goes low after 5s of KMBox RX silence
 */
module bridge_engine #(
    parameter CLK_FREQ       = 48_000_000,
    parameter PING_INTERVAL  = 48_000_000 * 2,   // 2 seconds
    parameter TIMEOUT_CLKS   = 48_000_000 * 5    // 5 seconds
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- CMD UART (from/to RP2350) ---
    input  wire [7:0]  cmd_rx_data,
    input  wire        cmd_rx_valid,
    output reg  [7:0]  cmd_tx_data,
    output reg         cmd_tx_valid,
    input  wire        cmd_tx_ready,

    // --- KMBox UART ---
    input  wire [7:0]  kmbox_rx_data,
    input  wire        kmbox_rx_valid,
    output reg  [7:0]  kmbox_tx_data,
    output reg         kmbox_tx_valid,
    input  wire        kmbox_tx_ready,

    // --- Status ---
    output reg         connected,
    output reg         activity       // High when UART traffic active
);

    // ================================================================
    // Forward FIFO: CMD RX → KMBox TX (16 bytes)
    // ================================================================
    reg [7:0]  fwd_fifo [0:15];
    reg [4:0]  fwd_wr, fwd_rd;
    wire [4:0] fwd_cnt = fwd_wr - fwd_rd;
    wire       fwd_empty = (fwd_wr == fwd_rd);
    wire       fwd_full  = (fwd_cnt == 5'd16);

    // Return FIFO: KMBox RX → CMD TX (16 bytes)
    reg [7:0]  ret_fifo [0:15];
    reg [4:0]  ret_wr, ret_rd;
    wire [4:0] ret_cnt = ret_wr - ret_rd;
    wire       ret_empty = (ret_wr == ret_rd);
    wire       ret_full  = (ret_cnt == 5'd16);

    // Auto-ping state
    reg [31:0] ping_timer;
    reg        ping_phase;       // 0 = send sync byte, 1 = send cmd byte
    reg        ping_active;

    // Connection tracker
    reg [31:0] conn_timer;

    // Activity tracker (for LED)
    reg [7:0]  activity_cnt;

    // ================================================================
    // Forward FIFO write: CMD RX → fwd_fifo
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fwd_wr <= 0;
        end else begin
            if (cmd_rx_valid && !fwd_full) begin
                fwd_fifo[fwd_wr[3:0]] <= cmd_rx_data;
                fwd_wr <= fwd_wr + 1;
            end
        end
    end

    // ================================================================
    // Return FIFO write: KMBox RX → ret_fifo
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ret_wr <= 0;
        end else begin
            if (kmbox_rx_valid && !ret_full) begin
                ret_fifo[ret_wr[3:0]] <= kmbox_rx_data;
                ret_wr <= ret_wr + 1;
            end
        end
    end

    // ================================================================
    // KMBox TX: fwd_fifo read + auto-ping injection
    // Manages fwd_rd, kmbox_tx_data, kmbox_tx_valid, ping state
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fwd_rd         <= 0;
            kmbox_tx_data  <= 0;
            kmbox_tx_valid <= 0;
            ping_timer     <= 0;
            ping_active    <= 0;
            ping_phase     <= 0;
        end else begin
            kmbox_tx_valid <= 1'b0;

            // Auto-ping timer: reset on any CMD RX traffic
            if (cmd_rx_valid)
                ping_timer <= 0;
            else if (ping_timer < PING_INTERVAL)
                ping_timer <= ping_timer + 1;

            if (ping_active) begin
                // Sending auto-ping packet (2 bytes: 0xBD, 0xFE)
                if (kmbox_tx_ready) begin
                    if (!ping_phase) begin
                        kmbox_tx_data  <= 8'hBD;   // SYNC
                        kmbox_tx_valid <= 1'b1;
                        ping_phase     <= 1'b1;
                    end else begin
                        kmbox_tx_data  <= 8'hFE;   // CMD_PING
                        kmbox_tx_valid <= 1'b1;
                        ping_active    <= 1'b0;
                        ping_phase     <= 1'b0;
                        ping_timer     <= 0;
                    end
                end
            end else if (ping_timer >= PING_INTERVAL && fwd_empty) begin
                // Start auto-ping when idle long enough
                ping_active <= 1'b1;
                ping_phase  <= 1'b0;
            end else if (!fwd_empty && kmbox_tx_ready) begin
                // Normal forward: FIFO → KMBox TX
                kmbox_tx_data  <= fwd_fifo[fwd_rd[3:0]];
                kmbox_tx_valid <= 1'b1;
                fwd_rd         <= fwd_rd + 1;
            end
        end
    end

    // ================================================================
    // CMD TX: ret_fifo read → RP2350
    // Manages ret_rd, cmd_tx_data, cmd_tx_valid
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ret_rd       <= 0;
            cmd_tx_data  <= 0;
            cmd_tx_valid <= 0;
        end else begin
            cmd_tx_valid <= 1'b0;

            if (!ret_empty && cmd_tx_ready) begin
                cmd_tx_data  <= ret_fifo[ret_rd[3:0]];
                cmd_tx_valid <= 1'b1;
                ret_rd       <= ret_rd + 1;
            end
        end
    end

    // ================================================================
    // Connection tracker & activity LED
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            connected    <= 0;
            conn_timer   <= 0;
            activity_cnt <= 0;
            activity     <= 0;
        end else begin
            // Connection: high when KMBox responds, times out after 5s
            if (kmbox_rx_valid) begin
                connected  <= 1'b1;
                conn_timer <= 0;
            end else if (conn_timer < TIMEOUT_CLKS) begin
                conn_timer <= conn_timer + 1;
            end else begin
                connected <= 1'b0;
            end

            // Activity: decaying counter for LED
            if (cmd_rx_valid || kmbox_rx_valid)
                activity_cnt <= 8'hFF;
            else if (activity_cnt > 0)
                activity_cnt <= activity_cnt - 1;

            activity <= (activity_cnt > 0);
        end
    end

endmodule
