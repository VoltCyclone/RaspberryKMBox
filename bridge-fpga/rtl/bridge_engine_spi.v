/**
 * Bridge Engine - SPI-to-SPI Relay with Auto-Ping
 *
 * Receives 8-byte command packets from the bridge RP2350 (via SPI target),
 * buffers them in a FIFO, and forwards them to the KMBox RP2350 (via SPI master).
 *
 * Both links use KMBox fast binary 8-byte format — no protocol translation.
 * The engine is essentially a FIFO with auto-ping injection and connection tracking.
 *
 * CMD opcode 0x00 is treated as NOP (status read only, not forwarded).
 * All other opcodes are forwarded to KMBox.
 *
 * Auto-ping: When no commands arrive for PING_INTERVAL clocks, the engine
 * injects a ping packet [0xFE, 0x00..0x00] to the KMBox.
 *
 * Connection tracking: `connected` goes high on any KMBox SPI RX activity,
 * times out after TIMEOUT_CLKS of silence.
 *
 * Timing strategy: (1) A one-deep holding register breaks the cmd_push
 * qualification (8-bit NOP compare + full flag + cmd_valid) into a separate
 * pipeline stage, reducing the FIFO push path to a 2-input AND gate.
 * (2) FIFO status signals (full, empty, count) are maintained as registered
 * counters updated on push/pop events, avoiding combinational arithmetic
 * on pointer differences.
 */
module bridge_engine_spi #(
    parameter CLK_FREQ      = 48_000_000,
    parameter PING_INTERVAL = 48_000_000 * 2,   // 2 seconds
    parameter TIMEOUT_CLKS  = 48_000_000 * 5    // 5 seconds
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- From SPI target (bridge RP2350 commands) ---
    input  wire [63:0] cmd_data,       // 8-byte command from SPI target
    input  wire        cmd_valid,      // Pulse: complete frame received
    input  wire        txn_done,       // Pulse: SPI transaction completed (for RX pop)

    // --- To/from SPI master (KMBox output) ---
    output reg  [63:0] km_tx_data,     // 8-byte packet to send to KMBox
    output reg         km_tx_valid,    // Pulse to start KMBox SPI transaction
    input  wire        km_tx_ready,    // High when SPI master is idle
    input  wire [63:0] km_rx_data,     // 8-byte response from KMBox
    input  wire        km_rx_valid,    // Pulse: KMBox SPI transaction complete

    // --- Response data for SPI target MISO ---
    output reg  [63:0] resp_data,

    // --- Status ---
    output wire        connected,
    output wire        activity
);

    // ================================================================
    // Command FIFO: 4 entries × 64 bits
    //
    // Instead of computing cnt = wr - rd combinationally, we maintain
    // a registered count that is updated on push/pop events.
    // This eliminates the subtractor + comparator chain from timing.
    // ================================================================
    reg [63:0] cmd_fifo [0:3];
    reg [1:0]  cmd_wr_ptr, cmd_rd_ptr;
    reg [2:0]  cmd_count;          // 0..4, registered
    reg        cmd_empty_r;        // registered empty flag
    reg        cmd_full_r;         // registered full flag

    // Holding register: incoming commands are latched here, then pushed
    // to FIFO next cycle.  This breaks the critical path from the 8-bit
    // NOP compare + full-flag check into two shallow pipeline stages.
    reg        cmd_pending;
    reg [63:0] cmd_pending_data;

    // Push/pop events — cmd_push is now only 2 registered inputs (1 LUT)
    wire cmd_push = cmd_pending && !cmd_full_r;
    wire cmd_pop  = !cmd_empty_r && km_tx_ready;

    // ================================================================
    // Command holding register
    //
    // Latches valid non-NOP commands from SPI target.  The 8-bit opcode
    // compare and cmd_valid qualification happen here; the actual FIFO
    // push (cmd_push) is just cmd_pending && !cmd_full_r — two registered
    // signals, one LUT deep — keeping the FIFO management path short.
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_pending      <= 1'b0;
            cmd_pending_data <= 64'h0;
        end else begin
            if (cmd_push)
                cmd_pending <= 1'b0;          // consumed into FIFO
            if (cmd_valid && (cmd_data[63:56] != 8'h00)) begin
                cmd_pending      <= 1'b1;     // accept (wins over clear)
                cmd_pending_data <= cmd_data;
            end
        end
    end

    // ================================================================
    // RX FIFO: 8 entries × 64 bits (stores KMBox responses)
    //
    // Same registered-count strategy as command FIFO.
    // ================================================================
    reg [63:0] rx_fifo [0:7];
    reg [2:0]  rx_wr_ptr, rx_rd_ptr;
    reg [3:0]  rx_count;           // 0..8, registered
    reg        rx_empty_r;         // registered empty flag
    reg        rx_full_r;          // registered full flag
    reg [63:0] rx_peek_r;          // registered first entry for MISO

    wire rx_push = km_rx_valid && !rx_full_r;
    wire rx_pop  = txn_done && !rx_empty_r;

    // ================================================================
    // Status registers
    // ================================================================
    reg        connected_reg;
    reg        ping_expired;       // Registered flag: ping timer reached interval
    reg        conn_expired;       // Registered flag: connection timer reached timeout
    reg [31:0] conn_timer;
    reg [31:0] ping_timer;
    reg [7:0]  activity_cnt;
    reg        activity_reg;
    reg [7:0]  seq_counter;
    reg        last_cmd_ok;

    assign connected = connected_reg;
    assign activity  = activity_reg;

    // ================================================================
    // Response register (registered output for SPI target MISO)
    //
    // Byte 0: STATUS flags
    // Byte 1: RX_COUNT
    // Byte 2-5: First 4 bytes of rx_peek (first response packet)
    // Byte 6: TX_FREE (command FIFO free slots)
    // Byte 7: SEQ counter
    //
    // All inputs are registered — simple register-to-register paths.
    // ================================================================
    reg [7:0] tx_free_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_free_r <= 8'd4;
            resp_data <= 64'h0;
        end else begin
            tx_free_r <= 4 - cmd_count[2:0];
            resp_data <= {
                // Byte 0: status flags
                3'b0,
                last_cmd_ok,                       // bit 4: cmd_ok
                1'b0,                              // bit 3: uart_err (unused)
                ~rx_empty_r,                       // bit 2: rx_avail
                cmd_full_r,                        // bit 1: tx_full
                connected_reg,                     // bit 0: connected
                // Byte 1: rx count
                rx_count[3:0], 4'h0,
                // Bytes 2-5: first 4 bytes of response
                rx_peek_r[63:32],
                // Byte 6: tx_free
                tx_free_r,
                // Byte 7: seq counter
                seq_counter
            };
        end
    end

    // ================================================================
    // Command FIFO: push, pop, and count management
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_wr_ptr  <= 2'd0;
            cmd_rd_ptr  <= 2'd0;
            cmd_count   <= 3'd0;
            cmd_empty_r <= 1'b1;
            cmd_full_r  <= 1'b0;
        end else begin
            case ({cmd_push, cmd_pop})
                2'b10: begin  // push only
                    cmd_fifo[cmd_wr_ptr] <= cmd_pending_data;
                    cmd_wr_ptr  <= cmd_wr_ptr + 1;
                    cmd_count   <= cmd_count + 1;
                    cmd_empty_r <= 1'b0;
                    cmd_full_r  <= (cmd_count == 3'd3);  // will become 4
                end
                2'b01: begin  // pop only
                    cmd_rd_ptr  <= cmd_rd_ptr + 1;
                    cmd_count   <= cmd_count - 1;
                    cmd_empty_r <= (cmd_count == 3'd1);  // will become 0
                    cmd_full_r  <= 1'b0;
                end
                2'b11: begin  // simultaneous push and pop
                    cmd_fifo[cmd_wr_ptr] <= cmd_pending_data;
                    cmd_wr_ptr  <= cmd_wr_ptr + 1;
                    cmd_rd_ptr  <= cmd_rd_ptr + 1;
                    // count stays the same, empty/full unchanged
                end
                default: ;  // no change
            endcase
        end
    end

    // ================================================================
    // RX FIFO: push, pop, count management, and peek register
    //
    // The peek register is updated one cycle after a pop by using a
    // registered "needs update" flag, avoiding the combinational
    // rx_fifo[rx_rd_ptr+1] mux in the pop path.
    // ================================================================
    reg rx_peek_update;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wr_ptr     <= 3'd0;
            rx_rd_ptr     <= 3'd0;
            rx_count      <= 4'd0;
            rx_empty_r    <= 1'b1;
            rx_full_r     <= 1'b0;
            rx_peek_r     <= 64'h0;
            rx_peek_update <= 1'b0;
        end else begin
            // Deferred peek update: read rx_fifo[rx_rd_ptr] one cycle after pop
            if (rx_peek_update) begin
                rx_peek_update <= 1'b0;
                if (rx_empty_r)
                    rx_peek_r <= 64'h0;
                else
                    rx_peek_r <= rx_fifo[rx_rd_ptr];
            end

            case ({rx_push, rx_pop})
                2'b10: begin  // push only
                    rx_fifo[rx_wr_ptr] <= km_rx_data;
                    rx_wr_ptr  <= rx_wr_ptr + 1;
                    rx_count   <= rx_count + 1;
                    rx_empty_r <= 1'b0;
                    rx_full_r  <= (rx_count == 4'd7);  // will become 8
                    // Update peek: if FIFO was empty, new data is now head
                    if (rx_empty_r)
                        rx_peek_r <= km_rx_data;
                end
                2'b01: begin  // pop only
                    rx_rd_ptr  <= rx_rd_ptr + 1;
                    rx_count   <= rx_count - 1;
                    rx_empty_r <= (rx_count == 4'd1);  // will become 0
                    rx_full_r  <= 1'b0;
                    // Schedule peek update for next cycle
                    rx_peek_update <= 1'b1;
                end
                2'b11: begin  // simultaneous push and pop
                    rx_fifo[rx_wr_ptr] <= km_rx_data;
                    rx_wr_ptr  <= rx_wr_ptr + 1;
                    rx_rd_ptr  <= rx_rd_ptr + 1;
                    // count stays the same, empty/full unchanged
                    // Schedule peek update for next cycle
                    rx_peek_update <= 1'b1;
                end
                default: ;  // no change
            endcase
        end
    end

    // ================================================================
    // Ping timer with registered terminal-count flag
    //
    // Instead of comparing a 32-bit counter against a large constant
    // in the critical path, we register the comparison result.
    // ping_expired is set one cycle after the counter reaches the
    // interval — this adds 1 clock latency (negligible at 2s interval)
    // but completely eliminates the wide comparator from the critical path.
    //
    // ping_ack (from TX block) resets the timer when a ping is sent.
    // This avoids multi-driver conflicts: only this block drives
    // ping_timer and ping_expired.
    // ================================================================
    reg ping_ack;  // Set by TX block when auto-ping is sent

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ping_timer   <= 0;
            ping_expired <= 1'b0;
        end else begin
            if (ping_ack || (cmd_valid && cmd_data[63:56] != 8'h00)) begin
                ping_timer   <= 0;
                ping_expired <= 1'b0;
            end else if (!ping_expired) begin
                if (ping_timer == PING_INTERVAL - 1)
                    ping_expired <= 1'b1;
                ping_timer <= ping_timer + 1;
            end
        end
    end

    // ================================================================
    // KMBox TX: Forward commands from FIFO to SPI master + auto-ping
    //
    // Uses registered flags (cmd_empty_r, ping_expired) for minimal
    // combinational depth. The cmd_pop wire feeds back into the FIFO
    // management above.
    //
    // ping_ack signals the timer block to reset after sending a ping.
    // Only this block drives km_tx_data, km_tx_valid, and ping_ack.
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            km_tx_data  <= 64'h0;
            km_tx_valid <= 1'b0;
            ping_ack    <= 1'b0;
        end else begin
            km_tx_valid <= 1'b0;
            ping_ack    <= 1'b0;

            // Forward from FIFO or inject auto-ping
            if (!cmd_empty_r && km_tx_ready) begin
                // Normal forward: FIFO → KMBox SPI master
                km_tx_data  <= cmd_fifo[cmd_rd_ptr];
                km_tx_valid <= 1'b1;
            end else if (cmd_empty_r && ping_expired && km_tx_ready) begin
                // Auto-ping: inject [0xFE, 0x00..0x00]
                km_tx_data  <= {8'hFE, 56'h0};
                km_tx_valid <= 1'b1;
                ping_ack    <= 1'b1;  // Tell timer block to reset
            end
        end
    end

    // ================================================================
    // Sequence counter and cmd_ok tracking
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seq_counter <= 0;
            last_cmd_ok <= 0;
        end else begin
            if (cmd_valid) begin
                seq_counter <= seq_counter + 1;
                // cmd_ok if non-NOP (holding register provides backpressure)
                last_cmd_ok <= (cmd_data[63:56] != 8'h00);
            end
        end
    end

    // ================================================================
    // Connection tracker & activity LED
    //
    // Uses registered conn_expired flag to avoid wide comparator in
    // the critical path.
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            connected_reg <= 0;
            conn_timer    <= 0;
            conn_expired  <= 1'b0;
            activity_cnt  <= 0;
            activity_reg  <= 0;
        end else begin
            // Connection timeout: registered comparison
            if (km_rx_valid) begin
                connected_reg <= 1'b1;
                conn_timer    <= 0;
                conn_expired  <= 1'b0;
            end else if (!conn_expired) begin
                if (conn_timer == TIMEOUT_CLKS - 1) begin
                    conn_expired  <= 1'b1;
                    connected_reg <= 1'b0;
                end
                conn_timer <= conn_timer + 1;
            end

            // Activity: decaying counter for LED
            if (cmd_valid || km_rx_valid)
                activity_cnt <= 8'hFF;
            else if (activity_cnt > 0)
                activity_cnt <= activity_cnt - 1;

            activity_reg <= (activity_cnt > 0);
        end
    end

endmodule
