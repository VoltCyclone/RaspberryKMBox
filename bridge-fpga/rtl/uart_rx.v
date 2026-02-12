/**
 * UART Receiver - 3 Mbaud capable
 *
 * 8N1 UART RX with oversampling at 16x baud rate.
 * Samples at the center of each bit for maximum noise margin.
 * Designed for iCE40 UP5K at 48 MHz clock.
 */
module uart_rx #(
    parameter CLK_FREQ = 48_000_000,
    parameter BAUD     = 3_000_000
)(
    input  wire       clk,
    input  wire       rst_n,

    // Data interface
    output reg  [7:0] rx_data,
    output reg        rx_valid,

    // Error flags
    output reg        rx_frame_err,

    // UART input
    input  wire       uart_rxd
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD;  // 16 at 48MHz/3Mbaud
    localparam HALF_BIT     = CLKS_PER_BIT / 2;  // Sample point
    localparam BIT_CNT_W    = $clog2(CLKS_PER_BIT);

    localparam [2:0]
        S_IDLE  = 3'd0,
        S_START = 3'd1,
        S_DATA  = 3'd2,
        S_STOP  = 3'd3;

    // Input synchronizer (2-FF metastability guard)
    reg [1:0] rxd_sync;
    wire      rxd_s = rxd_sync[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rxd_sync <= 2'b11;
        else
            rxd_sync <= {rxd_sync[0], uart_rxd};
    end

    reg [2:0]          state;
    reg [BIT_CNT_W:0]  clk_cnt;
    reg [2:0]          bit_idx;
    reg [7:0]          shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            clk_cnt      <= 0;
            bit_idx      <= 0;
            shift_reg    <= 8'h00;
            rx_data      <= 8'h00;
            rx_valid     <= 1'b0;
            rx_frame_err <= 1'b0;
        end else begin
            rx_valid     <= 1'b0;
            rx_frame_err <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (!rxd_s) begin  // Start bit detected (falling edge)
                        clk_cnt <= 0;
                        state   <= S_START;
                    end
                end

                S_START: begin
                    // Sample at midpoint to verify start bit
                    if (clk_cnt == HALF_BIT - 1) begin
                        if (!rxd_s) begin
                            // Valid start bit
                            clk_cnt <= 0;
                            bit_idx <= 0;
                            state   <= S_DATA;
                        end else begin
                            // False start - back to idle
                            state <= S_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt   <= 0;
                        shift_reg <= {rxd_s, shift_reg[7:1]};  // LSB first
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        if (rxd_s) begin
                            // Valid stop bit
                            rx_data  <= shift_reg;
                            rx_valid <= 1'b1;
                        end else begin
                            // Framing error
                            rx_frame_err <= 1'b1;
                        end
                        state <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
