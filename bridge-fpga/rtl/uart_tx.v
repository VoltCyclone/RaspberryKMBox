/**
 * UART Transmitter - 3 Mbaud capable
 *
 * Simple 8N1 UART TX with FIFO input interface.
 * Designed for iCE40 UP5K at 48 MHz clock.
 *
 * At 48 MHz / 3 Mbaud = 16 clocks per bit (exact, 0 ppm error).
 */
module uart_tx #(
    parameter CLK_FREQ = 48_000_000,
    parameter BAUD     = 3_000_000
)(
    input  wire       clk,
    input  wire       rst_n,

    // Data interface
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output reg        tx_ready,

    // UART output
    output reg        uart_txd
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD;  // 16 at 48MHz/3Mbaud
    localparam BIT_CNT_W    = $clog2(CLKS_PER_BIT);

    localparam [2:0]
        S_IDLE  = 3'd0,
        S_START = 3'd1,
        S_DATA  = 3'd2,
        S_STOP  = 3'd3;

    reg [2:0]          state;
    reg [BIT_CNT_W:0]  clk_cnt;
    reg [2:0]          bit_idx;
    reg [7:0]          shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= 8'h00;
            uart_txd  <= 1'b1;
            tx_ready  <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    uart_txd <= 1'b1;
                    tx_ready <= 1'b1;
                    if (tx_valid && tx_ready) begin
                        shift_reg <= tx_data;
                        tx_ready  <= 1'b0;
                        state     <= S_START;
                        clk_cnt   <= 0;
                    end
                end

                S_START: begin
                    uart_txd <= 1'b0;  // Start bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    uart_txd <= shift_reg[0];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt   <= 0;
                        shift_reg <= {1'b0, shift_reg[7:1]};
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
                    uart_txd <= 1'b1;  // Stop bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
