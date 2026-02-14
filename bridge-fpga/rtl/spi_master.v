/**
 * SPI Master - 12 MHz, 64-bit fixed transactions
 *
 * SPI Mode 0 (CPOL=0, CPHA=0) master for sending 8-byte KMBox command
 * packets from the iCE40 UP5K FPGA to the KMBox RP2350.
 *
 * The FPGA is clock master; everything runs in the 48 MHz domain.
 * SPI clock is 12 MHz (48 MHz / 4: 2 clocks high, 2 clocks low).
 * MSB-first, full-duplex 64-bit transfers.
 *
 * Transaction timing (48 MHz clocks):
 *   SETUP:      2 clocks  (CS_N asserted, SCK=0)
 *   DATA:     256 clocks  (64 bits x 4 clocks per bit)
 *   HOLD:       2 clocks  (SCK=0 before CS_N deassert)
 *   Total:    260 clocks  = 5.42 us per 8-byte packet
 *
 * Mode 0 waveform (per bit):
 *   CLOCK_LOW  (2 clks): SCK=0, MOSI drives current TX bit
 *   CLOCK_HIGH (2 clks): SCK=1, MISO sampled on first clock of this phase
 */
module spi_master (
    input  wire        clk,        // 48 MHz system clock
    input  wire        rst_n,      // Active-low reset

    // SPI pins (directly driven from 48 MHz domain)
    output reg         spi_sck,    // SPI clock output (12 MHz)
    output reg         spi_cs_n,   // Chip select (active low)
    output reg         spi_mosi,   // Data to slave
    input  wire        spi_miso,   // Data from slave

    // Internal interface
    input  wire [63:0] tx_data,    // 8-byte data to send
    input  wire        tx_valid,   // Pulse to start transaction (only when tx_ready=1)
    output reg         tx_ready,   // High when idle, ready for new transaction
    output reg  [63:0] rx_data,    // 8-byte data received from slave
    output reg         rx_valid    // Pulse high for one clock when transaction complete
);

    // ================================================================
    // State encoding
    // ================================================================
    localparam [2:0]
        S_IDLE       = 3'd0,
        S_SETUP      = 3'd1,
        S_CLOCK_LOW  = 3'd2,
        S_CLOCK_HIGH = 3'd3,
        S_HOLD       = 3'd4;

    // ================================================================
    // Registers
    // ================================================================
    reg [2:0]  state;
    reg [1:0]  clk_div;     // 2-bit counter for 48/4 = 12 MHz timing
    reg [5:0]  bit_cnt;     // 0-63: which bit we are transferring
    reg [63:0] tx_shift;    // TX shift register (MSB out first)
    reg [63:0] rx_shift;    // RX shift register (MSB in first)

    // ================================================================
    // State machine
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            clk_div  <= 2'd0;
            bit_cnt  <= 6'd0;
            tx_shift <= 64'd0;
            rx_shift <= 64'd0;
            spi_sck  <= 1'b0;
            spi_cs_n <= 1'b1;
            spi_mosi <= 1'b0;
            tx_ready <= 1'b1;
            rx_data  <= 64'd0;
            rx_valid <= 1'b0;
        end else begin
            // Default: clear rx_valid pulse after one clock
            rx_valid <= 1'b0;

            case (state)
                // ------------------------------------------------
                // IDLE: SCK=0, CS_N=1, waiting for tx_valid
                // ------------------------------------------------
                S_IDLE: begin
                    spi_sck  <= 1'b0;
                    spi_cs_n <= 1'b1;
                    spi_mosi <= 1'b0;
                    tx_ready <= 1'b1;

                    if (tx_valid && tx_ready) begin
                        tx_shift <= tx_data;
                        tx_ready <= 1'b0;
                        spi_cs_n <= 1'b0;       // Assert chip select
                        clk_div  <= 2'd0;
                        state    <= S_SETUP;
                    end
                end

                // ------------------------------------------------
                // SETUP: CS_N=0, SCK=0 for 2 clocks (CS setup time)
                // ------------------------------------------------
                S_SETUP: begin
                    spi_sck <= 1'b0;

                    if (clk_div == 2'd1) begin
                        // Setup done after 2 clocks, enter first bit
                        clk_div  <= 2'd0;
                        bit_cnt  <= 6'd0;
                        spi_mosi <= tx_shift[63]; // Drive MSB on MOSI
                        state    <= S_CLOCK_LOW;
                    end else begin
                        clk_div <= clk_div + 2'd1;
                    end
                end

                // ------------------------------------------------
                // CLOCK_LOW: SCK=0, MOSI stable. Hold 2 clocks.
                // Then raise SCK and go to CLOCK_HIGH.
                // ------------------------------------------------
                S_CLOCK_LOW: begin
                    spi_sck <= 1'b0;

                    if (clk_div == 2'd1) begin
                        clk_div <= 2'd0;
                        spi_sck <= 1'b1;         // Rising edge of SCK
                        state   <= S_CLOCK_HIGH;
                    end else begin
                        clk_div <= clk_div + 2'd1;
                    end
                end

                // ------------------------------------------------
                // CLOCK_HIGH: SCK=1. Sample MISO on entry (clk_div==0).
                // Hold 2 clocks. Then check if done or next bit.
                // ------------------------------------------------
                S_CLOCK_HIGH: begin
                    spi_sck <= 1'b1;

                    // Sample MISO on the first clock of the high phase
                    if (clk_div == 2'd0) begin
                        rx_shift <= {rx_shift[62:0], spi_miso};
                    end

                    if (clk_div == 2'd1) begin
                        clk_div <= 2'd0;
                        spi_sck <= 1'b0;         // Falling edge of SCK

                        if (bit_cnt == 6'd63) begin
                            // All 64 bits transferred
                            state <= S_HOLD;
                        end else begin
                            // Advance to next bit
                            bit_cnt  <= bit_cnt + 6'd1;
                            tx_shift <= {tx_shift[62:0], 1'b0};
                            spi_mosi <= tx_shift[62]; // Next MSB
                            state    <= S_CLOCK_LOW;
                        end
                    end else begin
                        clk_div <= clk_div + 2'd1;
                    end
                end

                // ------------------------------------------------
                // HOLD: SCK=0, CS_N=0 for 2 clocks (CS hold time),
                // then deassert CS, pulse rx_valid, return to IDLE.
                // ------------------------------------------------
                S_HOLD: begin
                    spi_sck  <= 1'b0;
                    spi_mosi <= 1'b0;

                    if (clk_div == 2'd1) begin
                        clk_div  <= 2'd0;
                        spi_cs_n <= 1'b1;        // Deassert chip select
                        rx_data  <= rx_shift;     // Latch received data
                        rx_valid <= 1'b1;         // Pulse for one clock
                        state    <= S_IDLE;
                    end else begin
                        clk_div <= clk_div + 2'd1;
                    end
                end

                default: begin
                    state    <= S_IDLE;
                    spi_sck  <= 1'b0;
                    spi_cs_n <= 1'b1;
                    spi_mosi <= 1'b0;
                    tx_ready <= 1'b1;
                end
            endcase
        end
    end

endmodule
