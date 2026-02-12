/**
 * KMBox FPGA Bridge - Top Level Module
 *
 * iCE40 UP5K FPGA that offloads KMBox UART communication from the RP2350,
 * leaving it free for USB CDC handling and TFT display rendering.
 *
 * Architecture:
 *   RP2350 ←(SPI)→ iCE40 FPGA ←(UART 3Mbaud)→ KMBox RP2350
 *
 * The RP2350 sends mouse/keyboard commands via SPI registers.
 * The FPGA builds bridge protocol packets and sends them over UART.
 * The FPGA receives responses and buffers them for RP2350 readback.
 * Auto-ping keepalive runs independently in the FPGA.
 *
 * Pin Usage (pico2-ice board):
 *   SPI: Uses on-board SPI bus (shared with CRAM loading)
 *   UART: Uses PMOD A pins for KMBox communication
 *   LEDs: Status indication via on-board RGB LEDs
 */
module top (
    input  wire CLK,           // 48 MHz from RP2350

    // SPI interface (directly on ICE40-RP2350 bus)
    input  wire ICE_SCK,       // SPI clock
    input  wire ICE_SI,        // SPI MOSI (master out, slave in)
    output wire ICE_SO,        // SPI MISO (master in, slave out)
    input  wire ICE_SSN,       // SPI chip select (active low)

    // UART to KMBox (PMOD A top pins)
    output wire UART_TX,       // FPGA TX → KMBox RX (PMOD_A_TOP_IO2 = pin 2)
    input  wire UART_RX,       // FPGA RX ← KMBox TX (PMOD_A_TOP_IO1 = pin 4)

    // Status LEDs (active low, accent from RP2350)
    output wire LED_R,
    output wire LED_G,
    output wire LED_B
);

    // Internal reset (power-on reset using iCE40 SB_GB)
    reg [7:0] rst_cnt = 0;
    wire rst_n = rst_cnt[7];

    always @(posedge CLK) begin
        if (!rst_n)
            rst_cnt <= rst_cnt + 1;
    end

    // ================================================================
    // SPI Target
    // ================================================================
    wire [6:0] reg_addr;
    wire [7:0] reg_wdata;
    wire       reg_wen;
    wire [7:0] reg_rdata;
    wire       reg_ren;
    wire       reg_done;

    spi_target u_spi (
        .clk       (CLK),
        .rst_n     (rst_n),
        .spi_sck   (ICE_SCK),
        .spi_mosi  (ICE_SI),
        .spi_miso  (ICE_SO),
        .spi_cs_n  (ICE_SSN),
        .reg_addr  (reg_addr),
        .reg_wdata (reg_wdata),
        .reg_wen   (reg_wen),
        .reg_rdata (reg_rdata),
        .reg_ren   (reg_ren),
        .reg_done  (reg_done)
    );

    // ================================================================
    // UART TX
    // ================================================================
    wire [7:0] uart_tx_data;
    wire       uart_tx_valid;
    wire       uart_tx_ready;

    uart_tx #(
        .CLK_FREQ (48_000_000),
        .BAUD     (3_000_000)
    ) u_uart_tx (
        .clk      (CLK),
        .rst_n    (rst_n),
        .tx_data  (uart_tx_data),
        .tx_valid (uart_tx_valid),
        .tx_ready (uart_tx_ready),
        .uart_txd (UART_TX)
    );

    // ================================================================
    // UART RX
    // ================================================================
    wire [7:0] uart_rx_data;
    wire       uart_rx_valid;
    wire       uart_rx_frame_err;

    uart_rx #(
        .CLK_FREQ (48_000_000),
        .BAUD     (3_000_000)
    ) u_uart_rx (
        .clk          (CLK),
        .rst_n        (rst_n),
        .rx_data      (uart_rx_data),
        .rx_valid     (uart_rx_valid),
        .rx_frame_err (uart_rx_frame_err),
        .uart_rxd     (UART_RX)
    );

    // ================================================================
    // Bridge Protocol Engine
    // ================================================================
    wire       engine_connected;
    wire [7:0] engine_tx_fifo_count;
    wire [7:0] engine_rx_fifo_count;

    bridge_engine #(
        .CLK_FREQ      (48_000_000),
        .PING_INTERVAL (48_000_000 * 2)  // 2s auto-ping
    ) u_engine (
        .clk            (CLK),
        .rst_n          (rst_n),
        .reg_addr       (reg_addr),
        .reg_wdata      (reg_wdata),
        .reg_wen        (reg_wen),
        .reg_rdata      (reg_rdata),
        .reg_ren        (reg_ren),
        .tx_data        (uart_tx_data),
        .tx_valid       (uart_tx_valid),
        .tx_ready       (uart_tx_ready),
        .rx_data        (uart_rx_data),
        .rx_valid       (uart_rx_valid),
        .connected      (engine_connected),
        .tx_fifo_count  (engine_tx_fifo_count),
        .rx_fifo_count  (engine_rx_fifo_count)
    );

    // ================================================================
    // LED Status (active low on pico2-ice)
    //   Green = connected, Red = disconnected, Blue = SPI activity
    // ================================================================
    reg [23:0] led_timer;
    reg        led_blink;
    reg [7:0]  spi_activity_cnt;

    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            led_timer        <= 0;
            led_blink        <= 0;
            spi_activity_cnt <= 0;
        end else begin
            // 1 Hz blink
            if (led_timer == 24'd12_000_000) begin
                led_timer <= 0;
                led_blink <= ~led_blink;
            end else begin
                led_timer <= led_timer + 1;
            end

            // SPI activity flash (decaying counter)
            if (reg_wen || reg_ren)
                spi_activity_cnt <= 8'hFF;
            else if (spi_activity_cnt > 0)
                spi_activity_cnt <= spi_activity_cnt - 1;
        end
    end

    // Active-low LEDs
    assign LED_G = engine_connected ? 1'b0 : 1'b1;    // Green on when connected
    assign LED_R = engine_connected ? 1'b1 : led_blink; // Red blinks when disconnected
    assign LED_B = (spi_activity_cnt > 0) ? 1'b0 : 1'b1; // Blue flashes on SPI activity

endmodule
