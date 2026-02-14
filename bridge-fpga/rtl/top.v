/**
 * KMBox FPGA Bridge - Top Level Module (SPI)
 *
 * iCE40 UP5K FPGA that relays SPI traffic between the RP2350 and KMBox,
 * with autonomous auto-ping keepalive and connection tracking.
 *
 * Architecture:
 *   RP2350 ←(SPI 12M)→ iCE40 FPGA ←(SPI 12M)→ KMBox RP2350
 *
 * The RP2350 sends 8-byte KMBox fast binary packets via SPI (PIO2 master).
 * The FPGA receives them on an SPI target (slave), buffers in a FIFO,
 * and forwards them to KMBox via a second SPI master on PMOD A.
 * The FPGA provides the clock for the KMBox link — deterministic timing.
 * Auto-ping runs independently in the FPGA when the RP2350 is idle.
 *
 * Pin Usage (pico2-ice board):
 *   Bridge SPI: iCE40 pins 14-17 (connected to RP2350 GPIO 4-7)
 *   KMBox SPI: PMOD A pins 2/4/47/45
 *   LEDs: RGB status via on-board active-low LEDs
 */
module top (
    input  wire CLK,            // 48 MHz from RP2350

    // Bridge SPI (SPI target, RP2350 is master)
    input  wire SPI_MOSI,       // iCE40 pin 14 ← RP2350 GPIO 7
    input  wire SPI_SCK,        // iCE40 pin 15 ← RP2350 GPIO 6
    input  wire SPI_CS_N,       // iCE40 pin 16 ← RP2350 GPIO 5
    output wire SPI_MISO,       // iCE40 pin 17 → RP2350 GPIO 4

    // KMBox SPI (SPI master, FPGA drives clock)
    output wire SPI_KM_SCK,     // iCE40 pin 2 → KMBox SCK
    output wire SPI_KM_MOSI,    // iCE40 pin 4 → KMBox MOSI
    output wire SPI_KM_CS_N,    // iCE40 pin 47 → KMBox CS_N
    input  wire SPI_KM_MISO,    // iCE40 pin 45 ← KMBox MISO

    // Status LEDs (active low on pico2-ice)
    output wire LED_R,
    output wire LED_G,
    output wire LED_B
);

    // ================================================================
    // Power-on reset (256 clocks at 48 MHz ≈ 5.3 us)
    // ================================================================
    reg [7:0] rst_cnt = 0;
    wire rst_n = rst_cnt[7];

    always @(posedge CLK) begin
        if (!rst_n)
            rst_cnt <= rst_cnt + 1;
    end

    // ================================================================
    // SPI Target (Bridge RP2350 → FPGA)
    //
    // Receives 8-byte commands from RP2350 PIO SPI master.
    // Returns 8-byte status on MISO simultaneously.
    // ================================================================
    wire [63:0] tgt_cmd_data;
    wire        tgt_cmd_valid;
    wire        tgt_txn_done;

    spi_target u_spi_target (
        .clk       (CLK),
        .rst_n     (rst_n),
        // SPI pins
        .spi_sck   (SPI_SCK),
        .spi_cs_n  (SPI_CS_N),
        .spi_mosi  (SPI_MOSI),
        .spi_miso  (SPI_MISO),
        // Internal interface
        .cmd_data  (tgt_cmd_data),
        .cmd_valid (tgt_cmd_valid),
        .resp_data (engine_resp_data),
        .txn_done  (tgt_txn_done)
    );

    // ================================================================
    // SPI Master (FPGA → KMBox RP2350)
    //
    // Sends 8-byte packets to KMBox at 12 MHz (FPGA-generated clock).
    // ================================================================
    wire [63:0] km_tx_data;
    wire        km_tx_valid;
    wire        km_tx_ready;
    wire [63:0] km_rx_data;
    wire        km_rx_valid;

    spi_master u_spi_master (
        .clk       (CLK),
        .rst_n     (rst_n),
        // SPI pins
        .spi_sck   (SPI_KM_SCK),
        .spi_cs_n  (SPI_KM_CS_N),
        .spi_mosi  (SPI_KM_MOSI),
        .spi_miso  (SPI_KM_MISO),
        // Internal interface
        .tx_data   (km_tx_data),
        .tx_valid  (km_tx_valid),
        .tx_ready  (km_tx_ready),
        .rx_data   (km_rx_data),
        .rx_valid  (km_rx_valid)
    );

    // ================================================================
    // Bridge Engine (SPI→SPI FIFO relay + auto-ping + connection tracking)
    // ================================================================
    wire [63:0] engine_resp_data;
    wire        engine_connected;
    wire        engine_activity;

    bridge_engine_spi #(
        .CLK_FREQ      (48_000_000),
        .PING_INTERVAL (48_000_000 * 2),  // 2s auto-ping
        .TIMEOUT_CLKS  (48_000_000 * 5)   // 5s connection timeout
    ) u_engine (
        .clk         (CLK),
        .rst_n       (rst_n),
        // From SPI target (bridge RP2350 commands)
        .cmd_data    (tgt_cmd_data),
        .cmd_valid   (tgt_cmd_valid),
        .txn_done    (tgt_txn_done),
        // To/from SPI master (KMBox output)
        .km_tx_data  (km_tx_data),
        .km_tx_valid (km_tx_valid),
        .km_tx_ready (km_tx_ready),
        .km_rx_data  (km_rx_data),
        .km_rx_valid (km_rx_valid),
        // Response data for SPI target MISO
        .resp_data   (engine_resp_data),
        // Status
        .connected   (engine_connected),
        .activity    (engine_activity)
    );

    // ================================================================
    // FPGA RGB LED Status (active low, iCE40 pins 39/40/41)
    //
    // Separate physical LED from the Pico RGB LED (RP2350 GPIO 0/1/9).
    // Both LEDs show independent status simultaneously.
    //
    //   Green (pin 39) = KMBox SPI connected (solid)
    //   Red   (pin 41) = KMBox SPI disconnected (blinks 1 Hz)
    //   Blue  (pin 40) = SPI activity (brief flash)
    // ================================================================
    reg [23:0] led_timer;
    reg        led_blink;
    reg        led_tick;   // Registered terminal count to avoid wide comparator

    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            led_timer <= 0;
            led_blink <= 0;
            led_tick  <= 0;
        end else begin
            led_tick <= (led_timer == 24'd23_999_999);
            if (led_tick) begin
                led_timer <= 0;
                led_blink <= ~led_blink;
            end else begin
                led_timer <= led_timer + 1;
            end
        end
    end

    // Register LED outputs to avoid combinational paths to IO pads
    //   Green = KMBox SPI connected (solid)
    //   Red   = KMBox SPI disconnected (blinks 1 Hz)
    //   Blue  = bridge RP2350 SPI activity
    reg led_r_reg, led_g_reg, led_b_reg;
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            led_r_reg <= 1'b1;
            led_g_reg <= 1'b1;
            led_b_reg <= 1'b1;
        end else begin
            led_g_reg <= engine_connected ? 1'b0 : 1'b1;   // Green ON when connected
            led_r_reg <= engine_connected ? 1'b1 :          // Red OFF when connected
                         (led_blink ? 1'b0 : 1'b1);        // Red blinks when disconnected
            led_b_reg <= engine_activity  ? 1'b0 : 1'b1;   // Blue ON during activity
        end
    end

    assign LED_G = led_g_reg;
    assign LED_R = led_r_reg;
    assign LED_B = led_b_reg;

endmodule
