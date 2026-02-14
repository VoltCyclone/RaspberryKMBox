/**
 * SPI Target (Slave) - Mode 0, 64-bit Fixed Transactions
 *
 * Receives 8-byte (64-bit) commands from an SPI master (RP2350 PIO) and
 * simultaneously returns 8-byte status on MISO. SPI Mode 0: CPOL=0, CPHA=0
 * (data sampled on SCK rising edge, shifted out on SCK falling edge).
 *
 * Designed for iCE40 UP5K at 48 MHz system clock with 12 MHz SPI clock.
 * At 12 MHz SPI / 48 MHz sys, each SPI half-period is 4 system clocks.
 * The 2-FF synchronizer adds 2 clocks latency, leaving 2 clocks margin
 * for combinational logic — sufficient for reliable operation.
 *
 * Transaction protocol:
 *   1. Master asserts CS_N low
 *   2. FPGA latches resp_data into MISO shift register
 *   3. Master clocks 64 bits (8 bytes) MSB first on MOSI/MISO
 *   4. Master deasserts CS_N high
 *   5. FPGA asserts cmd_valid for one clock with received data
 *   6. FPGA asserts txn_done for one clock
 */
module spi_target (
    input  wire        clk,        // 48 MHz system clock
    input  wire        rst_n,      // Active-low reset

    // SPI pins (directly from pad)
    input  wire        spi_sck,    // SPI clock from master
    input  wire        spi_cs_n,   // Chip select (active low)
    input  wire        spi_mosi,   // Data from master
    output wire        spi_miso,   // Data to master

    // Internal interface
    output reg  [63:0] cmd_data,   // 8-byte command received
    output reg         cmd_valid,  // Pulse high for one clk when complete frame received
    input  wire [63:0] resp_data,  // 8-byte response to send (latched at CS falling edge)

    // Transaction complete signal (for RX FIFO pop timing)
    output reg         txn_done    // Pulse when CS deasserts after valid transaction
);

    // ================================================================
    // 2-FF Synchronizers (metastability guard for all SPI inputs)
    //
    // All three SPI signals cross from the 12 MHz SPI domain into the
    // 48 MHz system domain. Two flip-flop stages reduce MTBF of
    // metastable events to negligible levels for iCE40 UP5K.
    // ================================================================
    reg [1:0] sck_sync;
    reg [1:0] cs_n_sync;
    reg [1:0] mosi_sync;

    wire sck_s  = sck_sync[1];     // Synchronized SCK
    wire cs_n_s = cs_n_sync[1];    // Synchronized CS_N
    wire mosi_s = mosi_sync[1];    // Synchronized MOSI

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_sync  <= 2'b00;    // SCK idles low in Mode 0 (CPOL=0)
            cs_n_sync <= 2'b11;    // CS_N idles high (deasserted)
            mosi_sync <= 2'b00;
        end else begin
            sck_sync  <= {sck_sync[0],  spi_sck};
            cs_n_sync <= {cs_n_sync[0], spi_cs_n};
            mosi_sync <= {mosi_sync[0], spi_mosi};
        end
    end

    // ================================================================
    // Edge Detectors
    //
    // Previous-cycle value of synchronized signals, used to detect
    // rising and falling edges. Each edge detector is valid for
    // exactly one system clock cycle.
    // ================================================================
    reg sck_prev;
    reg cs_n_prev;

    wire sck_rising  = ( sck_s && !sck_prev);   // SCK 0→1
    wire sck_falling = (!sck_s &&  sck_prev);   // SCK 1→0
    wire cs_n_fall   = (!cs_n_s &&  cs_n_prev); // CS_N 1→0 (assert)
    wire cs_n_rise   = ( cs_n_s && !cs_n_prev); // CS_N 0→1 (deassert)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_prev  <= 1'b0;    // SCK idles low (CPOL=0)
            cs_n_prev <= 1'b1;    // CS_N idles high
        end else begin
            sck_prev  <= sck_s;
            cs_n_prev <= cs_n_s;
        end
    end

    // ================================================================
    // Bit Counter
    //
    // Counts the number of bits transferred in the current transaction.
    // Reset to 0 on CS_N falling edge, incremented on each SCK rising
    // edge. A complete transaction is 64 bits (bit_counter == 64 after
    // the last rising edge). Uses 7 bits to hold values 0..64.
    // ================================================================
    reg [6:0] bit_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_counter <= 7'd0;
        end else begin
            if (cs_n_fall) begin
                // CS just asserted — start of new transaction
                bit_counter <= 7'd0;
            end else if (sck_rising && !cs_n_s) begin
                // Active transaction: count each rising edge
                bit_counter <= bit_counter + 7'd1;
            end
        end
    end

    // ================================================================
    // MOSI Capture Shift Register (Master → Slave data)
    //
    // On each SCK rising edge (Mode 0 sample point), the MOSI bit is
    // shifted into the LSB while existing bits move left. After 64
    // rising edges, mosi_shift[63:0] contains the complete command
    // with the first bit received in bit 63 (MSB first).
    // ================================================================
    reg [63:0] mosi_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mosi_shift <= 64'd0;
        end else begin
            if (cs_n_fall) begin
                // Clear at start of transaction for clean capture
                mosi_shift <= 64'd0;
            end else if (sck_rising && !cs_n_s) begin
                // Shift in new MOSI bit at MSB-first position:
                // existing data shifts left, new bit enters at LSB
                mosi_shift <= {mosi_shift[62:0], mosi_s};
            end
        end
    end

    // ================================================================
    // MISO Output Shift Register (Slave → Master data)
    //
    // Latches resp_data on CS_N falling edge. On each SCK falling edge
    // (Mode 0 output change point), shifts left to present the next
    // bit on MISO. The MSB (bit 63) is driven first.
    //
    // After latching, the MSB is immediately available on spi_miso
    // before the first SCK rising edge, satisfying Mode 0 timing:
    // data must be valid before the first rising edge.
    // ================================================================
    reg [63:0] miso_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            miso_shift <= 64'd0;
        end else begin
            if (cs_n_fall) begin
                // Latch response data at the start of transaction
                miso_shift <= resp_data;
            end else if (sck_falling && !cs_n_s) begin
                // Shift out next bit (MSB first): move left, fill LSB with 0
                miso_shift <= {miso_shift[62:0], 1'b0};
            end
        end
    end

    // ================================================================
    // MISO Output Driver
    //
    // When CS_N is asserted (low), drive the MSB of the MISO shift
    // register. When CS_N is deasserted (high), drive 0 — the master
    // ignores MISO when CS is inactive, and iCE40 does not support
    // true tri-state on general I/O pins.
    // ================================================================
    assign spi_miso = (!cs_n_s) ? miso_shift[63] : 1'b0;

    // ================================================================
    // Command Output & Transaction Done
    //
    // On CS_N rising edge (end of transaction), if exactly 64 bits
    // were transferred, latch the captured MOSI data into cmd_data
    // and pulse cmd_valid and txn_done for one system clock cycle.
    //
    // Both cmd_valid and txn_done are single-cycle pulses: they are
    // cleared in all other branches to prevent latches.
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_data  <= 64'd0;
            cmd_valid <= 1'b0;
            txn_done  <= 1'b0;
        end else begin
            // Default: deassert pulses (prevents latches)
            cmd_valid <= 1'b0;
            txn_done  <= 1'b0;

            if (cs_n_rise) begin
                // CS just deasserted — check if we got a full 64-bit frame
                if (bit_counter == 7'd64) begin
                    cmd_data  <= mosi_shift;
                    cmd_valid <= 1'b1;
                end
                // Always signal transaction end (even for short/partial frames)
                txn_done <= 1'b1;
            end
        end
    end

endmodule
