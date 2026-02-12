/**
 * SPI Target (Slave) - Register-mapped interface
 *
 * Implements a simple SPI slave that maps to an internal register bus.
 * Protocol: 
 *   Transaction: [R/W(1) + ADDR(7)] [DATA bytes...]
 *   Bit 7 of first byte: 1=read, 0=write
 *   Bits 6:0: register address (0x00-0x7F)
 *   Subsequent bytes: data (auto-increment address)
 *
 * SPI Mode 0: CPOL=0, CPHA=0
 *   - Data sampled on rising edge of SCK
 *   - Data shifted out on falling edge of SCK
 *
 * Directly compatible with the pico-ice-sdk SPI master.
 */
module spi_target (
    input  wire        clk,      // System clock (48 MHz)
    input  wire        rst_n,

    // SPI interface
    input  wire        spi_sck,
    input  wire        spi_mosi, // Master Out Slave In (SDI)
    output reg         spi_miso, // Master In Slave Out (SDO)
    input  wire        spi_cs_n, // Chip Select (active low)

    // Register bus interface
    output reg  [6:0]  reg_addr,
    output reg  [7:0]  reg_wdata,
    output reg         reg_wen,
    input  wire [7:0]  reg_rdata,
    output reg         reg_ren,  // Pulse when read address is valid
    output reg         reg_done  // Pulse when transaction complete
);

    // Synchronize SPI signals to system clock (3-stage for SCK edge detect)
    reg [2:0] sck_sync;
    reg [1:0] mosi_sync;
    reg [1:0] cs_sync;

    wire sck_rising  = (sck_sync[2:1] == 2'b01);
    wire sck_falling = (sck_sync[2:1] == 2'b10);
    wire cs_active   = !cs_sync[1];
    wire cs_deassert = (cs_sync == 2'b01);

    always @(posedge clk) begin
        sck_sync  <= {sck_sync[1:0], spi_sck};
        mosi_sync <= {mosi_sync[0], spi_mosi};
        cs_sync   <= {cs_sync[0], spi_cs_n};
    end

    // State
    reg [2:0]  bit_cnt;
    reg [7:0]  shift_in;
    reg [7:0]  shift_out;
    reg        first_byte;    // True during address/cmd byte
    reg        is_read;
    reg        addr_latched;  // Read data has been fetched for current addr

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt     <= 0;
            shift_in    <= 0;
            shift_out   <= 0;
            first_byte  <= 1;
            is_read     <= 0;
            addr_latched <= 0;
            reg_addr    <= 0;
            reg_wdata   <= 0;
            reg_wen     <= 0;
            reg_ren     <= 0;
            reg_done    <= 0;
            spi_miso    <= 0;
        end else begin
            reg_wen  <= 1'b0;
            reg_ren  <= 1'b0;
            reg_done <= 1'b0;

            if (!cs_active) begin
                // CS deasserted - reset for next transaction
                if (cs_deassert) begin
                    reg_done <= 1'b1;
                end
                bit_cnt     <= 0;
                first_byte  <= 1;
                is_read     <= 0;
                addr_latched <= 0;
            end else begin
                // Rising edge of SCK: sample MOSI
                if (sck_rising) begin
                    shift_in <= {shift_in[6:0], mosi_sync[1]};
                    bit_cnt  <= bit_cnt + 1;

                    if (bit_cnt == 3'd7) begin
                        // Full byte received
                        if (first_byte) begin
                            // Address/command byte
                            is_read  <= shift_in[6];  // Bit 7 (first shifted in) = R/W
                            reg_addr <= {shift_in[5:0], mosi_sync[1]};  // 7-bit address
                            first_byte <= 0;
                            // If read, fetch data now so it's ready for output
                            if (shift_in[6]) begin  // Read
                                reg_ren <= 1'b1;
                            end
                        end else begin
                            // Data byte
                            if (!is_read) begin
                                // Write: store received byte
                                reg_wdata <= {shift_in[6:0], mosi_sync[1]};
                                reg_wen   <= 1'b1;
                                reg_addr  <= reg_addr + 1;  // Auto-increment
                            end else begin
                                // Read: advance to next address
                                reg_addr  <= reg_addr + 1;
                                reg_ren   <= 1'b1;
                            end
                        end
                    end
                end

                // Falling edge of SCK: shift out MISO
                if (sck_falling) begin
                    if (bit_cnt == 0 && !first_byte && is_read && !addr_latched) begin
                        // Load read data for first data byte
                        shift_out    <= reg_rdata;
                        addr_latched <= 1;
                        spi_miso     <= reg_rdata[7];
                    end else if (bit_cnt == 0 && is_read) begin
                        // Load read data for subsequent bytes
                        shift_out <= reg_rdata;
                        spi_miso  <= reg_rdata[7];
                    end else begin
                        spi_miso  <= shift_out[7];
                        shift_out <= {shift_out[6:0], 1'b0};
                    end
                end
            end
        end
    end

endmodule
