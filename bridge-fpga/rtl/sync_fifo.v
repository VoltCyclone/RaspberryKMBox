/**
 * Synchronous FIFO - Dual-port RAM based
 *
 * Parameterized depth and width for use in UART and command FIFOs.
 * Uses binary counter addressing (not gray code - single clock domain).
 */
module sync_fifo #(
    parameter WIDTH = 8,
    parameter DEPTH = 64,
    parameter ADDR_W = $clog2(DEPTH)
)(
    input  wire             clk,
    input  wire             rst_n,

    // Write interface
    input  wire [WIDTH-1:0] wr_data,
    input  wire             wr_en,
    output wire             full,

    // Read interface
    output wire [WIDTH-1:0] rd_data,
    input  wire             rd_en,
    output wire             empty,

    // Status
    output wire [ADDR_W:0]  count
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];

    reg [ADDR_W:0] wr_ptr;
    reg [ADDR_W:0] rd_ptr;

    assign count = wr_ptr - rd_ptr;
    assign full  = (count == DEPTH);
    assign empty = (wr_ptr == rd_ptr);

    // Read data is combinational for low latency
    assign rd_data = mem[rd_ptr[ADDR_W-1:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr[ADDR_W-1:0]] <= wr_data;
                wr_ptr <= wr_ptr + 1;
            end
            if (rd_en && !empty) begin
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

endmodule
