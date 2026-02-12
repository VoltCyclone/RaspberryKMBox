# FPGA Bridge — Implementation Guide

This document covers the bugs to fix and the architectural changes needed to make the
FPGA bridge deliver on its core promise: the RP2350 should never wait on UART timing.

## Current Bottleneck

The FPGA handles 3 Mbaud UART in gateware, but the RP2350 still experiences UART-speed
delays because:

1. **TX**: Only one command slot. The RP2350 must poll `REG_TX_BUSY` and wait for the
   previous packet to finish on the wire before submitting the next one.
2. **RX**: FIFO is read one byte per SPI transaction. 8 bytes = 8 separate CS
   assert/deassert cycles (~12.8us), slower than the UART received them.

Target: command submission in a single ~1us SPI burst, no polling. RX drain in a
single burst read regardless of byte count.

---

## Phase 0: Fix Synthesis Bugs

These must be fixed before any feature work. The current RTL has multi-driver
violations that produce undefined hardware.

### 0a. Consolidate multi-driven signals in `bridge_engine.v`

Three signals are assigned from multiple `always` blocks:

| Signal             | Block 1 (role)          | Block 2 (role)          |
|--------------------|-------------------------|-------------------------|
| `rx_fifo_rd`       | Register write (pop)    | RX parser (reset only)  |
| `ping_needed`      | TX FSM (clear)          | Ping timer (set+clear)  |
| `rx_activity_timer`| RX parser (reset on rx) | Conn state (increment)  |

**Fix**: Merge the three `always` blocks (register write, RX parser, connection state)
into two blocks with clean signal ownership:

- **Block A** — TX FSM. Owns: `tx_state`, `tx_packet[]`, `tx_pkt_len`, `tx_byte_idx`,
  `tx_data`, `tx_valid`, `tx_pkt_count`. No changes needed, this block is already clean.

- **Block B** — Everything else. Merge register writes, RX parsing, connection state,
  and ping timer into one block. This block owns: `cmd_*`, `cmd_pending`, `rx_fifo_rd`,
  `rx_fifo_wr`, `rx_bin_*`, `rx_pkt_count`, `rx_err_count`, `last_rx_cmd`, `connected`,
  `conn_state`, `ping_timer`, `ping_needed`, `rx_activity_timer`.

The merged block structure:

```verilog
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset ALL signals owned by this block
    end else begin
        // 1. Default pulse signals
        // 2. Ping timer logic (set ping_needed)
        // 3. Connection timeout logic (increment/reset rx_activity_timer)
        // 4. Register write handling (cmd regs, FIFO pop)
        // 5. RX byte processing (FIFO push, binary packet detection)
        //    - reset rx_activity_timer on rx_valid
        //    - increment rx_err_count on frame error
        // 6. Clear cmd_pending when TX FSM consumes it
        // 7. Clear ping_needed when TX FSM consumes it
    end
end
```

For items 6 and 7, the merged block reads `tx_state` (from Block A) but does not
write it. This is a clean cross-block read with no multi-driver conflict.

### 0b. Wire UART frame errors

Add `rx_frame_err` input to `bridge_engine`:

```verilog
// bridge_engine port list:
input  wire        rx_frame_err,

// In the merged RX handling section:
if (rx_frame_err)
    rx_err_count <= rx_err_count + 1;
```

In `top.v`, connect `uart_rx_frame_err` to the engine:

```verilog
bridge_engine u_engine (
    ...
    .rx_frame_err   (uart_rx_frame_err),
    ...
);
```

Also count RX timeout resets (when `rx_bin_active` is cleared by timeout):

```verilog
if (rx_bin_active && rx_timeout_cnt >= (CLK_FREQ / 1000)) begin
    rx_err_count <= rx_err_count + 1;  // count as error
    rx_bin_active <= 0;
    ...
end
```

### 0c. Remove unused `sync_fifo.v` from synthesis

In `CMakeLists.txt`, remove `${RTL_DIR}/sync_fifo.v` from `RTL_SOURCES`. The module
is never instantiated and wastes iCE40 LUTs. Keep the file in the repo for future use.

---

## Phase 1: TX Command FIFO

Replace the single `cmd_pending` slot with a small command FIFO so the RP2350 can
submit multiple commands back-to-back without polling.

### 1a. Define the TX command entry

Each queued command needs: type (8b), mouse_x (16b), mouse_y (16b), wheel (8b),
btn_mask (8b), btn_state (8b) = 8 bytes total.

```verilog
// TX command FIFO (4 entries, 64 bits each)
localparam TX_FIFO_DEPTH = 4;
localparam TX_FIFO_AW    = 2;  // $clog2(4)

reg [63:0] tx_cmd_fifo [0:TX_FIFO_DEPTH-1];
reg [TX_FIFO_AW:0] tx_cmd_wr;
reg [TX_FIFO_AW:0] tx_cmd_rd;
wire [TX_FIFO_AW:0] tx_cmd_count = tx_cmd_wr - tx_cmd_rd;
wire tx_cmd_full  = (tx_cmd_count == TX_FIFO_DEPTH);
wire tx_cmd_empty = (tx_cmd_wr == tx_cmd_rd);
```

Pack/unpack format:

```
[63:56] = cmd_type
[55:48] = btn_state
[47:40] = btn_mask
[39:32] = wheel
[31:16] = mouse_y
[15:0]  = mouse_x
```

### 1b. Push on REG_CMD_TYPE write

When the RP2350 writes `REG_CMD_TYPE`, push the current command registers into the
FIFO instead of setting `cmd_pending`:

```verilog
REG_CMD_TYPE: begin
    if (!tx_cmd_full) begin
        tx_cmd_fifo[tx_cmd_wr[TX_FIFO_AW-1:0]] <= {
            reg_wdata,          // cmd_type   [63:56]
            cmd_btn_state,      // btn_state  [55:48]
            cmd_btn_mask,       // btn_mask   [47:40]
            cmd_wheel,          // wheel      [39:32]
            cmd_mouse_y,        // mouse_y    [31:16]
            cmd_mouse_x         // mouse_x    [15:0]
        };
        tx_cmd_wr <= tx_cmd_wr + 1;
    end
end
```

### 1c. TX FSM pops from FIFO

Replace the `cmd_pending` check in `TX_IDLE` with a FIFO pop:

```verilog
TX_IDLE: begin
    tx_valid <= 1'b0;
    if (!tx_cmd_empty) begin
        // Pop command from FIFO
        wire [63:0] cmd = tx_cmd_fifo[tx_cmd_rd[TX_FIFO_AW-1:0]];
        tx_cmd_rd <= tx_cmd_rd + 1;

        // Unpack and build packet (same switch on cmd_type as today)
        automatic reg [7:0] ctype = cmd[63:56];
        tx_packet[0] <= SYNC_BYTE;
        case (ctype)
            CMD_MOUSE_MOVE: begin
                tx_packet[1] <= CMD_MOUSE_MOVE;
                tx_packet[2] <= cmd[7:0];    // x_lo
                tx_packet[3] <= cmd[15:8];   // x_hi
                tx_packet[4] <= cmd[23:16];  // y_lo  (mouse_y[7:0] at bits [23:16])
                tx_packet[5] <= cmd[31:24];  // y_hi
                tx_pkt_len   <= 6;
            end
            // ... same for other command types
        endcase
        tx_byte_idx <= 0;
        tx_state    <= TX_SYNC;
    end else if (ping_needed) begin
        // Auto-ping (unchanged)
        ...
    end
end
```

### 1d. Update REG_TX_BUSY semantics

`REG_TX_BUSY` should now report whether the FIFO is full (i.e., whether the RP2350
can submit another command), not just whether the TX FSM is active:

```verilog
REG_TX_BUSY: reg_rdata = {6'b0, tx_cmd_full, (tx_state != TX_IDLE)};
```

Bit 0 = TX FSM active, Bit 1 = FIFO full. The RP2350 only needs to check bit 1
before submitting (and in practice, with 4 slots, it almost never will be full).

### 1e. Add REG_TX_FIFO_FREE register

Add a new status register so the RP2350 can check available slots:

```verilog
// bridge_regs.vh
localparam REG_TX_FIFO_FREE = 7'h0B;  // Slots available in TX FIFO

// bridge_engine.v read logic
REG_TX_FIFO_FREE: reg_rdata = TX_FIFO_DEPTH - tx_cmd_count;
```

### 1f. Update RP2350 C driver

Remove `wait_tx_ready()` polling. Commands become fire-and-forget:

```c
bool fpga_send_mouse_move(int16_t x, int16_t y) {
    // Single burst write: XL, XH, YL, YH, (skip 0x14-0x16), TYPE
    // Or write just the needed regs + type
    uint8_t data[] = {
        (uint8_t)(x & 0xFF), (uint8_t)(x >> 8),
        (uint8_t)(y & 0xFF), (uint8_t)(y >> 8),
    };
    fpga_reg_write_burst(FPGA_REG_CMD_MOUSE_XL, data, 4);
    fpga_reg_write(FPGA_REG_CMD_TYPE, FPGA_CMD_MOUSE_MOVE);
    return true;  // FIFO absorbs it; only fails if 4+ commands queued
}
```

For maximum throughput, use a single 8-byte burst from `0x10` to `0x17`:

```c
bool fpga_send_mouse_move(int16_t x, int16_t y) {
    uint8_t data[8] = {
        x & 0xFF, x >> 8,           // 0x10-0x11: mouse X
        y & 0xFF, y >> 8,           // 0x12-0x13: mouse Y
        0,                          // 0x14: wheel (unused)
        0, 0,                       // 0x15-0x16: btn mask/state (unused)
        FPGA_CMD_MOUSE_MOVE         // 0x17: type (triggers FIFO push)
    };
    fpga_reg_write_burst(FPGA_REG_CMD_MOUSE_XL, data, 8);
    return true;
}
```

This is a single SPI transaction: 1 byte command + 8 bytes data = 9 bytes = 72 SPI
clocks = **~7.2us at 10 MHz**. Compare to the current implementation which does
a burst(4) + single write + polling = ~10-20us.

---

## Phase 2: Burst-Readable RX FIFO

### 2a. Suppress auto-increment on FIFO data register

In `spi_target.v`, the address auto-increments after every byte in a burst
read/write. For the RX FIFO data register, we want repeated reads from the same
address (each read pops the next byte).

Add a `no_increment` signal from the engine, or hardcode the exception:

```verilog
// In spi_target.v, where address auto-increments on read:
if (is_read) begin
    reg_ren <= 1'b1;
    // Don't auto-increment on FIFO data register
    if (reg_addr != 7'h21)  // REG_RX_FIFO_DATA
        reg_addr <= reg_addr + 1;
end

// Same for write auto-increment (to be safe):
if (!is_read) begin
    reg_wdata <= {shift_in[6:0], mosi_sync[1]};
    reg_wen   <= 1'b1;
    if (reg_addr != 7'h21)
        reg_addr <= reg_addr + 1;
end
```

Note: hardcoding `7'h21` couples spi_target to the register map. Cleaner alternative:
add a `fifo_addr` parameter to spi_target, or have bridge_engine export a
`no_increment` wire that spi_target samples. The hardcoded approach is simpler and
the register map is unlikely to change.

### 2b. Update RP2350 FIFO drain to use burst read

```c
size_t fpga_drain_rx_fifo(uint8_t *buf, size_t maxlen) {
    uint8_t available = fpga_reg_read(FPGA_REG_RX_FIFO_CNT);
    if (available == 0) return 0;

    size_t to_read = (available < maxlen) ? available : maxlen;

    // Single burst read — SPI address stays at 0x21, FPGA pops on each byte
    fpga_reg_read_burst(FPGA_REG_RX_FIFO_DATA, buf, to_read);

    return to_read;
}
```

8 bytes now takes: 1 byte cmd + 8 bytes data = 72 SPI clocks = **~7.2us**, down
from ~12.8us (8 separate transactions).

---

## Phase 3: IRQ Pin (Optional)

Add an active-low interrupt output from the FPGA to an RP2350 GPIO. This eliminates
polling entirely — the RP2350 only reads the FPGA when there's something to do.

### 3a. RTL changes

In `top.v`, add an output pin:

```verilog
output wire IRQ_N    // Active-low interrupt to RP2350
```

Drive it based on actionable conditions:

```verilog
// IRQ asserted when:
//   - RX FIFO has data (KMBox sent a response)
//   - Connection state changed
//   - TX FIFO has room (if RP2350 was waiting)
assign IRQ_N = (engine_rx_fifo_count == 0) ? 1'b1 : 1'b0;
```

Or for more granularity, add an interrupt enable/status register pair:

```verilog
// bridge_regs.vh
localparam REG_IRQ_ENABLE = 7'h30;  // [0]=rx_fifo_nonempty [1]=conn_change
localparam REG_IRQ_STATUS = 7'h31;  // Same bits, write-1-to-clear

// bridge_engine.v
reg [7:0] irq_enable;
reg [7:0] irq_status;
wire irq_active = |(irq_enable & irq_status);
```

### 3b. Pin constraint

Add to `pico2_ice_bridge.pcf` — pick a free PMOD pin:

```
# IRQ output to RP2350 (active low)
set_io -nowarn IRQ_N    3    # PMOD_A_TOP_IO3
```

### 3c. RP2350 firmware

Wire the GPIO to an interrupt handler or just use it as a fast poll gate:

```c
#define FPGA_IRQ_PIN  XX  // Whichever GPIO connects to the PMOD pin

// In main loop — skip SPI entirely if no interrupt:
static void fpga_rx_task(void) {
    if (gpio_get(FPGA_IRQ_PIN))  // IRQ_N is high = no data
        return;
    // ... burst drain FIFO
}
```

This reduces idle SPI traffic to zero.

---

## Phase 4: Minor Improvements

### 4a. SPI MISO tri-state

The SPI bus is shared with CRAM loading. After bitstream load, the user design should
only drive MISO when selected. In `top.v`, replace the direct `spi_miso` output with
a tri-state `SB_IO`:

```verilog
wire spi_miso_int;

// In spi_target, output to spi_miso_int instead of ICE_SO directly.
// Then in top.v:
SB_IO #(
    .PIN_TYPE(6'b1010_01)  // output=registered, enable=simple
) u_miso_tristate (
    .PACKAGE_PIN   (ICE_SO),
    .OUTPUT_ENABLE (~cs_sync[1]),  // drive only when CS active
    .D_OUT_0       (spi_miso_int)
);
```

This requires splitting the `spi_miso` output in `spi_target.v` from the top-level
`ICE_SO` pad.

### 4b. Remap REG_RX_FIFO_CNT for burst status reads

Currently `REG_RX_FIFO_CNT` is at `0x20`, with a 22-register gap from the status
block (`0x00-0x0A`). `fpga_poll_status()` does a burst read of 11 bytes then a
separate single read.

Add an alias at `0x0B`:

```verilog
// bridge_regs.vh
localparam REG_RX_FIFO_CNT_ALIAS = 7'h0B;

// bridge_engine.v read mux
REG_RX_FIFO_CNT_ALIAS: reg_rdata = {1'b0, rx_fifo_cnt};
```

Then `fpga_poll_status()` becomes a single 12-byte burst read of `0x00-0x0B`.

### 4c. Throttle `fpga_rx_task` in main.c

The `RX_DRAIN_MS` constant is defined (10ms) but never used. Add a timer gate:

```c
static void fpga_rx_task(void) {
    static uint32_t last_drain_ms = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_drain_ms < RX_DRAIN_MS) return;
    last_drain_ms = now;

    uint8_t rx_buf[64];
    size_t n = fpga_drain_rx_fifo(rx_buf, sizeof(rx_buf));
    // ...
}
```

With the Phase 3 IRQ pin, this timer becomes unnecessary — just check the pin.

---

## Summary: Expected Latency After Changes

| Operation | Current | After Phase 1+2 |
|-----------|---------|-----------------|
| Mouse move submit | ~15-20us (poll + 2 SPI txns) | ~7us (1 burst write) |
| Combined move+wheel | ~20-25us (poll + 3 SPI txns) | ~7us (1 burst write) |
| Back-to-back commands | blocked by UART (23us/pkt) | no wait (FIFO absorbs 4) |
| RX drain 8 bytes | ~13us (8 SPI txns) | ~7us (1 burst read) |
| RX drain idle check | ~1.6us (read count reg) | 0 (IRQ pin, Phase 3) |
| Status poll | ~3us (burst + 1 extra) | ~2us (single burst, 4b) |

## Implementation Order

1. **Phase 0** — Fix multi-driver bugs and wire frame errors. Required for correct
   synthesis. Test by building with yosys and checking for warnings.
2. **Phase 1** — TX command FIFO. Biggest user-facing improvement: eliminates all
   TX-side polling.
3. **Phase 2** — Burst RX FIFO. Simple change with immediate measurable improvement.
4. **Phase 3** — IRQ pin. Eliminates idle SPI traffic. Requires a physical wire
   between PMOD and RP2350 GPIO, so verify board routing first.
5. **Phase 4** — Cleanup (tri-state, register remapping, timer gate). Low risk,
   do whenever convenient.
