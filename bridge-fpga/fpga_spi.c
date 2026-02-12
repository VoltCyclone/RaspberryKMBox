/**
 * FPGA SPI Communication Driver Implementation
 *
 * Uses pico-ice-sdk SPI bus (spi0) for register-level FPGA access.
 * After CRAM bitstream loading is complete, we reconfigure the SPI
 * pins for bidirectional register access at higher speed.
 */

#include "fpga_spi.h"
#include "bridge_fpga_regs.h"
#include "boards.h"

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include <string.h>
#include <stdio.h>

// SPI instance and pins (from pico2-ice board definition)
// These match the ice_fpga_data.c pico2_spibus configuration
#define FPGA_SPI       spi0
#define FPGA_SPI_SCK   6    // ICE_SCK
#define FPGA_SPI_MOSI  7    // ICE_SO (from FPGA perspective, this is RP2350 MOSI)
#define FPGA_SPI_MISO  4    // ICE_SI (from FPGA perspective, this is RP2350 MISO)
#define FPGA_SPI_CS    5    // ICE_SSN

// SPI speed for register access (after bitstream load)
// 10 MHz is safe for iCE40 UP5K SPI slave at 48 MHz clock
#define FPGA_SPI_BAUD  10000000

static bool spi_initialized = false;

bool fpga_spi_init(void) {
    // Initialize SPI at register-access speed
    uint actual = spi_init(FPGA_SPI, FPGA_SPI_BAUD);
    printf("[FPGA_SPI] Init at %u Hz (requested %u)\n", actual, FPGA_SPI_BAUD);

    // Configure SPI mode 0 (CPOL=0, CPHA=0)
    spi_set_format(FPGA_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Configure GPIO pins
    gpio_set_function(FPGA_SPI_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(FPGA_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(FPGA_SPI_MISO, GPIO_FUNC_SPI);

    // CS is manual GPIO (not hardware CS) for precise control
    gpio_init(FPGA_SPI_CS);
    gpio_set_dir(FPGA_SPI_CS, GPIO_OUT);
    gpio_put(FPGA_SPI_CS, 1);  // Deassert (active low)

    spi_initialized = true;
    
    // Verify FPGA is alive by reading version register
    uint8_t ver = fpga_reg_read(FPGA_REG_VERSION);
    printf("[FPGA_SPI] FPGA version: 0x%02X\n", ver);
    
    if (ver == 0x00 || ver == 0xFF) {
        printf("[FPGA_SPI] WARNING: FPGA may not be running (ver=0x%02X)\n", ver);
        return false;
    }
    
    return true;
}

// ============================================================================
// Low-level register access
// ============================================================================

void fpga_reg_write(uint8_t addr, uint8_t value) {
    uint8_t cmd = FPGA_SPI_WRITE(addr);
    
    gpio_put(FPGA_SPI_CS, 0);
    spi_write_blocking(FPGA_SPI, &cmd, 1);
    spi_write_blocking(FPGA_SPI, &value, 1);
    gpio_put(FPGA_SPI_CS, 1);
}

uint8_t fpga_reg_read(uint8_t addr) {
    uint8_t cmd = FPGA_SPI_READ(addr);
    uint8_t value = 0;
    
    gpio_put(FPGA_SPI_CS, 0);
    spi_write_blocking(FPGA_SPI, &cmd, 1);
    spi_read_blocking(FPGA_SPI, 0x00, &value, 1);
    gpio_put(FPGA_SPI_CS, 1);
    
    return value;
}

void fpga_reg_write_burst(uint8_t start_addr, const uint8_t *data, size_t len) {
    uint8_t cmd = FPGA_SPI_WRITE(start_addr);
    
    gpio_put(FPGA_SPI_CS, 0);
    spi_write_blocking(FPGA_SPI, &cmd, 1);
    spi_write_blocking(FPGA_SPI, data, len);
    gpio_put(FPGA_SPI_CS, 1);
}

void fpga_reg_read_burst(uint8_t start_addr, uint8_t *data, size_t len) {
    uint8_t cmd = FPGA_SPI_READ(start_addr);
    
    gpio_put(FPGA_SPI_CS, 0);
    spi_write_blocking(FPGA_SPI, &cmd, 1);
    spi_read_blocking(FPGA_SPI, 0x00, data, len);
    gpio_put(FPGA_SPI_CS, 1);
}

// ============================================================================
// Command submission helpers
// ============================================================================

static inline bool wait_tx_ready(void) {
    // Quick check - usually the FPGA FSM is idle
    for (int i = 0; i < 10; i++) {
        if (!(fpga_reg_read(FPGA_REG_TX_BUSY) & 0x01))
            return true;
    }
    return false;
}

bool fpga_send_mouse_move(int16_t x, int16_t y) {
    if (!wait_tx_ready()) return false;
    
    // Write X, Y, then command type (triggers send)
    uint8_t data[5] = {
        (uint8_t)(x & 0xFF),         // XL
        (uint8_t)((x >> 8) & 0xFF),  // XH
        (uint8_t)(y & 0xFF),         // YL
        (uint8_t)((y >> 8) & 0xFF),  // YH
    };
    
    // Burst write: REG_CMD_MOUSE_XL through REG_CMD_MOUSE_YH (4 bytes)
    fpga_reg_write_burst(FPGA_REG_CMD_MOUSE_XL, data, 4);
    
    // Write command type last → triggers TX FSM
    fpga_reg_write(FPGA_REG_CMD_TYPE, FPGA_CMD_MOUSE_MOVE);
    
    return true;
}

bool fpga_send_mouse_wheel(int8_t wheel) {
    if (!wait_tx_ready()) return false;
    
    fpga_reg_write(FPGA_REG_CMD_WHEEL, (uint8_t)wheel);
    fpga_reg_write(FPGA_REG_CMD_TYPE, FPGA_CMD_MOUSE_WHEEL);
    
    return true;
}

bool fpga_send_button_set(uint8_t mask, uint8_t state) {
    if (!wait_tx_ready()) return false;
    
    fpga_reg_write(FPGA_REG_CMD_BTN_MASK, mask);
    fpga_reg_write(FPGA_REG_CMD_BTN_STATE, state);
    fpga_reg_write(FPGA_REG_CMD_TYPE, FPGA_CMD_BUTTON_SET);
    
    return true;
}

bool fpga_send_mouse_move_wheel(int16_t x, int16_t y, int8_t wheel) {
    if (!wait_tx_ready()) return false;
    
    uint8_t data[5] = {
        (uint8_t)(x & 0xFF),
        (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF),
        (uint8_t)((y >> 8) & 0xFF),
        (uint8_t)wheel
    };
    
    // Burst write: XL, XH, YL, YH to cmd regs
    fpga_reg_write_burst(FPGA_REG_CMD_MOUSE_XL, data, 4);
    fpga_reg_write(FPGA_REG_CMD_WHEEL, data[4]);
    fpga_reg_write(FPGA_REG_CMD_TYPE, FPGA_CMD_MOUSE_MOVE_WHEEL);
    
    return true;
}

bool fpga_send_ping(void) {
    if (!wait_tx_ready()) return false;
    fpga_reg_write(FPGA_REG_CMD_TYPE, FPGA_CMD_PING);
    return true;
}

bool fpga_send_reset(void) {
    if (!wait_tx_ready()) return false;
    fpga_reg_write(FPGA_REG_CMD_TYPE, FPGA_CMD_RESET);
    return true;
}

// ============================================================================
// Status readback
// ============================================================================

void fpga_poll_status(fpga_bridge_status_t *status) {
    if (!status) return;
    
    // Burst read status registers 0x00-0x0A (11 bytes)
    uint8_t raw[11];
    fpga_reg_read_burst(FPGA_REG_STATUS, raw, sizeof(raw));
    
    status->connected    = (raw[0] & 0x01);
    status->conn_state   = raw[1];
    status->tx_pkt_count = (uint16_t)(raw[2] | (raw[3] << 8));
    status->rx_pkt_count = (uint16_t)(raw[4] | (raw[5] << 8));
    status->rx_err_count = (uint16_t)(raw[6] | (raw[7] << 8));
    status->last_rx_cmd  = raw[8];
    status->fpga_version = raw[9];
    status->tx_busy      = (raw[10] & 0x01);
    
    // Read RX FIFO count separately
    status->rx_fifo_count = fpga_reg_read(FPGA_REG_RX_FIFO_CNT);
}

bool fpga_is_connected(void) {
    return (fpga_reg_read(FPGA_REG_STATUS) & 0x01);
}

size_t fpga_drain_rx_fifo(uint8_t *buf, size_t maxlen) {
    uint8_t available = fpga_reg_read(FPGA_REG_RX_FIFO_CNT);
    if (available == 0) return 0;
    
    size_t to_read = (available < maxlen) ? available : maxlen;
    
    // Read bytes one at a time (each read pops from FIFO)
    for (size_t i = 0; i < to_read; i++) {
        buf[i] = fpga_reg_read(FPGA_REG_RX_FIFO_DATA);
    }
    
    return to_read;
}
