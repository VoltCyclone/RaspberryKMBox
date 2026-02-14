/**
 * FPGA UART Communication Driver Implementation
 *
 * After CRAM bitstream loading completes (using SPI0 on GPIO 4-7),
 * this module reconfigures GPIO 4/5 as UART1 for bridge protocol
 * communication with the FPGA, and GPIO 7 as a connected status input.
 */

#include "fpga_uart.h"
#include "../bridge_protocol.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#include <string.h>
#include <stdio.h>

static bool uart_initialized = false;

bool fpga_uart_init(void) {
    // Deinit SPI0 — it was used for CRAM bitstream loading on GPIO 4-7.
    // After CRAM close, the FPGA is running and these pins become user I/O.
    spi_deinit(spi0);

    // Release GPIO 4-7 from SPI function
    gpio_set_function(4, GPIO_FUNC_NULL);
    gpio_set_function(5, GPIO_FUNC_NULL);
    gpio_set_function(6, GPIO_FUNC_NULL);
    gpio_set_function(7, GPIO_FUNC_NULL);

    // Initialize UART1 on GPIO 4 (TX) and GPIO 5 (RX)
    uint actual_baud = uart_init(FPGA_UART_INST, FPGA_UART_BAUD);
    printf("[FPGA_UART] Init at %u baud (requested %u)\n", actual_baud, FPGA_UART_BAUD);

    // 8N1 format
    uart_set_format(FPGA_UART_INST, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(FPGA_UART_INST, false, false);

    // Enable FIFO for buffering
    uart_set_fifo_enabled(FPGA_UART_INST, true);

    // Set GPIO functions
    gpio_set_function(FPGA_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FPGA_UART_RX_PIN, GPIO_FUNC_UART);

    // Configure GPIO 7 as input for FPGA connected status
    // (iCE40 pin 14 drives this — active high when KMBox is connected)
    gpio_init(FPGA_CONNECTED_PIN);
    gpio_set_dir(FPGA_CONNECTED_PIN, GPIO_IN);
    gpio_pull_down(FPGA_CONNECTED_PIN);

    uart_initialized = true;

    // Drain any garbage from UART RX FIFO
    while (uart_is_readable(FPGA_UART_INST))
        uart_getc(FPGA_UART_INST);

    printf("[FPGA_UART] Ready (GPIO %d TX, GPIO %d RX, GPIO %d connected)\n",
           FPGA_UART_TX_PIN, FPGA_UART_RX_PIN, FPGA_CONNECTED_PIN);
    return true;
}

// ============================================================================
// Command submission — builds bridge protocol packets and sends via UART1
// ============================================================================

bool fpga_send_mouse_move(int16_t x, int16_t y) {
    if (!uart_initialized) return false;
    uint8_t buf[6];
    size_t len = bridge_build_mouse_move(buf, x, y);
    uart_write_blocking(FPGA_UART_INST, buf, len);
    return true;
}

bool fpga_send_mouse_wheel(int8_t wheel) {
    if (!uart_initialized) return false;
    uint8_t buf[3];
    size_t len = bridge_build_mouse_wheel(buf, wheel);
    uart_write_blocking(FPGA_UART_INST, buf, len);
    return true;
}

bool fpga_send_button_set(uint8_t mask, uint8_t state) {
    if (!uart_initialized) return false;
    uint8_t buf[4];
    size_t len = bridge_build_button_set(buf, mask, state);
    uart_write_blocking(FPGA_UART_INST, buf, len);
    return true;
}

bool fpga_send_mouse_move_wheel(int16_t x, int16_t y, int8_t wheel) {
    if (!uart_initialized) return false;
    uint8_t buf[7];
    size_t len = bridge_build_mouse_move_wheel(buf, x, y, wheel);
    uart_write_blocking(FPGA_UART_INST, buf, len);
    return true;
}

bool fpga_send_ping(void) {
    if (!uart_initialized) return false;
    uint8_t buf[2];
    size_t len = bridge_build_ping(buf);
    uart_write_blocking(FPGA_UART_INST, buf, len);
    return true;
}

bool fpga_send_reset(void) {
    if (!uart_initialized) return false;
    uint8_t buf[2];
    size_t len = bridge_build_reset(buf);
    uart_write_blocking(FPGA_UART_INST, buf, len);
    return true;
}

// ============================================================================
// RX readback
// ============================================================================

size_t fpga_read_rx(uint8_t *buf, size_t maxlen) {
    if (!uart_initialized || !buf || maxlen == 0) return 0;

    size_t n = 0;
    while (n < maxlen && uart_is_readable(FPGA_UART_INST)) {
        buf[n++] = uart_getc(FPGA_UART_INST);
    }
    return n;
}

bool fpga_is_connected(void) {
    return gpio_get(FPGA_CONNECTED_PIN);
}
