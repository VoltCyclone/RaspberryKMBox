/**
 * Side Channel - UART1 status/control link to bridge
 *
 * Handles incoming query packets from the bridge board and responds
 * with KMBox status data (humanization mode, temperature, USB descriptors).
 *
 * Protocol: [0xCC][CMD][LEN][payload...][CHK]
 * CHK = XOR of CMD, LEN, and all payload bytes.
 *
 * This runs on the KMBox RP2350, responding to queries from the bridge.
 */

#include "side_channel.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

#include "smooth_injection.h"
#include "usb_hid.h"

#include <string.h>
#include <stdio.h>

// ============================================================================
// RX State Machine
// ============================================================================

typedef enum {
    SC_RX_SYNC,
    SC_RX_CMD,
    SC_RX_LEN,
    SC_RX_PAYLOAD,
    SC_RX_CHK
} sc_rx_state_t;

static sc_rx_state_t rx_state = SC_RX_SYNC;
static uint8_t rx_cmd;
static uint8_t rx_len;
static uint8_t rx_payload[SC_MAX_PAYLOAD];
static uint8_t rx_pos;
static uint8_t rx_chk;

// ============================================================================
// Temperature Helper (duplicated from kmbox_serial_handler.c to avoid
// making static functions non-static — it's just 3 lines of ADC read)
// ============================================================================

static inline int16_t sc_read_temperature_decidegrees(void) {
    adc_select_input(4);
    uint16_t raw = adc_read();
    return (int16_t)(270 - ((int32_t)raw * 3300 / 4096 - 706) * 10000 / 1721);
}

// ============================================================================
// TX Helper
// ============================================================================

static void sc_send_packet(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    uint8_t chk = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= payload[i];
    }

    uart_putc_raw(SC_UART, SC_SYNC);
    uart_putc_raw(SC_UART, cmd);
    uart_putc_raw(SC_UART, len);
    if (len > 0) {
        uart_write_blocking(SC_UART, payload, len);
    }
    uart_putc_raw(SC_UART, chk);
}

// ============================================================================
// Command Handlers
// ============================================================================

static void handle_get_info(void) {
    humanization_mode_t hm = smooth_get_humanization_mode();
    inject_mode_t im = smooth_get_inject_mode();
    int16_t max_pf = smooth_get_max_per_frame();
    uint8_t queue_count = 0;
    smooth_get_stats(NULL, NULL, NULL, &queue_count);
    int16_t temp = sc_read_temperature_decidegrees();

    // Pack flags: [0]=jitter_en [1]=vel_match
    uint8_t flags = 0;
    if (hm >= HUMANIZATION_LOW) flags |= 0x01;
    if (smooth_get_velocity_matching()) flags |= 0x02;

    uint8_t resp[8] = {
        (uint8_t)hm,
        (uint8_t)im,
        (uint8_t)(max_pf & 0xFF),
        queue_count,
        (uint8_t)(temp & 0xFF),
        (uint8_t)((temp >> 8) & 0xFF),
        flags,
        (uint8_t)(is_mouse_connected() ? 1 : 0)
    };
    sc_send_packet(SC_CMD_INFO_RESP, resp, 8);
}

static void handle_get_descriptors(void) {
    uint16_t vid = get_attached_vid();
    uint16_t pid = get_attached_pid();
    const char *mfr = get_attached_manufacturer();
    const char *prod = get_attached_product();
    uint8_t mfr_len = (uint8_t)strlen(mfr);
    uint8_t prod_len = (uint8_t)strlen(prod);

    // Cap at 64 chars
    if (mfr_len > 64) mfr_len = 64;
    if (prod_len > 64) prod_len = 64;

    uint8_t resp[6] = {
        (uint8_t)(vid & 0xFF),
        (uint8_t)((vid >> 8) & 0xFF),
        (uint8_t)(pid & 0xFF),
        (uint8_t)((pid >> 8) & 0xFF),
        mfr_len,
        prod_len
    };
    sc_send_packet(SC_CMD_DESC_RESP, resp, 6);
}

static void handle_get_string(void) {
    if (rx_len < 1) return;

    uint8_t string_id = rx_payload[0];
    const char *str = NULL;

    switch (string_id) {
        case SC_STRING_MANUFACTURER:
            str = get_attached_manufacturer();
            break;
        case SC_STRING_PRODUCT:
            str = get_attached_product();
            break;
        default:
            return;
    }

    uint8_t slen = (uint8_t)strlen(str);
    if (slen > 64) slen = 64;

    // Response: [string_id][string_data...]
    uint8_t resp[65];
    resp[0] = string_id;
    memcpy(&resp[1], str, slen);
    sc_send_packet(SC_CMD_STRING_RESP, resp, 1 + slen);
}

static void handle_cycle_humanization(void) {
    humanization_mode_t new_mode = smooth_cycle_humanization_mode();

    // Respond with the new mode
    uint8_t resp[1] = { (uint8_t)new_mode };
    sc_send_packet(SC_CMD_INFO_RESP, resp, 1);
}

// ============================================================================
// Packet Dispatch
// ============================================================================

static void dispatch_packet(void) {
    switch (rx_cmd) {
        case SC_CMD_GET_INFO:
            handle_get_info();
            break;
        case SC_CMD_GET_DESCRIPTORS:
            handle_get_descriptors();
            break;
        case SC_CMD_GET_STRING:
            handle_get_string();
            break;
        case SC_CMD_CYCLE_HUMAN:
            handle_cycle_humanization();
            break;
        default:
            break;
    }
}

// ============================================================================
// Initialization
// ============================================================================

void side_channel_init(void) {
    uart_init(SC_UART, SC_UART_BAUD);
    gpio_set_function(SC_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SC_UART_RX_PIN, GPIO_FUNC_UART);

    // Pull-up on RX for idle-high when disconnected
    gpio_pull_up(SC_UART_RX_PIN);

    rx_state = SC_RX_SYNC;

    printf("[SIDE_CH] Ready (TX=GPIO%d, RX=GPIO%d, %d baud)\n",
           SC_UART_TX_PIN, SC_UART_RX_PIN, SC_UART_BAUD);
}

// ============================================================================
// Main Loop Task
// ============================================================================

void side_channel_task(void) {
    // Drain all available bytes from UART1 RX
    while (uart_is_readable(SC_UART)) {
        uint8_t c = uart_getc(SC_UART);

        switch (rx_state) {
            case SC_RX_SYNC:
                if (c == SC_SYNC) {
                    rx_state = SC_RX_CMD;
                }
                break;

            case SC_RX_CMD:
                rx_cmd = c;
                rx_chk = c;
                rx_state = SC_RX_LEN;
                break;

            case SC_RX_LEN:
                rx_len = c;
                rx_chk ^= c;
                rx_pos = 0;
                if (rx_len > SC_MAX_PAYLOAD) {
                    rx_state = SC_RX_SYNC;  // Invalid, resync
                } else if (rx_len == 0) {
                    rx_state = SC_RX_CHK;   // No payload, go to checksum
                } else {
                    rx_state = SC_RX_PAYLOAD;
                }
                break;

            case SC_RX_PAYLOAD:
                rx_payload[rx_pos++] = c;
                rx_chk ^= c;
                if (rx_pos >= rx_len) {
                    rx_state = SC_RX_CHK;
                }
                break;

            case SC_RX_CHK:
                if (c == rx_chk) {
                    dispatch_packet();
                }
                rx_state = SC_RX_SYNC;
                break;
        }
    }
}
