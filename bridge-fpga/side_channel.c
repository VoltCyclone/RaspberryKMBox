/**
 * Side Channel - UART0 status/control link to KMBox (Bridge side)
 *
 * Periodically sends query packets to the KMBox and parses responses
 * into a cached status structure for TFT display rendering.
 *
 * Polling schedule:
 *   - GET_INFO every 500ms (temperature, humanization, queue depth)
 *   - GET_DESCRIPTORS once, then every 5s (VID/PID, string lengths)
 *   - GET_STRING after descriptor response (manufacturer, product)
 */

#include "side_channel.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <string.h>
#include <stdio.h>

// ============================================================================
// Module State
// ============================================================================

static side_channel_status_t sc_status;
static bool sc_initialized = false;

// Polling timers
static uint32_t last_info_poll_ms = 0;
static uint32_t last_desc_poll_ms = 0;

// String fetch state
static bool need_fetch_manufacturer = false;
static bool need_fetch_product = false;

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
// TX Helper
// ============================================================================

static void sc_send_packet(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    uint8_t chk = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= payload[i];
    }

    uart_putc_raw(SC_BRIDGE_UART, SC_SYNC);
    uart_putc_raw(SC_BRIDGE_UART, cmd);
    uart_putc_raw(SC_BRIDGE_UART, len);
    if (len > 0) {
        uart_write_blocking(SC_BRIDGE_UART, payload, len);
    }
    uart_putc_raw(SC_BRIDGE_UART, chk);
}

// ============================================================================
// Response Handlers
// ============================================================================

static void handle_info_resp(void) {
    if (rx_len == 8) {
        sc_status.humanization_mode = rx_payload[0];
        sc_status.inject_mode       = rx_payload[1];
        sc_status.max_per_frame     = rx_payload[2];
        sc_status.queue_count       = rx_payload[3];
        sc_status.temperature_decideg = (int16_t)(rx_payload[4] | (rx_payload[5] << 8));
        sc_status.flags             = rx_payload[6];
        sc_status.mouse_connected   = (rx_payload[7] != 0);
        sc_status.info_valid        = true;
    } else if (rx_len == 1) {
        // Short response from CYCLE_HUMANIZATION
        sc_status.humanization_mode = rx_payload[0];
    }
}

static void handle_desc_resp(void) {
    if (rx_len < 6) return;

    sc_status.vid = rx_payload[0] | (rx_payload[1] << 8);
    sc_status.pid = rx_payload[2] | (rx_payload[3] << 8);
    sc_status.descriptors_valid = true;

    uint8_t mfr_len = rx_payload[4];
    uint8_t prod_len = rx_payload[5];

    // Schedule string fetches if strings are available
    if (mfr_len > 0) need_fetch_manufacturer = true;
    if (prod_len > 0) need_fetch_product = true;
}

static void handle_string_resp(void) {
    if (rx_len < 1) return;

    uint8_t string_id = rx_payload[0];
    uint8_t slen = rx_len - 1;

    switch (string_id) {
        case SC_STRING_MANUFACTURER:
            if (slen > 64) slen = 64;
            memcpy(sc_status.manufacturer, &rx_payload[1], slen);
            sc_status.manufacturer[slen] = '\0';
            need_fetch_manufacturer = false;
            break;
        case SC_STRING_PRODUCT:
            if (slen > 64) slen = 64;
            memcpy(sc_status.product, &rx_payload[1], slen);
            sc_status.product[slen] = '\0';
            need_fetch_product = false;
            break;
    }

    // Mark strings valid when both are fetched (or not needed)
    if (!need_fetch_manufacturer && !need_fetch_product) {
        sc_status.strings_valid = true;
    }
}

// ============================================================================
// Packet Dispatch
// ============================================================================

static void dispatch_response(void) {
    switch (rx_cmd) {
        case SC_CMD_INFO_RESP:
            handle_info_resp();
            break;
        case SC_CMD_DESC_RESP:
            handle_desc_resp();
            break;
        case SC_CMD_STRING_RESP:
            handle_string_resp();
            break;
        default:
            break;
    }
}

// ============================================================================
// RX Processing
// ============================================================================

static void process_rx(void) {
    while (uart_is_readable(SC_BRIDGE_UART)) {
        uint8_t c = uart_getc(SC_BRIDGE_UART);

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
                    rx_state = SC_RX_SYNC;
                } else if (rx_len == 0) {
                    rx_state = SC_RX_CHK;
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
                    dispatch_response();
                }
                rx_state = SC_RX_SYNC;
                break;
        }
    }
}

// ============================================================================
// Polling Logic
// ============================================================================

static void poll_task(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Info poll (every 500ms)
    if ((now - last_info_poll_ms) >= SC_INFO_POLL_MS) {
        last_info_poll_ms = now;
        sc_send_packet(SC_CMD_GET_INFO, NULL, 0);
    }

    // Descriptor poll (every 5s, or immediately on first run)
    if ((now - last_desc_poll_ms) >= SC_DESC_POLL_MS) {
        last_desc_poll_ms = now;
        sc_send_packet(SC_CMD_GET_DESCRIPTORS, NULL, 0);
    }

    // Fetch strings if needed (one per call to spread out)
    if (need_fetch_manufacturer) {
        uint8_t id = SC_STRING_MANUFACTURER;
        sc_send_packet(SC_CMD_GET_STRING, &id, 1);
    } else if (need_fetch_product) {
        uint8_t id = SC_STRING_PRODUCT;
        sc_send_packet(SC_CMD_GET_STRING, &id, 1);
    }
}

// ============================================================================
// Initialization
// ============================================================================

void side_channel_init(void) {
    memset(&sc_status, 0, sizeof(sc_status));

    uart_init(SC_BRIDGE_UART, SC_BRIDGE_BAUD);
    gpio_set_function(SC_BRIDGE_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SC_BRIDGE_RX_PIN, GPIO_FUNC_UART);

    // Pull-up on RX for idle-high when disconnected
    gpio_pull_up(SC_BRIDGE_RX_PIN);

    rx_state = SC_RX_SYNC;
    sc_initialized = true;

    printf("[SIDE_CH] Ready (TX=GPIO%d, RX=GPIO%d, %d baud)\n",
           SC_BRIDGE_TX_PIN, SC_BRIDGE_RX_PIN, SC_BRIDGE_BAUD);
}

// ============================================================================
// Main Loop Task
// ============================================================================

void side_channel_task(void) {
    if (!sc_initialized) return;

    // Process any incoming responses
    process_rx();

    // Send periodic queries
    poll_task();
}

// ============================================================================
// Status Access
// ============================================================================

const side_channel_status_t* side_channel_get_status(void) {
    return &sc_status;
}
