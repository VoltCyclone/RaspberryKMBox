/**
 * Side Channel - UART0 status/control link to KMBox
 *
 * Periodically queries the KMBox board for status data (humanization mode,
 * temperature, USB descriptor strings) over a direct UART connection that
 * bypasses the FPGA. Results are cached for TFT display rendering.
 *
 * Physical connection (2 wires + GND):
 *   Bridge UART0 TX (GPIO 20) → KMBox UART1 RX (GPIO 9)
 *   Bridge UART0 RX (GPIO 25) ← KMBox UART1 TX (GPIO 8)
 *
 * Protocol: [0xCC][CMD][LEN][payload...][CHK]
 */

#ifndef SIDE_CHANNEL_H
#define SIDE_CHANNEL_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// UART Configuration
// ============================================================================

#define SC_BRIDGE_UART          uart0
#define SC_BRIDGE_TX_PIN        20      // GPIO 20: UART0 TX to KMBox RX
#define SC_BRIDGE_RX_PIN        25      // GPIO 25: UART0 RX from KMBox TX
#define SC_BRIDGE_BAUD          921600

// ============================================================================
// Protocol Constants (must match KMBox side_channel.h)
// ============================================================================

#define SC_SYNC                 0xCC
#define SC_MAX_PAYLOAD          128

// Request commands (Bridge → KMBox)
#define SC_CMD_GET_INFO         0x01
#define SC_CMD_GET_DESCRIPTORS  0x02
#define SC_CMD_GET_STRING       0x03
#define SC_CMD_CYCLE_HUMAN      0x05

// Response commands (KMBox → Bridge)
#define SC_CMD_INFO_RESP        0x81
#define SC_CMD_DESC_RESP        0x82
#define SC_CMD_STRING_RESP      0x83

// String IDs
#define SC_STRING_MANUFACTURER  0
#define SC_STRING_PRODUCT       1

// ============================================================================
// Polling Intervals
// ============================================================================

#define SC_INFO_POLL_MS         500     // Query basic status every 500ms
#define SC_DESC_POLL_MS         5000    // Re-check descriptors every 5s (in case device changes)

// ============================================================================
// Cached Status
// ============================================================================

typedef struct {
    // Basic info (from INFO_RESP)
    uint8_t  humanization_mode;
    uint8_t  inject_mode;
    uint8_t  max_per_frame;
    uint8_t  queue_count;
    int16_t  temperature_decideg;
    uint8_t  flags;
    bool     mouse_connected;
    bool     info_valid;

    // Descriptor info (from DESC_RESP)
    uint16_t vid;
    uint16_t pid;
    bool     descriptors_valid;

    // Strings (from STRING_RESP)
    char     manufacturer[65];
    char     product[65];
    bool     strings_valid;
} side_channel_status_t;

// ============================================================================
// Public API
// ============================================================================

/**
 * Initialize side channel UART0 for bridge-to-KMBox communication.
 */
void side_channel_init(void);

/**
 * Process side channel communication.
 * Sends periodic queries and processes responses.
 * Call from main loop. Non-blocking.
 */
void side_channel_task(void);

/**
 * Get pointer to cached KMBox status.
 * Valid after first successful INFO_RESP.
 */
const side_channel_status_t* side_channel_get_status(void);

#endif // SIDE_CHANNEL_H
