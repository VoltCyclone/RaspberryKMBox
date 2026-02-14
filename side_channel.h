/**
 * Side Channel - UART1 status/control link to bridge
 *
 * Provides a low-bandwidth UART connection for the bridge board to
 * query KMBox status (humanization, temperature, USB descriptors)
 * without using the FPGA's fast SPI command path.
 *
 * Physical connection (2 wires + GND):
 *   KMBox UART1 TX (GPIO 8) → Bridge UART0 RX (GPIO 1)
 *   KMBox UART1 RX (GPIO 9) ← Bridge UART0 TX (GPIO 0)
 *
 * Protocol: [0xCC][CMD][LEN][payload...][CHK]
 *   0xCC = sync byte, LEN = payload length, CHK = XOR of CMD+LEN+payload
 */

#ifndef SIDE_CHANNEL_H
#define SIDE_CHANNEL_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// UART Configuration
// ============================================================================

#ifndef SC_UART_TX_PIN
#define SC_UART_TX_PIN      8       // GPIO 8: UART1 TX to bridge RX
#endif

#ifndef SC_UART_RX_PIN
#define SC_UART_RX_PIN      9       // GPIO 9: UART1 RX from bridge TX
#endif

#define SC_UART             uart1
#define SC_UART_BAUD        921600

// ============================================================================
// Protocol Constants
// ============================================================================

#define SC_SYNC             0xCC    // Sync byte (distinct from KMBox 0xAA)
#define SC_MAX_PAYLOAD      128     // Maximum payload length

// Request commands (Bridge → KMBox)
#define SC_CMD_GET_INFO         0x01    // Query basic status
#define SC_CMD_GET_DESCRIPTORS  0x02    // Query VID/PID + string lengths
#define SC_CMD_GET_STRING       0x03    // Fetch descriptor string by ID
#define SC_CMD_CYCLE_HUMAN      0x05    // Cycle humanization mode

// Response commands (KMBox → Bridge)
#define SC_CMD_INFO_RESP        0x81    // Basic status response
#define SC_CMD_DESC_RESP        0x82    // Descriptor info response
#define SC_CMD_STRING_RESP      0x83    // String data response

// String IDs for SC_CMD_GET_STRING
#define SC_STRING_MANUFACTURER  0
#define SC_STRING_PRODUCT       1

// ============================================================================
// Public API
// ============================================================================

/**
 * Initialize side channel UART1.
 * Call during setup after clocks are configured.
 */
void side_channel_init(void);

/**
 * Process incoming side channel queries.
 * Call from main loop. Non-blocking.
 */
void side_channel_task(void);

#endif // SIDE_CHANNEL_H
