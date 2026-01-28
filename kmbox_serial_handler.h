/*
 * KMBox Serial Command Handler Header
 * Handles CP2110 HID-to-UART bridge communication with handshake protocol
 */

#ifndef KMBOX_SERIAL_HANDLER_H
#define KMBOX_SERIAL_HANDLER_H

#include <stdbool.h>
#include <stdint.h>
#include "defines.h"
#include "led_control.h"

//--------------------------------------------------------------------+
// CP2110 Handshake Protocol
//--------------------------------------------------------------------+
// 
// The handshake protocol ensures reliable connection between the PC client
// and the firmware. It uses simple text-based commands for compatibility.
//
// Protocol Flow:
//   1. Firmware boots and enters WAITING state (LED: breathing light blue)
//   2. PC sends: "KMBOX_PING\n"
//   3. Firmware responds: "KMBOX_PONG:v1.0\n" and enters CONNECTING state
//   4. PC sends: "KMBOX_CONNECT\n"
//   5. Firmware responds: "KMBOX_READY\n" and enters CONNECTED state (LED: green)
//   6. Normal command processing begins
//   7. PC can send periodic "KMBOX_PING\n" as heartbeat
//   8. If no data received for TIMEOUT, enter DISCONNECTED state (LED: orange-red breathing)
//
// Special Commands:
//   KMBOX_PING     - Heartbeat/discovery, firmware responds with KMBOX_PONG:version
//   KMBOX_CONNECT  - Establish connection, firmware responds with KMBOX_READY
//   KMBOX_DISCONNECT - Graceful disconnect, firmware returns to WAITING state
//   KMBOX_STATUS   - Query status, firmware responds with current state
//

// Handshake timeout (milliseconds) - disconnect if no data received
#define CP2110_HEARTBEAT_TIMEOUT_MS     5000
// Minimum interval between heartbeat checks
#define CP2110_HEARTBEAT_CHECK_MS       100

// Protocol version
#define KMBOX_PROTOCOL_VERSION          "1.0"

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

// Initialize the serial handler
void kmbox_serial_init(void);

// Process any available serial input (call this in main loop)
void kmbox_serial_task(void);

// Send mouse report with kmbox button states
bool kmbox_send_mouse_report(void);

// Get current CP2110 connection state
cp2110_connection_state_t kmbox_get_connection_state(void);

// Check if connected and ready for commands
bool kmbox_is_connected(void);

// Send a response string back to the PC (adds newline)
void kmbox_send_response(const char *response);

// Send status message to bridge (if connected via UART TX)
void kmbox_send_status(const char* message);

// Send ping to bridge (for bidirectional testing)
void kmbox_send_ping_to_bridge(void);

#endif // KMBOX_SERIAL_HANDLER_H