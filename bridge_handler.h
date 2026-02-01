/**
 * Bridge Protocol Handler for KMBox
 * 
 * Receives optimized bridge protocol packets and processes them
 */

#ifndef BRIDGE_HANDLER_H
#define BRIDGE_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "bridge_protocol.h"

// Parser state
typedef enum {
    BRIDGE_STATE_IDLE,
    BRIDGE_STATE_CMD,
    BRIDGE_STATE_PAYLOAD
} bridge_parser_state_t;

typedef struct {
    bridge_parser_state_t state;
    bridge_packet_t packet;
    uint8_t bytes_needed;
    uint8_t bytes_received;
} bridge_parser_t;

// Initialize bridge protocol parser
void bridge_parser_init(bridge_parser_t* parser);

// Process incoming byte, returns true if packet complete
bool bridge_parser_process_byte(bridge_parser_t* parser, uint8_t byte);

// Execute completed packet (call when process_byte returns true)
void bridge_execute_packet(const bridge_packet_t* packet, uint32_t current_time_ms);

#endif // BRIDGE_HANDLER_H
