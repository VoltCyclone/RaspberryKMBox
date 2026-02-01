/**
 * Bridge Protocol Handler Implementation
 */

#include "bridge_handler.h"
#include "../kmbox-commands/kmbox_commands.h"
#include <string.h>

void bridge_parser_init(bridge_parser_t* parser) {
    parser->state = BRIDGE_STATE_IDLE;
    parser->bytes_needed = 0;
    parser->bytes_received = 0;
    memset(&parser->packet, 0, sizeof(bridge_packet_t));
}

bool bridge_parser_process_byte(bridge_parser_t* parser, uint8_t byte) {
    switch (parser->state) {
        case BRIDGE_STATE_IDLE:
            if (byte == BRIDGE_SYNC_BYTE) {
                parser->packet.generic.sync = byte;
                parser->state = BRIDGE_STATE_CMD;
            }
            break;
            
        case BRIDGE_STATE_CMD:
            parser->packet.generic.cmd = byte;
            
            // Determine payload size based on command
            switch (byte) {
                case BRIDGE_CMD_MOUSE_MOVE:
                    parser->bytes_needed = 4;  // x:i16, y:i16
                    break;
                case BRIDGE_CMD_MOUSE_WHEEL:
                    parser->bytes_needed = 1;  // wheel:i8
                    break;
                case BRIDGE_CMD_BUTTON_SET:
                    parser->bytes_needed = 2;  // mask:u8, state:u8
                    break;
                case BRIDGE_CMD_MOUSE_MOVE_WHEEL:
                    parser->bytes_needed = 5;  // x:i16, y:i16, wheel:i8
                    break;
                case BRIDGE_CMD_PING:
                case BRIDGE_CMD_RESET:
                    parser->bytes_needed = 0;  // No payload
                    break;
                default:
                    // Unknown command, back to idle
                    parser->state = BRIDGE_STATE_IDLE;
                    return false;
            }
            
            if (parser->bytes_needed == 0) {
                // Command complete
                parser->state = BRIDGE_STATE_IDLE;
                return true;
            }
            
            parser->bytes_received = 0;
            parser->state = BRIDGE_STATE_PAYLOAD;
            break;
            
        case BRIDGE_STATE_PAYLOAD:
            parser->packet.generic.payload[parser->bytes_received++] = byte;
            
            if (parser->bytes_received >= parser->bytes_needed) {
                // Packet complete
                parser->state = BRIDGE_STATE_IDLE;
                return true;
            }
            break;
    }
    
    return false;
}

void bridge_execute_packet(const bridge_packet_t* packet, uint32_t current_time_ms) {
    switch (packet->generic.cmd) {
        case BRIDGE_CMD_MOUSE_MOVE: {
            int16_t x = *((int16_t*)(packet->generic.payload + 0));
            int16_t y = *((int16_t*)(packet->generic.payload + 2));
            kmbox_add_mouse_movement(x, y);
            break;
        }
        
        case BRIDGE_CMD_MOUSE_WHEEL: {
            int8_t wheel = packet->generic.payload[0];
            kmbox_add_wheel_movement(wheel);
            break;
        }
        
        case BRIDGE_CMD_BUTTON_SET: {
            uint8_t mask = packet->generic.payload[0];
            uint8_t state = packet->generic.payload[1];
            
            // Map button mask to button index and force state
            if (mask & BRIDGE_BTN_LEFT) {
                // Force left button state - need to add this API
                // For now, use physical button update as workaround
            }
            if (mask & BRIDGE_BTN_RIGHT) {
                // Force right button state
            }
            if (mask & BRIDGE_BTN_MIDDLE) {
                // Force middle button state
            }
            if (mask & BRIDGE_BTN_SIDE1) {
                // Force side1 button state
            }
            if (mask & BRIDGE_BTN_SIDE2) {
                // Force side2 button state
            }
            // TODO: Need to add button force API to kmbox_commands
            break;
        }
        
        case BRIDGE_CMD_MOUSE_MOVE_WHEEL: {
            int16_t x = *((int16_t*)(packet->generic.payload + 0));
            int16_t y = *((int16_t*)(packet->generic.payload + 2));
            int8_t wheel = packet->generic.payload[4];
            kmbox_add_mouse_movement(x, y);
            kmbox_add_wheel_movement(wheel);
            break;
        }
        
        case BRIDGE_CMD_PING:
            // Send pong response (handled by caller)
            break;
            
        case BRIDGE_CMD_RESET:
            // Reset all states
            kmbox_add_mouse_movement(0, 0);  // Clear accumulators
            kmbox_add_wheel_movement(0);
            break;
    }
}
