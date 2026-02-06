/**
 * Makcu to KMBox Protocol Translator Implementation
 * Translates Makcu binary protocol to KMBox text protocol
 */

#include "makcu_translator.h"
#include "makcu_protocol.h"
#include "protocol_luts.h"
#include <string.h>
#include <stdio.h>

void makcu_translator_init(void) {
    // Nothing to initialize yet
}

const char* makcu_translate_result_str(translate_result_t result) {
    switch (result) {
        case TRANSLATE_OK: return "OK";
        case TRANSLATE_UNSUPPORTED: return "UNSUPPORTED";
        case TRANSLATE_INVALID: return "INVALID";
        case TRANSLATE_PASSTHROUGH: return "PASSTHROUGH";
        default: return "UNKNOWN";
    }
}

bool makcu_cmd_needs_response(uint8_t cmd) {
    // GET commands that need responses
    switch (cmd) {
        case MAKCU_CMD_GETPOS:
        case MAKCU_CMD_KB_ISDOWN:
        case MAKCU_CMD_DEVICE:
        case MAKCU_CMD_INFO:
        case MAKCU_CMD_VERSION:
        case MAKCU_CMD_SCREEN:
            return true;
        default:
            return false;
    }
}

uint16_t makcu_build_response(
    uint8_t cmd,
    uint8_t status,
    const uint8_t* payload,
    uint16_t payload_len,
    uint8_t* out_buffer
) {
    makcu_frame_header_t* header = (makcu_frame_header_t*)out_buffer;
    header->start = MAKCU_FRAME_START;
    header->cmd = cmd;
    
    if (payload && payload_len > 0) {
        header->len = payload_len;
        memcpy(out_buffer + sizeof(makcu_frame_header_t), payload, payload_len);
        return sizeof(makcu_frame_header_t) + payload_len;
    } else {
        header->len = 1;
        out_buffer[sizeof(makcu_frame_header_t)] = status;
        return sizeof(makcu_frame_header_t) + 1;
    }
}

// Helper: Convert Makcu button number to HID button mask
static uint8_t makcu_button_to_mask(uint8_t button) {
    switch (button) {
        case 1: return 0x01;  // Left
        case 2: return 0x02;  // Right
        case 3: return 0x04;  // Middle
        case 4: return 0x08;  // Side1
        case 5: return 0x10;  // Side2
        default: return 0x00;
    }
}

translate_result_t makcu_translate_command(
    uint8_t cmd,
    const uint8_t* payload,
    uint16_t payload_len,
    translated_cmd_t* out_cmd
) {
    out_cmd->length = 0;
    out_cmd->result = TRANSLATE_UNSUPPORTED;
    
    if (!payload && payload_len > 0) {
        out_cmd->result = TRANSLATE_INVALID;
        return TRANSLATE_INVALID;
    }
    
    switch (cmd) {
        // ===== MOUSE COMMANDS =====
        
        case MAKCU_CMD_MOVE: {
            // Move (0x0D): [dx:i16, dy:i16, segments:u8, cx1:i8, cy1:i8]
            if (payload_len < 4) {
                out_cmd->result = TRANSLATE_INVALID;
                return TRANSLATE_INVALID;
            }
            
            makcu_move_t* move = (makcu_move_t*)payload;
            
            // Translate to KMBox text protocol: "M<x>,<y>\n"
            out_cmd->length = fast_build_move((char*)out_cmd->buffer, move->dx, move->dy);
            out_cmd->result = TRANSLATE_OK;
            
            // Note: Bezier segments/control points ignored
            // KMBox does smoothing internally
            return TRANSLATE_OK;
        }
        
        case MAKCU_CMD_MO: {
            // Raw mouse frame (0x0B): [buttons:u8, x:i16, y:i16, wheel:i8, pan:i8, tilt:i8]
            if (payload_len < 8) {
                out_cmd->result = TRANSLATE_INVALID;
                return TRANSLATE_INVALID;
            }
            
            makcu_mo_t* mo = (makcu_mo_t*)payload;
            
            // Translate movement and wheel
            if (mo->x != 0 || mo->y != 0) {
                out_cmd->length = fast_build_move((char*)out_cmd->buffer, mo->x, mo->y);
                if (mo->wheel != 0) {
                    // Append wheel command
                    out_cmd->length += fast_build_wheel((char*)out_cmd->buffer + out_cmd->length, mo->wheel);
                }
            } else if (mo->wheel != 0) {
                out_cmd->length = fast_build_wheel((char*)out_cmd->buffer, mo->wheel);
            }
            
            // Note: Button state and pan/tilt not supported in simple text protocol
            out_cmd->result = TRANSLATE_OK;
            return TRANSLATE_OK;
        }
        
        case MAKCU_CMD_LEFT_BUTTON:
        case MAKCU_CMD_RIGHT_BUTTON:
        case MAKCU_CMD_MIDDLE_BUTTON:
        case MAKCU_CMD_SIDE1_BUTTON:
        case MAKCU_CMD_SIDE2_BUTTON: {
            // Button control: [state:u8] (0=release, 1=down, 2=silent_release)
            if (payload_len < 1) {
                // GET query - not supported yet
                out_cmd->result = TRANSLATE_UNSUPPORTED;
                return TRANSLATE_UNSUPPORTED;
            }
            
            uint8_t state = payload[0];
            uint8_t button_num = 0;
            
            switch (cmd) {
                case MAKCU_CMD_LEFT_BUTTON: button_num = 1; break;
                case MAKCU_CMD_RIGHT_BUTTON: button_num = 2; break;
                case MAKCU_CMD_MIDDLE_BUTTON: button_num = 3; break;
                case MAKCU_CMD_SIDE1_BUTTON: button_num = 4; break;
                case MAKCU_CMD_SIDE2_BUTTON: button_num = 5; break;
            }
            
            uint8_t button_mask = makcu_button_to_mask(button_num);
            if (!button_mask) {
                out_cmd->result = TRANSLATE_INVALID;
                return TRANSLATE_INVALID;
            }
            
            // Translate to KMBox text protocol: "B<mask>\n"
            // If pressing, set the mask; if releasing, clear it (send 0)
            out_cmd->length = fast_build_button((char*)out_cmd->buffer, (state == 1) ? button_mask : 0);
            out_cmd->result = TRANSLATE_OK;
            return TRANSLATE_OK;
        }
        
        case MAKCU_CMD_CLICK: {
            // Click scheduling (0x04): [button:u8, count:u8, delay_ms:u8]
            if (payload_len < 3) {
                out_cmd->result = TRANSLATE_INVALID;
                return TRANSLATE_INVALID;
            }
            
            makcu_click_t* click = (makcu_click_t*)payload;
            uint8_t button_mask = makcu_button_to_mask(click->button);
            if (!button_mask) {
                out_cmd->result = TRANSLATE_INVALID;
                return TRANSLATE_INVALID;
            }
            
            // Translate to KMBox click command: press then release
            // Since KMBox doesn't have direct click with count, we'll do a single click
            out_cmd->length = fast_build_button((char*)out_cmd->buffer, button_mask);
            out_cmd->length += fast_build_button((char*)out_cmd->buffer + out_cmd->length, 0);
            out_cmd->result = TRANSLATE_OK;
            return TRANSLATE_OK;
        }
        
        case MAKCU_CMD_WHEEL: {
            // Wheel (0x18): [delta:i8]
            if (payload_len < 1) {
                out_cmd->result = TRANSLATE_INVALID;
                return TRANSLATE_INVALID;
            }
            
            int8_t delta = (int8_t)payload[0];
            
            // Translate to KMBox text protocol: "W<delta>\n"
            out_cmd->length = fast_build_wheel((char*)out_cmd->buffer, delta);
            out_cmd->result = TRANSLATE_OK;
            return TRANSLATE_OK;
        }
        
        // ===== KEYBOARD COMMANDS =====
        // KMBox is mouse-only, keyboard commands not supported
        
        case MAKCU_CMD_KB_DOWN:
        case MAKCU_CMD_KB_UP:
        case MAKCU_CMD_KB_PRESS:
        case MAKCU_CMD_KB_STRING:
            out_cmd->result = TRANSLATE_UNSUPPORTED;
            return TRANSLATE_UNSUPPORTED;
        
        // ===== UNSUPPORTED COMMANDS =====
        
        case MAKCU_CMD_MOVETO:
        case MAKCU_CMD_SILENT:
        case MAKCU_CMD_TURBO:
        case MAKCU_CMD_LOCK:
        case MAKCU_CMD_CATCH:
        case MAKCU_CMD_INVERT_X:
        case MAKCU_CMD_INVERT_Y:
        case MAKCU_CMD_SWAP_XY:
        case MAKCU_CMD_REMAP_BUTTON:
        case MAKCU_CMD_REMAP_AXIS:
        case MAKCU_CMD_PAN:
        case MAKCU_CMD_TILT:
        case MAKCU_CMD_GETPOS:
        case MAKCU_CMD_KB_INIT:
        case MAKCU_CMD_KB_ISDOWN:
        case MAKCU_CMD_KB_DISABLE:
        case MAKCU_CMD_KB_MASK:
        case MAKCU_CMD_KB_REMAP:
        case MAKCU_CMD_AXIS_STREAM:
        case MAKCU_CMD_BUTTONS_STREAM:
        case MAKCU_CMD_MOUSE_STREAM:
        case MAKCU_CMD_KB_STREAM:
            out_cmd->result = TRANSLATE_UNSUPPORTED;
            return TRANSLATE_UNSUPPORTED;
        
        // ===== MISC COMMANDS =====
        
        case MAKCU_CMD_VERSION:
            // Return bridge version
            out_cmd->length = snprintf((char*)out_cmd->buffer, sizeof(out_cmd->buffer),
                                      "kmbox_bridge v1.0\n");
            out_cmd->result = TRANSLATE_OK;
            return TRANSLATE_OK;
        
        case MAKCU_CMD_SCREEN:
        case MAKCU_CMD_BAUD:
        case MAKCU_CMD_ECHO:
        case MAKCU_CMD_LED:
        case MAKCU_CMD_INFO:
        case MAKCU_CMD_REBOOT:
        case MAKCU_CMD_DEVICE:
        case MAKCU_CMD_BYPASS:
        case MAKCU_CMD_HS:
        case MAKCU_CMD_LOG:
        case MAKCU_CMD_RELEASE:
        case MAKCU_CMD_SERIAL:
        case MAKCU_CMD_FAULT:
            // These are bridge/device management commands
            // Don't forward to KMBox
            out_cmd->result = TRANSLATE_UNSUPPORTED;
            return TRANSLATE_UNSUPPORTED;
        
        default:
            out_cmd->result = TRANSLATE_UNSUPPORTED;
            return TRANSLATE_UNSUPPORTED;
    }
}
