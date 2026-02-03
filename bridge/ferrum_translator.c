/**
 * Translates Ferrum text protocol to efficient binary bridge protocol
 */

#include "ferrum_translator.h"
#include "ferrum_protocol.h"
#include "../lib/bridge-protocol/bridge_protocol.h"
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <ctype.h>

void ferrum_translator_init(void) {
    // Nothing to initialize
}

// Parse integer argument
static bool parse_int(const char** p, int32_t* out) {
    // Skip whitespace
    while (**p == ' ' || **p == '\t') (*p)++;
    
    if (**p == '\0' || **p == ')') return false;
    
    // Parse sign
    bool neg = false;
    if (**p == '-') {
        neg = true;
        (*p)++;
    } else if (**p == '+') {
        (*p)++;
    }
    
    // Parse digits
    if (!isdigit(**p)) return false;
    
    int32_t val = 0;
    while (isdigit(**p)) {
        val = val * 10 + (**p - '0');
        (*p)++;
    }
    
    *out = neg ? -val : val;
    return true;
}

// Skip to next argument or end
static void skip_separator(const char** p) {
    while (**p == ' ' || **p == '\t' || **p == ',') (*p)++;
}

bool ferrum_translate_line(const char* line, size_t len, ferrum_translated_t* out) {
    if (!line || len == 0 || !out) return false;
    
    memset(out, 0, sizeof(ferrum_translated_t));
    
    // Check for km. prefix
    if (len < FERRUM_PREFIX_LEN || strncmp(line, FERRUM_PREFIX, FERRUM_PREFIX_LEN) != 0) {
        return false;
    }
    
    const char* p = line + FERRUM_PREFIX_LEN;
    
    // km.move(x, y) - Mouse movement
    if (strncmp(p, FERRUM_CMD_MOVE, strlen(FERRUM_CMD_MOVE)) == 0) {
        p += strlen(FERRUM_CMD_MOVE);
        if (*p != '(') return false;
        p++;
        
        int32_t x, y;
        if (!parse_int(&p, &x)) return false;
        skip_separator(&p);
        if (!parse_int(&p, &y)) return false;
        if (*p != ')') return false;
        
        // Build bridge protocol: [SYNC, CMD, x_lo, x_hi, y_lo, y_hi]
        out->length = bridge_build_mouse_move(out->buffer, (int16_t)x, (int16_t)y);
        out->needs_response = true;
        return true;
    }
    
    // km.wheel(amount) - Wheel movement  
    if (strncmp(p, FERRUM_CMD_WHEEL, strlen(FERRUM_CMD_WHEEL)) == 0) {
        p += strlen(FERRUM_CMD_WHEEL);
        if (*p != '(') return false;
        p++;
        
        int32_t wheel;
        if (!parse_int(&p, &wheel)) return false;
        if (*p != ')') return false;
        
        // Build bridge protocol: [SYNC, CMD, wheel]
        out->length = bridge_build_mouse_wheel(out->buffer, (int8_t)wheel);
        out->needs_response = true;
        return true;
    }
    
    // km.left(state) - Left mouse button (0=release, 1=press)
    if (strncmp(p, FERRUM_CMD_LEFT, strlen(FERRUM_CMD_LEFT)) == 0) {
        p += strlen(FERRUM_CMD_LEFT);
        if (*p != '(') return false;
        p++;
        
        int32_t state;
        if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        
        // Build bridge protocol: [SYNC, CMD, mask, state]
        out->length = bridge_build_button_set(out->buffer, BRIDGE_BTN_LEFT, (state != 0) ? 1 : 0);
        out->needs_response = true;
        return true;
    }
    
    // km.right(state) - Right mouse button
    if (strncmp(p, FERRUM_CMD_RIGHT, strlen(FERRUM_CMD_RIGHT)) == 0) {
        p += strlen(FERRUM_CMD_RIGHT);
        if (*p != '(') return false;
        p++;
        
        int32_t state;
        if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        
        out->length = bridge_build_button_set(out->buffer, BRIDGE_BTN_RIGHT, (state != 0) ? 1 : 0);
        out->needs_response = true;
        return true;
    }
    
    // km.middle(state) - Middle mouse button
    if (strncmp(p, FERRUM_CMD_MIDDLE, strlen(FERRUM_CMD_MIDDLE)) == 0) {
        p += strlen(FERRUM_CMD_MIDDLE);
        if (*p != '(') return false;
        p++;
        
        int32_t state;
        if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        
        out->length = bridge_build_button_set(out->buffer, BRIDGE_BTN_MIDDLE, (state != 0) ? 1 : 0);
        out->needs_response = true;
        return true;
    }
    
    // km.side1(state) - Side button 1 (Back)
    if (strncmp(p, FERRUM_CMD_SIDE1, strlen(FERRUM_CMD_SIDE1)) == 0) {
        p += strlen(FERRUM_CMD_SIDE1);
        if (*p != '(') return false;
        p++;
        
        int32_t state;
        if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        
        out->length = bridge_build_button_set(out->buffer, BRIDGE_BTN_SIDE1, (state != 0) ? 1 : 0);
        out->needs_response = true;
        return true;
    }
    
    // km.side2(state) - Side button 2 (Forward)
    if (strncmp(p, FERRUM_CMD_SIDE2, strlen(FERRUM_CMD_SIDE2)) == 0) {
        p += strlen(FERRUM_CMD_SIDE2);
        if (*p != '(') return false;
        p++;
        
        int32_t state;
        if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        
        out->length = bridge_build_button_set(out->buffer, BRIDGE_BTN_SIDE2, (state != 0) ? 1 : 0);
        out->needs_response = true;
        return true;
    }
    
    // Keyboard commands not supported (KMBox is mouse-only)
    if (strncmp(p, FERRUM_CMD_DOWN, strlen(FERRUM_CMD_DOWN)) == 0 ||
        strncmp(p, FERRUM_CMD_UP, strlen(FERRUM_CMD_UP)) == 0 ||
        strncmp(p, FERRUM_CMD_PRESS, strlen(FERRUM_CMD_PRESS)) == 0 ||
        strncmp(p, FERRUM_CMD_MULTIDOWN, strlen(FERRUM_CMD_MULTIDOWN)) == 0 ||
        strncmp(p, FERRUM_CMD_MULTIUP, strlen(FERRUM_CMD_MULTIUP)) == 0 ||
        strncmp(p, FERRUM_CMD_MULTIPRESS, strlen(FERRUM_CMD_MULTIPRESS)) == 0) {
        return false;  // Keyboard not supported
    }
    
    return false;  // Unknown command
}
