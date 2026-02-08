/**
 * Translates Ferrum text protocol to KMBox text protocol
 */

#include "ferrum_translator.h"
#include "ferrum_protocol.h"
#include "protocol_luts.h"
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <ctype.h>
#include <stdio.h>

void ferrum_translator_init(void) {
    // Nothing to initialize
}

// Helper: Convert button name to HID mask
static uint8_t ferrum_button_to_mask(const char* name) {
    if (strcmp(name, "left") == 0) return 0x01;
    if (strcmp(name, "right") == 0) return 0x02;
    if (strcmp(name, "middle") == 0) return 0x04;
    if (strcmp(name, "side1") == 0) return 0x08;
    if (strcmp(name, "side2") == 0) return 0x10;
    return 0x00;
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
        
        // Build KMBox text protocol: "M<x>,<y>\n"
        out->length = fast_build_move((char*)out->buffer, (int16_t)x, (int16_t)y);
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
        
        // Build KMBox text protocol: "W<wheel>\n"
        out->length = fast_build_wheel((char*)out->buffer, (int8_t)wheel);
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
        
        // Build KMBox text protocol: "B<mask>\n"
        uint8_t mask = ferrum_button_to_mask("left");
        out->length = fast_build_button((char*)out->buffer, (state != 0) ? mask : 0);
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
        
        uint8_t mask = ferrum_button_to_mask("right");
        out->length = fast_build_button((char*)out->buffer, (state != 0) ? mask : 0);
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
        
        uint8_t mask = ferrum_button_to_mask("middle");
        out->length = fast_build_button((char*)out->buffer, (state != 0) ? mask : 0);
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
        
        uint8_t mask = ferrum_button_to_mask("side1");
        out->length = fast_build_button((char*)out->buffer, (state != 0) ? mask : 0);
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
        
        uint8_t mask = ferrum_button_to_mask("side2");
        out->length = fast_build_button((char*)out->buffer, (state != 0) ? mask : 0);
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
