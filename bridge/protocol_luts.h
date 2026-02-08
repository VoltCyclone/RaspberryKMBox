/**
 * Protocol Translation Lookup Tables
 * 
 * Pre-computed strings to eliminate expensive snprintf() calls during
 * protocol translation. These LUTs are stored in flash and provide O(1)
 * translation for common commands.
 */

#ifndef PROTOCOL_LUTS_H
#define PROTOCOL_LUTS_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

// Button command LUTs (press/release)
extern const char* const button_press_lut[6];   // B1\n, B2\n, B4\n, B8\n, B16\n
extern const char* const button_release_lut;    // B0\n

// Integer to ASCII LUT for -128 to 127 (int8_t range)
// Format: "-128" to "127" (null-terminated, max 5 bytes including null)
extern char int8_ascii_lut[256][5];

// Integer to ASCII LUT for -32768 to 32767 (int16_t range)
// Only populate commonly used range for mouse movement
#define INT16_LUT_MIN -2048
#define INT16_LUT_MAX  2047
#define INT16_LUT_SIZE (INT16_LUT_MAX - INT16_LUT_MIN + 1)
extern char int16_ascii_lut[INT16_LUT_SIZE][7];  // "-32768" max

/**
 * Fast integer to ASCII conversion using LUTs
 * Returns pointer to null-terminated string
 */
static inline const char* fast_i8toa(int8_t val) {
    return int8_ascii_lut[(uint8_t)val];
}

static inline const char* fast_i16toa(int16_t val) {
    if (val >= INT16_LUT_MIN && val <= INT16_LUT_MAX) {
        return int16_ascii_lut[val - INT16_LUT_MIN];
    }
    // Fallback for out-of-range values (rare)
    static char fallback[7];
    snprintf(fallback, sizeof(fallback), "%d", val);
    return fallback;
}

/**
 * Fast command building using LUTs
 * Returns length of built command (excluding null terminator)
 */

// Build mouse move command: M<x>,<y>\n
static inline size_t fast_build_move(char* buf, int16_t x, int16_t y) {
    char* p = buf;
    *p++ = 'M';
    
    // Copy X value
    const char* x_str = fast_i16toa(x);
    while (*x_str) *p++ = *x_str++;
    
    *p++ = ',';
    
    // Copy Y value
    const char* y_str = fast_i16toa(y);
    while (*y_str) *p++ = *y_str++;
    
    *p++ = '\n';
    *p = '\0';
    
    return p - buf;
}

// Build wheel command: W<delta>\n
static inline size_t fast_build_wheel(char* buf, int8_t delta) {
    char* p = buf;
    *p++ = 'W';
    
    const char* delta_str = fast_i8toa(delta);
    while (*delta_str) *p++ = *delta_str++;
    
    *p++ = '\n';
    *p = '\0';
    
    return p - buf;
}

// Build button command: B<mask>\n
static inline size_t fast_build_button(char* buf, uint8_t mask) {
    char* p = buf;
    *p++ = 'B';
    
    // mask is 0-31, small enough to format directly
    if (mask == 0) {
        *p++ = '0';
    } else if (mask < 10) {
        *p++ = '0' + mask;
    } else {
        *p++ = '0' + (mask / 10);
        *p++ = '0' + (mask % 10);
    }
    
    *p++ = '\n';
    *p = '\0';
    
    return p - buf;
}

/**
 * Initialize lookup tables (call once at startup)
 */
void protocol_luts_init(void);

#endif // PROTOCOL_LUTS_H
