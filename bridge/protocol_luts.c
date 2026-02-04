/**
 * Protocol Translation Lookup Tables Implementation
 * 
 * Pre-computed strings stored in flash to eliminate snprintf() overhead.
 * These tables provide O(1) translation for protocol commands.
 */

#include "protocol_luts.h"
#include <stdio.h>

// Button press commands (indexed by button number 0-5)
const char* const button_press_lut[6] = {
    "B0\n",   // 0: no button
    "B1\n",   // 1: left
    "B2\n",   // 2: right
    "B4\n",   // 3: middle
    "B8\n",   // 4: side1
    "B16\n"   // 5: side2
};

const char* const button_release_lut = "B0\n";

// Integer to ASCII lookup tables
// These are generated at runtime to save flash space

char int8_ascii_lut[256][5];
char int16_ascii_lut[INT16_LUT_SIZE][7];

void protocol_luts_init(void) {
    // Generate int8 LUT (-128 to 127)
    for (int i = 0; i < 256; i++) {
        int8_t val = (int8_t)i;
        snprintf(int8_ascii_lut[i], sizeof(int8_ascii_lut[i]), "%d", val);
    }
    
    // Generate int16 LUT (common mouse movement range)
    for (int i = 0; i < INT16_LUT_SIZE; i++) {
        int16_t val = (int16_t)(i + INT16_LUT_MIN);
        snprintf(int16_ascii_lut[i], sizeof(int16_ascii_lut[i]), "%d", val);
    }
}
