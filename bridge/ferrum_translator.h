/**
 * Ferrum KM API to KMBox Protocol Translator
 */

#ifndef FERRUM_TRANSLATOR_H
#define FERRUM_TRANSLATOR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define FERRUM_MAX_KMBOX_CMDS 8  // Max KMBox commands per Ferrum command

typedef struct {
    uint8_t buffer[64];  // Buffer for translated KMBox commands (8 bytes * 8 max)
    uint16_t length;     // Length in bytes (multiple of 8)
    bool needs_response; // Whether to send >>> response
    bool valid;          // Translation succeeded
} ferrum_translated_t;

/**
 * Initialize Ferrum translator
 */
void ferrum_translator_init(void);

/**
 * Translate a Ferrum text command line to KMBox binary
 * @param line Text command (e.g., "km.move(10, -5)")
 * @param len Line length
 * @param out Output buffer
 * @return true if translated successfully
 */
bool ferrum_translate_line(const char* line, size_t len, ferrum_translated_t* out);

#endif // FERRUM_TRANSLATOR_H
