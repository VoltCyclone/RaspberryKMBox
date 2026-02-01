/**
 * Makcu to KMBox Protocol Translator
 * 
 * Translates Makcu binary protocol commands to KMBox optimized protocol.
 * This allows the bridge to accept Makcu API commands from PC and convert
 * them to the native KMBox format for optimal performance.
 */

#ifndef MAKCU_TRANSLATOR_H
#define MAKCU_TRANSLATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "makcu_protocol.h"

// Translation result
typedef enum {
    TRANSLATE_OK,               // Successfully translated
    TRANSLATE_UNSUPPORTED,      // Command not supported
    TRANSLATE_INVALID,          // Invalid command format
    TRANSLATE_PASSTHROUGH       // Forward directly to KMBox
} translate_result_t;

// Translated command buffer (max KMBox command size)
#define KMBOX_MAX_CMD_SIZE 256

typedef struct {
    uint8_t buffer[KMBOX_MAX_CMD_SIZE];
    uint16_t length;
    translate_result_t result;
} translated_cmd_t;

/**
 * Initialize translator module
 */
void makcu_translator_init(void);

/**
 * Translate a Makcu binary command to KMBox protocol
 * 
 * @param cmd Makcu command byte
 * @param payload Command payload data
 * @param payload_len Payload length in bytes
 * @param out_cmd Output buffer for translated command
 * @return Translation result
 */
translate_result_t makcu_translate_command(
    uint8_t cmd,
    const uint8_t* payload,
    uint16_t payload_len,
    translated_cmd_t* out_cmd
);

/**
 * Get a human-readable string for translation result
 */
const char* makcu_translate_result_str(translate_result_t result);

/**
 * Check if a command requires a response back to host
 */
bool makcu_cmd_needs_response(uint8_t cmd);

/**
 * Build a Makcu response frame
 * 
 * @param cmd Original command byte
 * @param status Status byte (MAKCU_STATUS_OK or MAKCU_STATUS_ERR)
 * @param payload Optional response payload
 * @param payload_len Response payload length
 * @param out_buffer Output buffer for response frame
 * @return Total frame length
 */
uint16_t makcu_build_response(
    uint8_t cmd,
    uint8_t status,
    const uint8_t* payload,
    uint16_t payload_len,
    uint8_t* out_buffer
);

#endif // MAKCU_TRANSLATOR_H
