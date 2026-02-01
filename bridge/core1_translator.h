/**
 * Core1 Protocol Translator - Hardware-Accelerated Translation Pipeline
 * 
 * Offloads Makcu→KMBox protocol translation to dedicated Core1,
 * freeing Core0 for USB/CDC/tracking and eliminating translation overhead.
 */

#ifndef CORE1_TRANSLATOR_H
#define CORE1_TRANSLATOR_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Inter-core lockless ring buffer for Makcu frames
#define MAKCU_FRAME_QUEUE_SIZE 32
#define KMBOX_CMD_QUEUE_SIZE 64

typedef struct {
    uint8_t cmd;
    uint8_t payload[256];
    uint16_t payload_len;
} queued_makcu_frame_t;

typedef struct {
    uint8_t packet[8];
} queued_kmbox_cmd_t;

/**
 * Initialize Core1 translator subsystem
 * Launches Core1 thread and sets up inter-core communication
 */
void core1_translator_init(void);

/**
 * Queue a Makcu frame for translation on Core1
 * @return true if queued, false if queue full
 */
bool core1_queue_makcu_frame(uint8_t cmd, const uint8_t* payload, uint16_t len);

/**
 * Check if translated KMBox commands are available
 */
bool core1_has_kmbox_commands(void);

/**
 * Dequeue a translated KMBox command (non-blocking)
 * @return true if command retrieved, false if queue empty
 */
bool core1_dequeue_kmbox_command(uint8_t* packet_out);

/**
 * Get translation statistics
 */
void core1_get_stats(uint32_t* frames_queued, uint32_t* cmds_produced, 
                     uint32_t* queue_overflows);

#endif // CORE1_TRANSLATOR_H
