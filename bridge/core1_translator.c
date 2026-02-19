/**
 * Core1 Protocol Translator Implementation
 */

#include "core1_translator.h"
#include "makcu_translator.h"
#include "ferrum_translator.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include <string.h>
#include <stdio.h>

// Lockless ring buffers for inter-core communication
static queued_makcu_frame_t makcu_queue[MAKCU_FRAME_QUEUE_SIZE] __attribute__((aligned(4)));
static queued_kmbox_cmd_t kmbox_queue[KMBOX_CMD_QUEUE_SIZE] __attribute__((aligned(4)));

// Ring buffer indices (volatile for inter-core visibility)
static volatile uint8_t makcu_head = 0;
static volatile uint8_t makcu_tail = 0;
static volatile uint8_t kmbox_head = 0;
static volatile uint8_t kmbox_tail = 0;

// Statistics
static volatile uint32_t frames_queued = 0;
static volatile uint32_t cmds_produced = 0;
static volatile uint32_t queue_overflows = 0;

// Core1 main translation loop
static void __not_in_flash_func(core1_main)(void) {
    // Run translation loop at max speed
    while (1) {
        // Check for Makcu frames to translate
        uint8_t head = makcu_head;
        uint8_t tail = makcu_tail;
        
        if (head != tail) {
            // Get frame from queue
            queued_makcu_frame_t* frame = &makcu_queue[tail & (MAKCU_FRAME_QUEUE_SIZE - 1)];
            
            // Translate to KMBox command(s)
            translated_cmd_t result;
            makcu_translate_command(frame->cmd, frame->payload, frame->payload_len, &result);
            
            // Queue translated wire protocol packet (max 7 bytes, fits in 8-byte slot)
            if (result.result == TRANSLATE_OK && result.length > 0) {
                uint8_t next_kmbox_head = (kmbox_head + 1) & (KMBOX_CMD_QUEUE_SIZE - 1);

                if (next_kmbox_head != kmbox_tail) {
                    memcpy(kmbox_queue[kmbox_head].packet, result.buffer, result.length);
                    kmbox_head = next_kmbox_head;
                    __atomic_add_fetch(&cmds_produced, 1, __ATOMIC_RELAXED);
                } else {
                    __atomic_add_fetch(&queue_overflows, 1, __ATOMIC_RELAXED);
                }
            }
            
            // Advance tail (consume frame)
            makcu_tail = (tail + 1) & (MAKCU_FRAME_QUEUE_SIZE - 1);
        } else {
            // No work - yield to save power
            __wfi();  // Wait for interrupt (low power idle)
        }
    }
}

// Initialize Core1 translator
void core1_translator_init(void) {
    // Reset queues
    makcu_head = 0;
    makcu_tail = 0;
    kmbox_head = 0;
    kmbox_tail = 0;
    
    frames_queued = 0;
    cmds_produced = 0;
    queue_overflows = 0;
    
    // Launch Core1
    multicore_launch_core1(core1_main);
    
    printf("Core1 translator launched\n");
}

// Queue Makcu frame for translation
bool core1_queue_makcu_frame(uint8_t cmd, const uint8_t* payload, uint16_t len) {
    if (len > 256) {
        return false;  // Payload too large
    }
    
    uint8_t next_head = (makcu_head + 1) & (MAKCU_FRAME_QUEUE_SIZE - 1);
    
    if (next_head == makcu_tail) {
        __atomic_add_fetch(&queue_overflows, 1, __ATOMIC_RELAXED);
        return false;  // Queue full
    }
    
    // Copy frame to queue
    queued_makcu_frame_t* frame = &makcu_queue[makcu_head];
    frame->cmd = cmd;
    frame->payload_len = len;
    if (payload && len > 0) {
        memcpy(frame->payload, payload, len);
    }
    
    // Fix #4: Data memory barrier - ensure all writes complete before publishing index
    __dmb();
    
    // Advance head (publish frame)
    makcu_head = next_head;
    __atomic_add_fetch(&frames_queued, 1, __ATOMIC_RELAXED);
    
    // Wake Core1 if idle
    __sev();  // Send event to wake WFI
    
    return true;
}

// Check if KMBox commands available
bool core1_has_kmbox_commands(void) {
    return kmbox_head != kmbox_tail;
}

// Dequeue translated KMBox command
bool core1_dequeue_kmbox_command(uint8_t* packet_out) {
    uint8_t head = kmbox_head;
    uint8_t tail = kmbox_tail;
    
    if (head == tail) {
        return false;  // Queue empty
    }
    
    // Copy packet
    memcpy(packet_out, kmbox_queue[tail].packet, 8);
    
    // Advance tail (consume command)
    kmbox_tail = (tail + 1) & (KMBOX_CMD_QUEUE_SIZE - 1);
    
    return true;
}

// Get statistics
void core1_get_stats(uint32_t* out_frames, uint32_t* out_cmds, uint32_t* out_overflows) {
    if (out_frames) *out_frames = __atomic_load_n(&frames_queued, __ATOMIC_RELAXED);
    if (out_cmds) *out_cmds = __atomic_load_n(&cmds_produced, __ATOMIC_RELAXED);
    if (out_overflows) *out_overflows = __atomic_load_n(&queue_overflows, __ATOMIC_RELAXED);
}
