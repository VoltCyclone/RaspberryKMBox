/*
 * RP2350 Hardware Acceleration for USB HID Processing
 * 
 * This header defines structures and functions for utilizing RP2350-specific
 * hardware acceleration features for improved USB HID processing performance.
 */

#ifndef RP2350_HW_ACCEL_H
#define RP2350_HW_ACCEL_H

#include <stdbool.h>
#include <stdint.h>
#include "tusb.h"
#include "class/hid/hid_host.h"

#ifdef RP2350

// Hardware acceleration configuration
typedef struct {
    uint8_t dma_channel_mouse;      // DMA channel for mouse data
    uint8_t dma_channel_keyboard;   // DMA channel for keyboard data
    uint8_t pio_block;              // PIO block used for acceleration
    uint8_t sm_mouse;               // State machine for mouse data
    uint8_t sm_keyboard;            // State machine for keyboard data
    bool dma_enabled;               // DMA acceleration enabled
    bool pio_enabled;               // PIO acceleration enabled
    bool fifo_enabled;              // Hardware FIFO enabled
    bool interpolator_enabled;      // Hardware interpolator enabled
} hw_accel_config_t;

// Hardware acceleration statistics
typedef struct {
    uint32_t dma_transfers_completed;   // Number of DMA transfers completed
    uint32_t dma_transfer_errors;       // Number of DMA transfer errors
    uint32_t pio_operations_completed;  // Number of PIO operations completed
    uint32_t pio_operation_errors;      // Number of PIO operation errors
    uint32_t fifo_overflows;            // Number of FIFO overflows
    uint32_t fifo_underflows;           // Number of FIFO underflows
    uint64_t processing_time_us;        // Total processing time in microseconds
    uint32_t processing_count;          // Number of processing operations
} hw_accel_stats_t;

// Hardware acceleration buffer
typedef struct {
    uint8_t* data;                  // Buffer data
    uint32_t size;                  // Buffer size
    uint32_t read_index;            // Current read position
    uint32_t write_index;           // Current write position
    bool full;                      // Buffer full flag
    bool empty;                     // Buffer empty flag
} hw_accel_buffer_t;

// Function declarations
bool hw_accel_init(void);
void hw_accel_deinit(void);
bool hw_accel_is_enabled(void);
bool hw_accel_process_mouse_report(const hid_mouse_report_t* report);
bool hw_accel_process_keyboard_report(const hid_keyboard_report_t* report);
void hw_accel_get_stats(hw_accel_stats_t* stats);
void hw_accel_reset_stats(void);
void hw_accel_get_config(hw_accel_config_t* config);
void hw_accel_get_mouse_buffer(uint8_t** buffer);
void hw_accel_get_keyboard_buffer(uint8_t** buffer);

// Enhanced TinyUSB host task with hardware acceleration
bool hw_accel_tuh_task(void);

#endif // RP2350

#endif // RP2350_HW_ACCEL_H