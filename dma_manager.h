/**
 * @file dma_manager.h
 * @brief Centralized DMA channel management system
 * 
 * This module provides explicit DMA channel assignments and management
 * for the Raspberry Pi Pico project, replacing dynamic channel allocation
 * with a centralized system that tracks channel usage and prevents conflicts.
 */

#ifndef DMA_MANAGER_H
#define DMA_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/dma.h"
#include "pico/mutex.h"

// Total number of DMA channels available on RP2040
#define DMA_NUM_CHANNELS 12

// Core-specific channel reservations
#define DMA_CORE0_CHANNEL_START 0
#define DMA_CORE0_CHANNEL_END   5
#define DMA_CORE1_CHANNEL_START 6
#define DMA_CORE1_CHANNEL_END   11

// DMA channel assignments
// Core 0 channels
#define DMA_CHANNEL_KEYBOARD    0  // Keyboard HID reports
#define DMA_CHANNEL_MOUSE       1  // Mouse HID reports
// Core 0 or shared channels
#define DMA_CHANNEL_PIO_USB_TX  2  // PIO USB transmit
// Core 1 channels (if used)
// Add more channel definitions as needed

// DMA channel status
typedef enum {
    DMA_CHANNEL_STATUS_FREE = 0,
    DMA_CHANNEL_STATUS_RESERVED,
    DMA_CHANNEL_STATUS_IN_USE
} dma_channel_status_t;

// DMA channel information
typedef struct {
    dma_channel_status_t status;
    const char* owner;           // String identifier of the module using this channel
    uint8_t core_num;            // Core number that reserved this channel (0 or 1)
} dma_channel_info_t;

/**
 * @brief Initialize the DMA manager
 * 
 * Must be called before any other DMA manager functions
 */
void dma_manager_init(void);

/**
 * @brief Request a specific DMA channel
 * 
 * @param channel The specific DMA channel to request
 * @param owner String identifier of the module requesting the channel
 * @return true if the channel was successfully reserved, false if already in use
 */
bool dma_manager_request_channel(uint channel, const char* owner);

/**
 * @brief Release a previously requested DMA channel
 * 
 * @param channel The DMA channel to release
 * @return true if the channel was successfully released, false if not owned
 */
bool dma_manager_release_channel(uint channel);

/**
 * @brief Check if a DMA channel is available
 * 
 * @param channel The DMA channel to check
 * @return true if the channel is available, false if already in use
 */
bool dma_manager_is_channel_available(uint channel);

/**
 * @brief Get the owner of a DMA channel
 * 
 * @param channel The DMA channel to check
 * @return const char* String identifier of the module using this channel, or NULL if not in use
 */
const char* dma_manager_get_channel_owner(uint channel);

/**
 * @brief Validate all DMA channel assignments
 * 
 * Checks for any conflicts in DMA channel assignments and logs errors
 * 
 * @return true if all channel assignments are valid, false if conflicts exist
 */
bool dma_manager_validate_channels(void);

/**
 * @brief Print the status of all DMA channels
 * 
 * Useful for debugging DMA channel assignments
 */
void dma_manager_print_status(void);

/**
 * @brief Get a channel reserved for the current core
 * 
 * @param owner String identifier of the module requesting the channel
 * @return int The assigned channel number, or -1 if no channels available
 */
int dma_manager_get_core_channel(const char* owner);

#endif // DMA_MANAGER_H