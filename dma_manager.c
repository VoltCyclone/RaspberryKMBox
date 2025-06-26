/**
 * @file dma_manager.c
 * @brief Implementation of centralized DMA channel management system
 */

#include "dma_manager.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

// Static array to track DMA channel status
static dma_channel_info_t dma_channels[DMA_NUM_CHANNELS];

// Mutex for thread-safe access to the DMA channel status
static mutex_t dma_mutex;

// Initialize the DMA manager
void dma_manager_init(void) {
    // Initialize the mutex
    mutex_init(&dma_mutex);
    
    // Initialize all channels as free
    for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
        dma_channels[i].status = DMA_CHANNEL_STATUS_FREE;
        dma_channels[i].owner = NULL;
        dma_channels[i].core_num = 0xFF; // Invalid core number
    }
    
    printf("DMA Manager: Initialized with %d channels\n", DMA_NUM_CHANNELS);
    printf("DMA Manager: Core 0 channels: %d-%d\n", DMA_CORE0_CHANNEL_START, DMA_CORE0_CHANNEL_END);
    printf("DMA Manager: Core 1 channels: %d-%d\n", DMA_CORE1_CHANNEL_START, DMA_CORE1_CHANNEL_END);
}

// Request a specific DMA channel
bool dma_manager_request_channel(uint channel, const char* owner) {
    bool success = false;
    
    // Validate channel number
    if (channel >= DMA_NUM_CHANNELS) {
        printf("DMA Manager: Invalid channel number %d\n", channel);
        return false;
    }
    
    // Get current core number
    uint core_num = get_core_num();
    
    // Check if the channel is in the appropriate range for this core
    bool is_valid_for_core = false;
    if (core_num == 0 && channel >= DMA_CORE0_CHANNEL_START && channel <= DMA_CORE0_CHANNEL_END) {
        is_valid_for_core = true;
    } else if (core_num == 1 && channel >= DMA_CORE1_CHANNEL_START && channel <= DMA_CORE1_CHANNEL_END) {
        is_valid_for_core = true;
    }
    
    if (!is_valid_for_core) {
        printf("DMA Manager: Channel %d is not valid for core %d\n", channel, core_num);
        // We'll still allow it, but warn about it
        printf("DMA Manager: WARNING - Using channel outside of core's reserved range\n");
    }
    
    // Acquire mutex for thread-safe access
    mutex_enter_blocking(&dma_mutex);
    
    // Check if the channel is available
    if (dma_channels[channel].status == DMA_CHANNEL_STATUS_FREE) {
        // Reserve the channel
        dma_channels[channel].status = DMA_CHANNEL_STATUS_RESERVED;
        dma_channels[channel].owner = owner;
        dma_channels[channel].core_num = core_num;
        
        // Claim the channel in hardware
        dma_claim_mask(1u << channel);
        
        success = true;
        printf("DMA Manager: Channel %d reserved by '%s' on core %d\n", channel, owner, core_num);
    } else {
        printf("DMA Manager: Channel %d already in use by '%s'\n", 
               channel, dma_channels[channel].owner);
    }
    
    // Release mutex
    mutex_exit(&dma_mutex);
    
    return success;
}

// Release a previously requested DMA channel
bool dma_manager_release_channel(uint channel) {
    bool success = false;
    
    // Validate channel number
    if (channel >= DMA_NUM_CHANNELS) {
        printf("DMA Manager: Invalid channel number %d\n", channel);
        return false;
    }
    
    // Get current core number
    uint core_num = get_core_num();
    
    // Acquire mutex for thread-safe access
    mutex_enter_blocking(&dma_mutex);
    
    // Check if the channel is in use
    if (dma_channels[channel].status != DMA_CHANNEL_STATUS_FREE) {
        // Check if the channel is being released by the same core that reserved it
        if (dma_channels[channel].core_num != core_num) {
            printf("DMA Manager: WARNING - Channel %d being released by core %d but was reserved by core %d\n",
                   channel, core_num, dma_channels[channel].core_num);
        }
        
        // Release the channel
        dma_channels[channel].status = DMA_CHANNEL_STATUS_FREE;
        printf("DMA Manager: Channel %d released (was owned by '%s')\n", 
               channel, dma_channels[channel].owner);
        dma_channels[channel].owner = NULL;
        dma_channels[channel].core_num = 0xFF; // Invalid core number
        
        // Release the channel in hardware
        dma_channel_unclaim(channel);
        
        success = true;
    } else {
        printf("DMA Manager: Channel %d is not in use\n", channel);
    }
    
    // Release mutex
    mutex_exit(&dma_mutex);
    
    return success;
}

// Check if a DMA channel is available
bool dma_manager_is_channel_available(uint channel) {
    // Validate channel number
    if (channel >= DMA_NUM_CHANNELS) {
        return false;
    }
    
    bool available = false;
    
    // Acquire mutex for thread-safe access
    mutex_enter_blocking(&dma_mutex);
    
    // Check if the channel is free
    available = (dma_channels[channel].status == DMA_CHANNEL_STATUS_FREE);
    
    // Release mutex
    mutex_exit(&dma_mutex);
    
    return available;
}

// Get the owner of a DMA channel
const char* dma_manager_get_channel_owner(uint channel) {
    // Validate channel number
    if (channel >= DMA_NUM_CHANNELS) {
        return NULL;
    }
    
    const char* owner = NULL;
    
    // Acquire mutex for thread-safe access
    mutex_enter_blocking(&dma_mutex);
    
    // Get the owner
    owner = dma_channels[channel].owner;
    
    // Release mutex
    mutex_exit(&dma_mutex);
    
    return owner;
}

// Validate all DMA channel assignments
bool dma_manager_validate_channels(void) {
    bool valid = true;
    
    // Acquire mutex for thread-safe access
    mutex_enter_blocking(&dma_mutex);
    
    // Check for any conflicts
    printf("DMA Manager: Validating channel assignments...\n");
    
    for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
        if (dma_channels[i].status != DMA_CHANNEL_STATUS_FREE) {
            // Check if this channel is in the correct range for its core
            uint core_num = dma_channels[i].core_num;
            bool is_valid_for_core = false;
            
            if (core_num == 0 && i >= DMA_CORE0_CHANNEL_START && i <= DMA_CORE0_CHANNEL_END) {
                is_valid_for_core = true;
            } else if (core_num == 1 && i >= DMA_CORE1_CHANNEL_START && i <= DMA_CORE1_CHANNEL_END) {
                is_valid_for_core = true;
            }
            
            if (!is_valid_for_core) {
                printf("DMA Manager: WARNING - Channel %d is used by core %d but is outside its reserved range\n",
                       i, core_num);
                // Don't fail validation for this, just warn
            }
            
            printf("DMA Manager: Channel %d is used by '%s' on core %d\n", 
                   i, dma_channels[i].owner, dma_channels[i].core_num);
        }
    }
    
    // Release mutex
    mutex_exit(&dma_mutex);
    
    return valid;
}

// Print the status of all DMA channels
void dma_manager_print_status(void) {
    // Acquire mutex for thread-safe access
    mutex_enter_blocking(&dma_mutex);
    
    printf("DMA Manager: Channel Status\n");
    printf("-------------------------\n");
    
    for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
        printf("Channel %2d: ", i);
        
        switch (dma_channels[i].status) {
            case DMA_CHANNEL_STATUS_FREE:
                printf("FREE\n");
                break;
            case DMA_CHANNEL_STATUS_RESERVED:
                printf("RESERVED by '%s' on core %d\n", 
                       dma_channels[i].owner, dma_channels[i].core_num);
                break;
            case DMA_CHANNEL_STATUS_IN_USE:
                printf("IN USE by '%s' on core %d\n", 
                       dma_channels[i].owner, dma_channels[i].core_num);
                break;
            default:
                printf("UNKNOWN STATUS\n");
                break;
        }
    }
    
    printf("-------------------------\n");
    
    // Release mutex
    mutex_exit(&dma_mutex);
}

// Get a channel reserved for the current core
int dma_manager_get_core_channel(const char* owner) {
    int assigned_channel = -1;
    uint core_num = get_core_num();
    
    // Determine the range of channels for this core
    uint start_channel = (core_num == 0) ? DMA_CORE0_CHANNEL_START : DMA_CORE1_CHANNEL_START;
    uint end_channel = (core_num == 0) ? DMA_CORE0_CHANNEL_END : DMA_CORE1_CHANNEL_END;
    
    // Acquire mutex for thread-safe access
    mutex_enter_blocking(&dma_mutex);
    
    // Look for a free channel in the core's range
    for (uint i = start_channel; i <= end_channel; i++) {
        if (dma_channels[i].status == DMA_CHANNEL_STATUS_FREE) {
            // Found a free channel, reserve it
            dma_channels[i].status = DMA_CHANNEL_STATUS_RESERVED;
            dma_channels[i].owner = owner;
            dma_channels[i].core_num = core_num;
            
            // Claim the channel in hardware
            dma_claim_mask(1u << i);
            
            assigned_channel = i;
            printf("DMA Manager: Assigned channel %d to '%s' on core %d\n", 
                   i, owner, core_num);
            break;
        }
    }
    
    if (assigned_channel == -1) {
        printf("DMA Manager: No free channels available for core %d\n", core_num);
    }
    
    // Release mutex
    mutex_exit(&dma_mutex);
    
    return assigned_channel;
}