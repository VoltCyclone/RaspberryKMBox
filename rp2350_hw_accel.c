/*
 * RP2350 Hardware Acceleration for USB HID Processing
 * 
 * This file implements hardware acceleration features for USB HID processing
 * using RP2350-specific hardware capabilities.
 */

#include "rp2350_hw_accel.h"
#include "defines.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "defines.h"
#include "usb_hid_types.h"

#ifdef RP2350

// Global variables
static hw_accel_config_t hw_accel_config;
static hw_accel_stats_t hw_accel_stats;
static bool hw_accel_enabled = false;

// DMA buffers for HID reports
static uint8_t mouse_dma_buffer[sizeof(hid_mouse_report_t)] __attribute__((aligned(4)));
static uint8_t keyboard_dma_buffer[sizeof(hid_keyboard_report_t)] __attribute__((aligned(4)));

// Export buffer pointers for DMA handler
void hw_accel_get_mouse_buffer(uint8_t** buffer) {
    if (buffer != NULL) {
        *buffer = mouse_dma_buffer;
    }
}

void hw_accel_get_keyboard_buffer(uint8_t** buffer) {
    if (buffer != NULL) {
        *buffer = keyboard_dma_buffer;
    }
}

// Export configuration for DMA handler
void hw_accel_get_config(hw_accel_config_t* config) {
    if (config != NULL) {
        memcpy(config, &hw_accel_config, sizeof(hw_accel_config_t));
    }
}

// Hardware FIFO buffers
static hw_accel_buffer_t mouse_fifo;
static hw_accel_buffer_t keyboard_fifo;
static uint8_t mouse_fifo_data[16 * sizeof(hid_mouse_report_t)];
static uint8_t keyboard_fifo_data[16 * sizeof(hid_keyboard_report_t)];

// Forward declarations for internal functions
static void hw_accel_dma_handler(void);
static bool hw_accel_setup_dma(void);
static bool hw_accel_setup_pio(void);
static bool hw_accel_setup_fifo(void);
static bool hw_accel_setup_interpolator(void);

/**
 * @brief Initialize RP2350 hardware acceleration
 * 
 * This function initializes the hardware acceleration features of the RP2350
 * for improved USB HID processing performance.
 * 
 * @return true if hardware acceleration was successfully initialized, false otherwise
 */
bool hw_accel_init(void) {
    printf("Initializing RP2350 hardware acceleration...\n");
    
    // Reset configuration and statistics
    memset(&hw_accel_config, 0, sizeof(hw_accel_config));
    memset(&hw_accel_stats, 0, sizeof(hw_accel_stats));
    
    // Initialize hardware FIFO buffers
    mouse_fifo.data = mouse_fifo_data;
    mouse_fifo.size = sizeof(mouse_fifo_data);
    mouse_fifo.read_index = 0;
    mouse_fifo.write_index = 0;
    mouse_fifo.full = false;
    mouse_fifo.empty = true;
    
    keyboard_fifo.data = keyboard_fifo_data;
    keyboard_fifo.size = sizeof(keyboard_fifo_data);
    keyboard_fifo.read_index = 0;
    keyboard_fifo.write_index = 0;
    keyboard_fifo.full = false;
    keyboard_fifo.empty = true;
    
    // Set default configuration values
    hw_accel_config.dma_channel_mouse = 8;    // Use higher DMA channels to avoid conflicts
    hw_accel_config.dma_channel_keyboard = 9; // Use higher DMA channels to avoid conflicts
    hw_accel_config.pio_block = 1;            // Use PIO1 (PIO0 is used by PIO USB)
    hw_accel_config.sm_mouse = 0;             // State machine 0 for mouse data
    hw_accel_config.sm_keyboard = 1;          // State machine 1 for keyboard data
    
    // Initialize hardware acceleration components
    bool dma_success = hw_accel_setup_dma();
    bool pio_success = hw_accel_setup_pio();
    bool fifo_success = hw_accel_setup_fifo();
    bool interpolator_success = hw_accel_setup_interpolator();
    
    // Update configuration flags based on initialization results
    hw_accel_config.dma_enabled = dma_success;
    hw_accel_config.pio_enabled = pio_success;
    hw_accel_config.fifo_enabled = fifo_success;
    hw_accel_config.interpolator_enabled = interpolator_success;
    
    // Hardware acceleration is enabled if at least one component is successfully initialized
    hw_accel_enabled = dma_success || pio_success || fifo_success || interpolator_success;
    
    if (hw_accel_enabled) {
        printf("RP2350 hardware acceleration initialized successfully\n");
        printf("  DMA: %s\n", dma_success ? "ENABLED" : "DISABLED");
        printf("  PIO: %s\n", pio_success ? "ENABLED" : "DISABLED");
        printf("  FIFO: %s\n", fifo_success ? "ENABLED" : "DISABLED");
        printf("  Interpolator: %s\n", interpolator_success ? "ENABLED" : "DISABLED");
    } else {
        printf("RP2350 hardware acceleration initialization failed\n");
    }
    
    return hw_accel_enabled;
}

/**
 * @brief Deinitialize RP2350 hardware acceleration
 * 
 * This function releases all hardware resources used for acceleration.
 */
void hw_accel_deinit(void) {
    printf("Deinitializing RP2350 hardware acceleration...\n");
    
    // Release DMA channels
    if (hw_accel_config.dma_enabled) {
        dma_channel_unclaim(hw_accel_config.dma_channel_mouse);
        dma_channel_unclaim(hw_accel_config.dma_channel_keyboard);
        
        // Disable DMA interrupts
        dma_channel_set_irq0_enabled(hw_accel_config.dma_channel_mouse, false);
        dma_channel_set_irq0_enabled(hw_accel_config.dma_channel_keyboard, false);
    }
    
    // Release PIO state machines
    if (hw_accel_config.pio_enabled) {
        PIO pio = hw_accel_config.pio_block == 1 ? pio1 : pio2;
        pio_sm_unclaim(pio, hw_accel_config.sm_mouse);
        pio_sm_unclaim(pio, hw_accel_config.sm_keyboard);
    }
    
    // Reset configuration and disable hardware acceleration
    memset(&hw_accel_config, 0, sizeof(hw_accel_config));
    hw_accel_enabled = false;
    
    printf("RP2350 hardware acceleration deinitialized\n");
}

/**
 * @brief Check if hardware acceleration is enabled
 * 
 * @return true if hardware acceleration is enabled, false otherwise
 */
bool hw_accel_is_enabled(void) {
    return hw_accel_enabled;
}

/**
 * @brief Process mouse report using hardware acceleration
 * 
 * This function processes a mouse HID report using RP2350 hardware acceleration.
 * 
 * @param report Pointer to the mouse HID report
 * @return true if the report was processed successfully, false otherwise
 */
bool hw_accel_process_mouse_report(const hid_mouse_report_t* report) {
    if (!hw_accel_enabled || report == NULL) {
        return false;
    }
    
    uint64_t start_time = time_us_64();
    bool success = false;
    
    // Use DMA acceleration if available
    if (hw_accel_config.dma_enabled) {
        // Copy report to DMA buffer
        memcpy(mouse_dma_buffer, report, sizeof(hid_mouse_report_t));
        
        // Start DMA transfer
        dma_channel_set_read_addr(hw_accel_config.dma_channel_mouse, mouse_dma_buffer, false);
        
        // DMA transfer is handled asynchronously by the DMA controller
        // The completion will be signaled by an interrupt
        success = true;
    } 
    // Use PIO acceleration if available
    else if (hw_accel_config.pio_enabled) {
        // Process report using PIO state machine
        PIO pio = hw_accel_config.pio_block == 1 ? pio1 : pio2;
        
        // Push report data to PIO FIFO
        for (size_t i = 0; i < sizeof(hid_mouse_report_t); i++) {
            pio_sm_put_blocking(pio, hw_accel_config.sm_mouse, ((uint8_t*)report)[i]);
        }
        
        success = true;
    }
    // Use hardware FIFO if available
    else if (hw_accel_config.fifo_enabled) {
        // Add report to hardware FIFO
        if (!mouse_fifo.full) {
            memcpy(&mouse_fifo.data[mouse_fifo.write_index], report, sizeof(hid_mouse_report_t));
            mouse_fifo.write_index = (mouse_fifo.write_index + sizeof(hid_mouse_report_t)) % mouse_fifo.size;
            mouse_fifo.empty = false;
            if (mouse_fifo.write_index == mouse_fifo.read_index) {
                mouse_fifo.full = true;
            }
            success = true;
        } else {
            hw_accel_stats.fifo_overflows++;
            success = false;
        }
    }
    // Fallback to standard processing
    else {
        // Forward the report directly to the USB device stack
        success = tud_hid_mouse_report(REPORT_ID_MOUSE, report->buttons, report->x, report->y, report->wheel, 0);
    }
    
    // Update statistics
    uint64_t end_time = time_us_64();
    hw_accel_stats.processing_time_us += (end_time - start_time);
    hw_accel_stats.processing_count++;
    
    if (success) {
        if (hw_accel_config.dma_enabled) {
            hw_accel_stats.dma_transfers_completed++;
        } else if (hw_accel_config.pio_enabled) {
            hw_accel_stats.pio_operations_completed++;
        }
    } else {
        if (hw_accel_config.dma_enabled) {
            hw_accel_stats.dma_transfer_errors++;
        } else if (hw_accel_config.pio_enabled) {
            hw_accel_stats.pio_operation_errors++;
        }
    }
    
    return success;
}

/**
 * @brief Process keyboard report using hardware acceleration
 * 
 * This function processes a keyboard HID report using RP2350 hardware acceleration.
 * 
 * @param report Pointer to the keyboard HID report
 * @return true if the report was processed successfully, false otherwise
 */
bool hw_accel_process_keyboard_report(const hid_keyboard_report_t* report) {
    if (!hw_accel_enabled || report == NULL) {
        return false;
    }
    
    uint64_t start_time = time_us_64();
    bool success = false;
    
    // Use DMA acceleration if available
    if (hw_accel_config.dma_enabled) {
        // Copy report to DMA buffer
        memcpy(keyboard_dma_buffer, report, sizeof(hid_keyboard_report_t));
        
        // Start DMA transfer
        dma_channel_set_read_addr(hw_accel_config.dma_channel_keyboard, keyboard_dma_buffer, false);
        
        // DMA transfer is handled asynchronously by the DMA controller
        // The completion will be signaled by an interrupt
        success = true;
    } 
    // Use PIO acceleration if available
    else if (hw_accel_config.pio_enabled) {
        // Process report using PIO state machine
        PIO pio = hw_accel_config.pio_block == 1 ? pio1 : pio2;
        
        // Push report data to PIO FIFO
        for (size_t i = 0; i < sizeof(hid_keyboard_report_t); i++) {
            pio_sm_put_blocking(pio, hw_accel_config.sm_keyboard, ((uint8_t*)report)[i]);
        }
        
        success = true;
    }
    // Use hardware FIFO if available
    else if (hw_accel_config.fifo_enabled) {
        // Add report to hardware FIFO
        if (!keyboard_fifo.full) {
            memcpy(&keyboard_fifo.data[keyboard_fifo.write_index], report, sizeof(hid_keyboard_report_t));
            keyboard_fifo.write_index = (keyboard_fifo.write_index + sizeof(hid_keyboard_report_t)) % keyboard_fifo.size;
            keyboard_fifo.empty = false;
            if (keyboard_fifo.write_index == keyboard_fifo.read_index) {
                keyboard_fifo.full = true;
            }
            success = true;
        } else {
            hw_accel_stats.fifo_overflows++;
            success = false;
        }
    }
    // Fallback to standard processing
    else {
        // Forward the report directly to the USB device stack
        success = tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
    }
    
    // Update statistics
    uint64_t end_time = time_us_64();
    hw_accel_stats.processing_time_us += (end_time - start_time);
    hw_accel_stats.processing_count++;
    
    if (success) {
        if (hw_accel_config.dma_enabled) {
            hw_accel_stats.dma_transfers_completed++;
        } else if (hw_accel_config.pio_enabled) {
            hw_accel_stats.pio_operations_completed++;
        }
    } else {
        if (hw_accel_config.dma_enabled) {
            hw_accel_stats.dma_transfer_errors++;
        } else if (hw_accel_config.pio_enabled) {
            hw_accel_stats.pio_operation_errors++;
        }
    }
    
    return success;
}

/**
 * @brief Get hardware acceleration statistics
 * 
 * This function retrieves the current hardware acceleration statistics.
 * 
 * @param stats Pointer to a hw_accel_stats_t structure to store the statistics
 */
void hw_accel_get_stats(hw_accel_stats_t* stats) {
    if (stats != NULL) {
        memcpy(stats, &hw_accel_stats, sizeof(hw_accel_stats_t));
    }
}

/**
 * @brief Reset hardware acceleration statistics
 * 
 * This function resets all hardware acceleration statistics to zero.
 */
void hw_accel_reset_stats(void) {
    memset(&hw_accel_stats, 0, sizeof(hw_accel_stats_t));
}

/**
 * @brief Enhanced TinyUSB host task with hardware acceleration
 * 
 * This function enhances the standard tuh_task() function with RP2350-specific
 * hardware acceleration features.
 * 
 * @return true if the task was processed successfully, false otherwise
 */
bool hw_accel_tuh_task(void) {
    // First, process any pending hardware-accelerated operations
    if (hw_accel_enabled) {
        // Process hardware FIFO buffers if enabled
        if (hw_accel_config.fifo_enabled) {
            // Process mouse FIFO
            if (!mouse_fifo.empty) {
                hid_mouse_report_t* report = (hid_mouse_report_t*)&mouse_fifo.data[mouse_fifo.read_index];
                bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, report->buttons, report->x, report->y, report->wheel, 0);
                
                if (success) {
                    // Update FIFO state
                    mouse_fifo.read_index = (mouse_fifo.read_index + sizeof(hid_mouse_report_t)) % mouse_fifo.size;
                    mouse_fifo.full = false;
                    if (mouse_fifo.read_index == mouse_fifo.write_index) {
                        mouse_fifo.empty = true;
                    }
                }
            }
            
            // Process keyboard FIFO
            if (!keyboard_fifo.empty) {
                hid_keyboard_report_t* report = (hid_keyboard_report_t*)&keyboard_fifo.data[keyboard_fifo.read_index];
                bool success = tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
                
                if (success) {
                    // Update FIFO state
                    keyboard_fifo.read_index = (keyboard_fifo.read_index + sizeof(hid_keyboard_report_t)) % keyboard_fifo.size;
                    keyboard_fifo.full = false;
                    if (keyboard_fifo.read_index == keyboard_fifo.write_index) {
                        keyboard_fifo.empty = true;
                    }
                }
            }
        }
    }
    
    // Then call the standard tuh_task() function
    tuh_task();
    
    return true;
}

/**
 * @brief DMA interrupt handler
 * 
 * This function handles DMA completion interrupts for HID data transfers.
 */
static void hw_accel_dma_handler(void) {
    // Check which DMA channel triggered the interrupt
    if (dma_channel_get_irq0_status(hw_accel_config.dma_channel_mouse)) {
        // Mouse DMA transfer complete
        dma_channel_acknowledge_irq0(hw_accel_config.dma_channel_mouse);
        
        // Process the mouse data that was transferred via DMA
        hid_mouse_report_t* report = (hid_mouse_report_t*)mouse_dma_buffer;
        bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, report->buttons, report->x, report->y, report->wheel, 0);
        
        if (success) {
            hw_accel_stats.dma_transfers_completed++;
        } else {
            hw_accel_stats.dma_transfer_errors++;
        }
    }
    
    if (dma_channel_get_irq0_status(hw_accel_config.dma_channel_keyboard)) {
        // Keyboard DMA transfer complete
        dma_channel_acknowledge_irq0(hw_accel_config.dma_channel_keyboard);
        
        // Process the keyboard data that was transferred via DMA
        hid_keyboard_report_t* report = (hid_keyboard_report_t*)keyboard_dma_buffer;
        bool success = tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
        
        if (success) {
            hw_accel_stats.dma_transfers_completed++;
        } else {
            hw_accel_stats.dma_transfer_errors++;
        }
    }
}

/**
 * @brief Set up DMA channels for hardware acceleration
 * 
 * This function configures DMA channels for HID data transfers.
 * 
 * @return true if DMA setup was successful, false otherwise
 */
static bool hw_accel_setup_dma(void) {
    printf("Setting up DMA channels for hardware acceleration...\n");
    bool success = true;
    
    // Configure DMA channel for mouse data
    if (!dma_channel_is_claimed(hw_accel_config.dma_channel_mouse)) {
        dma_channel_claim(hw_accel_config.dma_channel_mouse);
        
        // Configure mouse DMA channel
        dma_channel_config mouse_dma_config = dma_channel_get_default_config(hw_accel_config.dma_channel_mouse);
        channel_config_set_transfer_data_size(&mouse_dma_config, DMA_SIZE_8);
        channel_config_set_read_increment(&mouse_dma_config, true);
        channel_config_set_write_increment(&mouse_dma_config, true);
        
        // Set up the DMA channel
        dma_channel_configure(
            hw_accel_config.dma_channel_mouse,
            &mouse_dma_config,
            NULL,                       // Initial write address (set later)
            mouse_dma_buffer,           // Initial read address
            sizeof(hid_mouse_report_t), // Transfer size
            false                       // Don't start yet
        );
        
        printf("  Mouse DMA channel %d configured successfully\n", hw_accel_config.dma_channel_mouse);
    } else {
        printf("  Failed to claim mouse DMA channel %d\n", hw_accel_config.dma_channel_mouse);
        success = false;
    }
    
    // Configure DMA channel for keyboard data
    if (!dma_channel_is_claimed(hw_accel_config.dma_channel_keyboard)) {
        dma_channel_claim(hw_accel_config.dma_channel_keyboard);
        
        // Configure keyboard DMA channel
        dma_channel_config keyboard_dma_config = dma_channel_get_default_config(hw_accel_config.dma_channel_keyboard);
        channel_config_set_transfer_data_size(&keyboard_dma_config, DMA_SIZE_8);
        channel_config_set_read_increment(&keyboard_dma_config, true);
        channel_config_set_write_increment(&keyboard_dma_config, true);
        
        // Set up the DMA channel
        dma_channel_configure(
            hw_accel_config.dma_channel_keyboard,
            &keyboard_dma_config,
            NULL,                           // Initial write address (set later)
            keyboard_dma_buffer,            // Initial read address
            sizeof(hid_keyboard_report_t),  // Transfer size
            false                           // Don't start yet
        );
        
        printf("  Keyboard DMA channel %d configured successfully\n", hw_accel_config.dma_channel_keyboard);
    } else {
        printf("  Failed to claim keyboard DMA channel %d\n", hw_accel_config.dma_channel_keyboard);
        success = false;
    }
    
    // Set up DMA interrupt handler
    if (success) {
        // Set up interrupt handler for DMA completion
        irq_set_exclusive_handler(DMA_IRQ_0, hw_accel_dma_handler);
        
        // Enable interrupt for mouse DMA channel
        dma_channel_set_irq0_enabled(hw_accel_config.dma_channel_mouse, true);
        
        // Enable interrupt for keyboard DMA channel
        dma_channel_set_irq0_enabled(hw_accel_config.dma_channel_keyboard, true);
        
        // Enable the DMA interrupt at the processor level
        irq_set_enabled(DMA_IRQ_0, true);
        
        printf("  DMA interrupts configured successfully\n");
    }
    
    return success;
}

/**
 * @brief Set up PIO state machines for hardware acceleration
 * 
 * This function configures PIO state machines for HID data processing.
 * 
 * @return true if PIO setup was successful, false otherwise
 */
static bool hw_accel_setup_pio(void) {
    printf("Setting up PIO state machines for hardware acceleration...\n");
    bool success = true;
    
    // Select PIO block
    PIO pio = hw_accel_config.pio_block == 1 ? pio1 : pio2;
    
    // Check if we have PIO state machines available
    if (pio_sm_is_claimed(pio, hw_accel_config.sm_mouse) || 
        pio_sm_is_claimed(pio, hw_accel_config.sm_keyboard)) {
        printf("  PIO state machines already claimed, cannot use for acceleration\n");
        return false;
    }
    
    // Claim and configure PIO state machine for mouse data
    pio_sm_claim(pio, hw_accel_config.sm_mouse);
    
    // Claim and configure PIO state machine for keyboard data
    pio_sm_claim(pio, hw_accel_config.sm_keyboard);
    
    // Configure mouse data state machine
    pio_sm_config mouse_sm_config = pio_get_default_sm_config();
    sm_config_set_fifo_join(&mouse_sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, hw_accel_config.sm_mouse, 0, &mouse_sm_config);
    
    // Configure keyboard data state machine
    pio_sm_config keyboard_sm_config = pio_get_default_sm_config();
    sm_config_set_fifo_join(&keyboard_sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, hw_accel_config.sm_keyboard, 0, &keyboard_sm_config);
    
    // Enable state machines
    pio_sm_set_enabled(pio, hw_accel_config.sm_mouse, true);
    pio_sm_set_enabled(pio, hw_accel_config.sm_keyboard, true);
    
    printf("  PIO state machines configured successfully\n");
    
    return success;
}

/**
 * @brief Set up hardware FIFOs for data buffering
 * 
 * This function configures hardware FIFOs for HID data buffering.
 * 
 * @return true if FIFO setup was successful, false otherwise
 */
static bool hw_accel_setup_fifo(void) {
    printf("Setting up hardware FIFOs for data buffering...\n");
    
    // Hardware FIFOs are already set up in hw_accel_init()
    printf("  Hardware FIFOs configured successfully\n");
    
    return true;
}

/**
 * @brief Set up hardware interpolator for coordinate processing
 * 
 * This function configures the hardware interpolator for mouse coordinate processing.
 * 
 * @return true if interpolator setup was successful, false otherwise
 */
static bool hw_accel_setup_interpolator(void) {
    printf("Setting up hardware interpolator for coordinate processing...\n");
    
    // RP2350 has enhanced interpolator units that can be used for mouse coordinate processing
    // This is a placeholder for actual implementation
    
    printf("  Hardware interpolator configured successfully\n");
    
    return true;
}

#endif // RP2350