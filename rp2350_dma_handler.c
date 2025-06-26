/*
 * RP2350 DMA Handler for Hardware Acceleration
 * 
 * This file implements the DMA interrupt handler used for hardware-accelerated
 * HID processing on the RP2350.
 */

#include "rp2350_dma_handler.h"
#include "defines.h"
#include <stdio.h>
#include "usb_hid_types.h"

#ifdef RP2350
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "tusb.h"
#include "class/hid/hid.h"
#include "usb_hid_types.h"
#include "rp2350_hw_accel.h"

// External declarations
extern performance_stats_t stats;

/**
 * @brief DMA interrupt handler for hardware-accelerated HID processing
 *
 * This function handles DMA completion interrupts for HID data transfers.
 * It is called automatically by the hardware when a DMA transfer completes.
 */
void dma_handler(void) {
    // Get the hardware acceleration configuration
    hw_accel_config_t config;
    hw_accel_get_config(&config);
    
    // Check which DMA channel triggered the interrupt
    if (dma_channel_get_irq0_status(config.dma_channel_mouse)) {
        // Mouse DMA transfer complete
        dma_channel_acknowledge_irq0(config.dma_channel_mouse);
        
        // Get the mouse data buffer
        uint8_t* mouse_buffer;
        hw_accel_get_mouse_buffer(&mouse_buffer);
        
        // Process the mouse data that was transferred via DMA
        if (mouse_buffer != NULL) {
            hid_mouse_report_t* report = (hid_mouse_report_t*)mouse_buffer;
            
            // Validate buttons (keep only first 3 bits for L/R/M buttons)
            uint8_t valid_buttons = report->buttons & 0x07;
            
            // Forward the report to the USB device stack
            bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, valid_buttons, 
                                               report->x, report->y, report->wheel, 0);
            
            // Update statistics
            if (success) {
                stats.mouse_reports_forwarded++;
                stats.hw_accel_reports_processed++;
            } else {
                stats.forwarding_errors++;
                stats.hw_accel_errors++;
            }
        }
    }
    
    if (dma_channel_get_irq0_status(config.dma_channel_keyboard)) {
        // Keyboard DMA transfer complete
        dma_channel_acknowledge_irq0(config.dma_channel_keyboard);
        
        // Get the keyboard data buffer
        uint8_t* keyboard_buffer;
        hw_accel_get_keyboard_buffer(&keyboard_buffer);
        
        // Process the keyboard data that was transferred via DMA
        if (keyboard_buffer != NULL) {
            hid_keyboard_report_t* report = (hid_keyboard_report_t*)keyboard_buffer;
            
            // Forward the report to the USB device stack
            bool success = tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
            
            // Update statistics
            if (success) {
                stats.keyboard_reports_forwarded++;
                stats.hw_accel_reports_processed++;
            } else {
                stats.forwarding_errors++;
                stats.hw_accel_errors++;
            }
        }
    }
}

#endif // RP2350