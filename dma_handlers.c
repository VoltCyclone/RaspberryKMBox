/*
 * Generic DMA Handlers for non-RP2350 platforms
 * 
 * This file implements the DMA interrupt handlers for keyboard and mouse
 * data transfers on platforms that don't have the RP2350 hardware acceleration.
 */

#include "defines.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "usb_hid_types.h"
#include "dma_manager.h"
#include "tusb.h"

// No external declarations needed

/**
 * @brief DMA interrupt handler for keyboard HID data
 *
 * This function handles DMA completion interrupts for keyboard data transfers.
 * It is called automatically by the hardware when a keyboard DMA transfer completes.
 */
void dma_kbd_irq_handler(void) {
    // Get the DMA channel that triggered the interrupt
    int channel = DMA_CHANNEL_KEYBOARD;
    
    // Acknowledge the interrupt
    dma_channel_acknowledge_irq0(channel);
    
    // Process the keyboard data that was transferred via DMA
    // This would need to be implemented based on your specific DMA setup
    
    // Re-enable the DMA channel for the next transfer if needed
    // dma_channel_set_read_addr(channel, ...);
    // dma_channel_set_write_addr(channel, ...);
    // dma_channel_set_trans_count(channel, ...);
    // dma_channel_start(channel);
}

/**
 * @brief DMA interrupt handler for mouse HID data
 *
 * This function handles DMA completion interrupts for mouse data transfers.
 * It is called automatically by the hardware when a mouse DMA transfer completes.
 */
void dma_mouse_irq_handler(void) {
    // Get the DMA channel that triggered the interrupt
    int channel = DMA_CHANNEL_MOUSE;
    
    // Acknowledge the interrupt
    dma_channel_acknowledge_irq0(channel);
    
    // Process the mouse data that was transferred via DMA
    // This would need to be implemented based on your specific DMA setup
    
    // Re-enable the DMA channel for the next transfer if needed
    // dma_channel_set_read_addr(channel, ...);
    // dma_channel_set_write_addr(channel, ...);
    // dma_channel_set_trans_count(channel, ...);
    // dma_channel_start(channel);
}