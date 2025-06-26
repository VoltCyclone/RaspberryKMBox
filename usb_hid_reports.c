/*
 * Hurricane PIOKMbox Firmware
 */

#include "usb_hid_reports.h"
#include "defines.h"
#include "led_control.h"
#include <stdio.h>

#ifdef RP2350
#include "rp2350_hw_accel.h"
#endif

// External declarations for variables defined in other modules
extern performance_stats_t stats;

// Forward declarations for static functions
static bool process_keyboard_report_internal(const hid_keyboard_report_t* report);
static bool process_mouse_report_internal(const hid_mouse_report_t* report);

#ifdef RP2350
// RP2350 hardware-accelerated implementations
extern bool hw_accel_process_keyboard_report(const hid_keyboard_report_t* report);
extern bool hw_accel_process_mouse_report(const hid_mouse_report_t* report);
extern bool hw_accel_is_enabled(void);
#endif

// Word-aligned circular buffers for DMA transfers
static __attribute__((aligned(4))) hid_keyboard_report_t kbd_buffer[KBD_BUFFER_SIZE];
static __attribute__((aligned(4))) hid_mouse_report_t mouse_buffer[MOUSE_BUFFER_SIZE];

// Circular buffer control structures
static dma_circular_buffer_t kbd_circular_buffer;
static dma_circular_buffer_t mouse_circular_buffer;

// DMA channel handles
static int kbd_dma_channel;
static int mouse_dma_channel;

// Spinlocks for thread-safe buffer access
static spin_lock_t *kbd_spinlock;
static spin_lock_t *mouse_spinlock;

// Include DMA manager
#include "dma_manager.h"

// Initialize DMA channels and circular buffers
void init_hid_dma(void) {
    // Initialize circular buffer structures
    kbd_circular_buffer.read_idx = 0;
    kbd_circular_buffer.write_idx = 0;
    kbd_circular_buffer.size = KBD_BUFFER_SIZE;
    kbd_circular_buffer.mask = KBD_BUFFER_SIZE - 1;
    kbd_circular_buffer.buffer = kbd_buffer;

    mouse_circular_buffer.read_idx = 0;
    mouse_circular_buffer.write_idx = 0;
    mouse_circular_buffer.size = MOUSE_BUFFER_SIZE;
    mouse_circular_buffer.mask = MOUSE_BUFFER_SIZE - 1;
    mouse_circular_buffer.buffer = mouse_buffer;

    // Request specific DMA channels from the DMA manager
    if (!dma_manager_request_channel(DMA_CHANNEL_KEYBOARD, "HID Keyboard")) {
        // Failed to get the keyboard DMA channel
        LOG_ERROR("Failed to request keyboard DMA channel");
        return;
    }
    kbd_dma_channel = DMA_CHANNEL_KEYBOARD;
    
    if (!dma_manager_request_channel(DMA_CHANNEL_MOUSE, "HID Mouse")) {
        // Failed to get the mouse DMA channel
        LOG_ERROR("Failed to request mouse DMA channel");
        // Release the keyboard channel we already claimed
        dma_manager_release_channel(DMA_CHANNEL_KEYBOARD);
        return;
    }
    mouse_dma_channel = DMA_CHANNEL_MOUSE;

    // Get spinlocks for thread safety
    kbd_spinlock = spin_lock_init(spin_lock_claim_unused(true));
    mouse_spinlock = spin_lock_init(spin_lock_claim_unused(true));

    // Configure keyboard DMA channel
    dma_channel_config kbd_config = dma_channel_get_default_config(kbd_dma_channel);
    channel_config_set_transfer_data_size(&kbd_config, DMA_SIZE_32);
    channel_config_set_read_increment(&kbd_config, true);
    channel_config_set_write_increment(&kbd_config, false);
    channel_config_set_dreq(&kbd_config, DREQ_FORCE);
    
    dma_channel_configure(
        kbd_dma_channel,
        &kbd_config,
        NULL,                              // Dest address set during transfer
        NULL,                              // Source address set during transfer
        sizeof(hid_keyboard_report_t) / 4, // Transfer size in words
        false                              // Don't start immediately
    );

    // Configure mouse DMA channel
    dma_channel_config mouse_config = dma_channel_get_default_config(mouse_dma_channel);
    channel_config_set_transfer_data_size(&mouse_config, DMA_SIZE_32);
    channel_config_set_read_increment(&mouse_config, true);
    channel_config_set_write_increment(&mouse_config, false);
    channel_config_set_dreq(&mouse_config, DREQ_FORCE);
    
    dma_channel_configure(
        mouse_dma_channel,
        &mouse_config,
        NULL,                             // Dest address set during transfer
        NULL,                             // Source address set during transfer
        sizeof(hid_mouse_report_t) / 4,   // Transfer size in words
        false                             // Don't start immediately
    );

    // Set up DMA interrupts
    dma_channel_set_irq0_enabled(kbd_dma_channel, true);
    dma_channel_set_irq0_enabled(mouse_dma_channel, true);
    
    irq_set_exclusive_handler(DMA_IRQ_0, dma_kbd_irq_handler);
    irq_set_priority(DMA_IRQ_0, DMA_IRQ_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);
    
    irq_set_exclusive_handler(DMA_IRQ_1, dma_mouse_irq_handler);
    irq_set_priority(DMA_IRQ_1, DMA_IRQ_PRIORITY);
    irq_set_enabled(DMA_IRQ_1, true);
    
    LOG_INIT("DMA HID report processing initialized");
}

// Process keyboard report - queue to circular buffer
void process_kbd_report(const hid_keyboard_report_t* report)
{
    if (report == NULL) {
        return; // Fast fail without printf for performance
    }
    
    // Reduced activity flash frequency for better performance
    static uint32_t activity_counter = 0;
    if (++activity_counter % KEYBOARD_ACTIVITY_THROTTLE == 0) {
        neopixel_trigger_keyboard_activity();
    }
    
    // Skip key press processing for console output to improve performance
    // Only forward the report for maximum speed
    
    // Fast forward the report using hardware acceleration if available
#ifdef RP2350
    if (hw_accel_is_enabled() && hw_accel_process_keyboard_report(report)) {
        stats.keyboard_reports_received++;
        
#ifdef RP2350
        // Update RP2350 hardware acceleration statistics
        stats.hw_accel_reports_processed++;
#endif
    } else {
        if (process_keyboard_report_internal(report)) {
            stats.keyboard_reports_received++;
            
#ifdef RP2350
            // Update RP2350 software fallback statistics
            stats.sw_fallback_reports_processed++;
#endif
        }
    }
#else
    if (process_keyboard_report_internal(report)) {
        stats.keyboard_reports_received++;
    }
}

// Process mouse report - queue to circular buffer
void process_mouse_report(const hid_mouse_report_t* report)
{
    if (report == NULL) {
        return; // Fast fail without printf for performance
    }
    
    // Reduced activity flash frequency for better performance
    static uint32_t activity_counter = 0;
    if (++activity_counter % MOUSE_ACTIVITY_THROTTLE == 0) {
        neopixel_trigger_mouse_activity();
    }
    
    // Fast forward the report using hardware acceleration if available
#ifdef RP2350
    if (hw_accel_is_enabled() && hw_accel_process_mouse_report(report)) {
        stats.mouse_reports_received++;
        
#ifdef RP2350
        // Update RP2350 hardware acceleration statistics
        stats.hw_accel_reports_processed++;
#endif
    } else {
        if (process_mouse_report_internal(report)) {
            stats.mouse_reports_received++;
            
#ifdef RP2350
            // Update RP2350 software fallback statistics
            stats.sw_fallback_reports_processed++;
#endif
        }
    }
#else
    if (process_mouse_report_internal(report)) {
        stats.mouse_reports_forwarded++;
    } else {
        stats.forwarding_errors++;
    }
#endif
    
    // Process next report if available
    if (!is_mouse_buffer_empty()) {
        dequeue_and_process_mouse_report();
    }
}

bool find_key_in_report(const hid_keyboard_report_t* report, uint8_t keycode)
{
    if (report == NULL) {
        return false;
    }
    
    for (uint8_t i = 0; i < HID_KEYBOARD_KEYCODE_COUNT; i++) {
        if (report->keycode[i] == keycode) {
            return true;
        }
    }
    
    return false;
}

__attribute__((unused)) static bool process_keyboard_report_internal(const hid_keyboard_report_t* report)
{
    if (report == NULL) {
        return false;
    }
    
    // Fast path: skip ready check for maximum performance
    // TinyUSB will handle the queuing internally
    bool success = tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
    return success;
}

// Hardware acceleration implementations are now in rp2350_hw_accel.c

__attribute__((unused)) static bool process_mouse_report_internal(const hid_mouse_report_t* report)
{
    if (report == NULL) {
        return false;
    }
    
    // Skip coordinate clamping for performance - trust the input device
    // Most modern mice send valid coordinates anyway
    
    // Fast button validation using bitwise AND
    uint8_t valid_buttons = report->buttons & 0x07; // Keep only first 3 bits (L/R/M buttons)
    
    // Fast path: skip ready check for maximum performance
    bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, valid_buttons, report->x, report->y, report->wheel, 0);
    return success;
}