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

void process_kbd_report(const hid_keyboard_report_t* report)
{
    if (report == NULL) {
        return; // Fast fail without printf for performance
    }
    
    // Previous report tracking removed - not currently used
    
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
#endif
    
    // Previous report tracking removed - not currently used
}

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
        stats.mouse_reports_received++;
    }
#endif
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

static bool process_keyboard_report_internal(const hid_keyboard_report_t* report)
{
    if (report == NULL) {
        return false;
    }
    
    // Fast path: skip ready check for maximum performance
    // TinyUSB will handle the queuing internally
    bool success = tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
    if (success) {
        stats.keyboard_reports_forwarded++;
        // Skip error counter reset for performance
        return true;
    } else {
        stats.forwarding_errors++;
        return false;
    }
}

// Hardware acceleration implementations are now in rp2350_hw_accel.c

static bool process_mouse_report_internal(const hid_mouse_report_t* report)
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
    if (success) {
        stats.mouse_reports_forwarded++;
        return true;
    } else {
        stats.forwarding_errors++;
        return false;
    }
}