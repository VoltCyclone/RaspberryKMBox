/*
 * Hurricane PIOKMbox Firmware
 */

#include "usb_hid_stats.h"
#include "defines.h"
#include <stdio.h>
#include <string.h>
#include "pico/time.h"

#ifdef RP2350
#include "rp2350_hw_accel.h"
#endif

// External declarations for variables defined in other modules
extern device_connection_state_t connection_state;
extern performance_stats_t stats;
extern tusb_desc_device_t desc_device;

// Performance optimization: reduce debug output
static uint32_t debug_counter = 0;

// Print HID device connection status with VID/PID information
void print_hid_connection_status(void)
{
    bool mouse_connected, keyboard_connected;
    uint16_t mouse_vid, mouse_pid, keyboard_vid, keyboard_pid;
    char manufacturer[USB_STRING_BUFFER_SIZE];
    char product[USB_STRING_BUFFER_SIZE];
    char serial[USB_STRING_BUFFER_SIZE];
    
    // Safely access connection state with critical section
    critical_section_enter_blocking(&usb_state_lock);
    mouse_connected = connection_state.mouse_connected;
    keyboard_connected = connection_state.keyboard_connected;
    mouse_vid = connection_state.mouse_vid;
    mouse_pid = connection_state.mouse_pid;
    keyboard_vid = connection_state.keyboard_vid;
    keyboard_pid = connection_state.keyboard_pid;
    
    // Copy string descriptors to local buffers
    strncpy(manufacturer, connection_state.manufacturer, USB_STRING_BUFFER_SIZE - 1);
    strncpy(product, connection_state.product, USB_STRING_BUFFER_SIZE - 1);
    strncpy(serial, connection_state.serial, USB_STRING_BUFFER_SIZE - 1);
    critical_section_exit(&usb_state_lock);
    
    // Ensure null termination
    manufacturer[USB_STRING_BUFFER_SIZE - 1] = '\0';
    product[USB_STRING_BUFFER_SIZE - 1] = '\0';
    serial[USB_STRING_BUFFER_SIZE - 1] = '\0';
    
    if (mouse_connected) {
        printf("Mouse: Connected (VID:0x%04X PID:0x%04X)\n", mouse_vid, mouse_pid);
    } else {
        printf("Mouse: Not connected\n");
    }
    
    if (keyboard_connected) {
        printf("Keyboard: Connected (VID:0x%04X PID:0x%04X)\n", keyboard_vid, keyboard_pid);
    } else {
        printf("Keyboard: Not connected\n");
    }
    
    // Show current device descriptor VID/PID
    printf("Current Device Descriptor: VID:0x%04X PID:0x%04X\n",
           desc_device.idVendor, desc_device.idProduct);
    
    // Show current string descriptors
    printf("Current String Descriptors:\n");
    printf("  Manufacturer: %s\n", manufacturer);
    printf("  Product: %s\n", product);
    printf("  Serial: %s\n", serial);
}

void get_hid_stats(hid_stats_t* stats_out)
{
    if (stats_out == NULL) {
        return;
    }
    
    // Thread-safe copy of statistics using critical section
    critical_section_enter_blocking(&stats_lock);
    *stats_out = stats;  // Copy the entire structure
    critical_section_exit(&stats_lock);
    
#if defined(RP2350)
    // Calculate derived statistics
    stats_out->hw_accel_success_rate = calculate_hw_accel_success_rate();
    stats_out->hw_avg_processing_time_us = calculate_avg_hw_processing_time_us();
    stats_out->sw_avg_processing_time_us = calculate_avg_sw_processing_time_us();
    
    // Get additional hardware acceleration statistics
    if (hw_accel_is_enabled()) {
        hw_accel_stats_t hw_stats;
        hw_accel_get_stats(&hw_stats);
        
        // Update hardware acceleration statistics
        stats_out->hw_accel_reports_processed += hw_stats.dma_transfers_completed + hw_stats.pio_operations_completed;
        stats_out->hw_accel_errors += hw_stats.dma_transfer_errors + hw_stats.pio_operation_errors +
                                     hw_stats.fifo_overflows + hw_stats.fifo_underflows;
        
        // Update processing time statistics
        if (hw_stats.processing_count > 0) {
            stats_out->hw_processing_time_us += hw_stats.processing_time_us;
            stats_out->hw_processing_count += hw_stats.processing_count;
        }
    }
#endif
    
    // Print connection status with VID/PID information
    print_hid_connection_status();
}

void reset_hid_stats(void)
{
    critical_section_enter_blocking(&stats_lock);
    memset(&stats, 0, sizeof(stats));
    debug_counter = 0;
    critical_section_exit(&stats_lock);
}

#if defined(RP2350)
// RP2350 hardware acceleration statistics functions

void track_hw_processing_latency(uint64_t start_time)
{
    uint64_t end_time = time_us_64();
    uint64_t latency = end_time - start_time;
    
    critical_section_enter_blocking(&stats_lock);
    stats.hw_processing_time_us += latency;
    stats.hw_processing_count++;
    critical_section_exit(&stats_lock);
}

void track_sw_processing_latency(uint64_t start_time)
{
    uint64_t end_time = time_us_64();
    uint64_t latency = end_time - start_time;
    
    critical_section_enter_blocking(&stats_lock);
    stats.sw_processing_time_us += latency;
    stats.sw_processing_count++;
    critical_section_exit(&stats_lock);
}

void increment_hw_accel_reports(void)
{
    critical_section_enter_blocking(&stats_lock);
    stats.hw_accel_reports_processed++;
    critical_section_exit(&stats_lock);
}

void increment_sw_fallback_reports(void)
{
    critical_section_enter_blocking(&stats_lock);
    stats.sw_fallback_reports_processed++;
    critical_section_exit(&stats_lock);
}

void increment_hw_accel_errors(void)
{
    critical_section_enter_blocking(&stats_lock);
    stats.hw_accel_errors++;
    critical_section_exit(&stats_lock);
}

float calculate_hw_accel_success_rate(void)
{
    float result;
    uint32_t hw_reports, sw_reports;
    
    critical_section_enter_blocking(&stats_lock);
    hw_reports = stats.hw_accel_reports_processed;
    sw_reports = stats.sw_fallback_reports_processed;
    critical_section_exit(&stats_lock);
    
    uint32_t total_reports = hw_reports + sw_reports;
    if (total_reports == 0) {
        return 0.0f;
    }
    return (float)hw_reports / (float)total_reports * 100.0f;
}

float calculate_avg_hw_processing_time_us(void)
{
    uint64_t processing_time;
    uint32_t count;
    
    critical_section_enter_blocking(&stats_lock);
    processing_time = stats.hw_processing_time_us;
    count = stats.hw_processing_count;
    critical_section_exit(&stats_lock);
    
    if (count == 0) {
        return 0.0f;
    }
    return (float)processing_time / (float)count;
}

float calculate_avg_sw_processing_time_us(void)
{
    uint64_t processing_time;
    uint32_t count;
    
    critical_section_enter_blocking(&stats_lock);
    processing_time = stats.sw_processing_time_us;
    count = stats.sw_processing_count;
    critical_section_exit(&stats_lock);
    
    if (count == 0) {
        return 0.0f;
    }
    return (float)processing_time / (float)count;
}

void print_hw_accel_stats(void)
{
    uint32_t hw_reports, sw_reports, hw_errors;
    
    critical_section_enter_blocking(&stats_lock);
    hw_reports = stats.hw_accel_reports_processed;
    sw_reports = stats.sw_fallback_reports_processed;
    hw_errors = stats.hw_accel_errors;
    critical_section_exit(&stats_lock);
    
    printf("RP2350 Hardware Acceleration Statistics:\n");
    printf("  HW Accelerated Reports: %lu\n", hw_reports);
    printf("  SW Fallback Reports: %lu\n", sw_reports);
    printf("  HW Acceleration Errors: %lu\n", hw_errors);
    printf("  HW Acceleration Success Rate: %.2f%%\n", calculate_hw_accel_success_rate());
    
    float hw_avg_time = calculate_avg_hw_processing_time_us();
    float sw_avg_time = calculate_avg_sw_processing_time_us();
    
    printf("  Avg HW Processing Time: %.2f µs\n", hw_avg_time);
    printf("  Avg SW Processing Time: %.2f µs\n", sw_avg_time);
    
    if (hw_avg_time > 0 && sw_avg_time > 0) {
        float speedup = sw_avg_time / hw_avg_time;
        printf("  HW Acceleration Speedup: %.2fx\n", speedup);
    }
    
    // Print additional hardware acceleration details if available
    if (hw_accel_is_enabled()) {
        hw_accel_stats_t hw_stats;
        hw_accel_get_stats(&hw_stats);
        
        printf("\nHardware Acceleration Details:\n");
        printf("  DMA Transfers Completed: %lu\n", hw_stats.dma_transfers_completed);
        printf("  DMA Transfer Errors: %lu\n", hw_stats.dma_transfer_errors);
        printf("  PIO Operations Completed: %lu\n", hw_stats.pio_operations_completed);
        printf("  PIO Operation Errors: %lu\n", hw_stats.pio_operation_errors);
        printf("  FIFO Overflows: %lu\n", hw_stats.fifo_overflows);
        printf("  FIFO Underflows: %lu\n", hw_stats.fifo_underflows);
    } else {
        printf("\nHardware Acceleration: DISABLED\n");
    }
}
#endif