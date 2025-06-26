/*
 * Hurricane PIOKMbox Firmware
 */

#ifndef USB_HID_STATS_H
#define USB_HID_STATS_H

#include "usb_hid_types.h"
#include <stdint.h>
#include "pico/critical_section.h"

// Critical section locks for thread safety
extern critical_section_t usb_state_lock;
extern critical_section_t stats_lock;

// Performance monitoring functions
void print_hid_connection_status(void);
void get_hid_stats(hid_stats_t* stats_out);
void reset_hid_stats(void);

#if defined(RP2350)
// RP2350 hardware acceleration statistics functions
void track_hw_processing_latency(uint64_t start_time);
void track_sw_processing_latency(uint64_t start_time);
void increment_hw_accel_reports(void);
void increment_sw_fallback_reports(void);
void increment_hw_accel_errors(void);
float calculate_hw_accel_success_rate(void);
float calculate_avg_hw_processing_time_us(void);
float calculate_avg_sw_processing_time_us(void);
void print_hw_accel_stats(void);
#endif

#endif // USB_HID_STATS_H