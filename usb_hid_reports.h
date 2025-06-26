/*
 * Hurricane PIOKMbox Firmware
 */

#ifndef USB_HID_REPORTS_H
#define USB_HID_REPORTS_H

#include <stdbool.h>
#include "usb_hid_types.h"
#include "class/hid/hid.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// Circular buffer sizes (must be power of 2)
#define KBD_BUFFER_SIZE 16
#define MOUSE_BUFFER_SIZE 32

// DMA channel configuration
#define DMA_KBD_CHANNEL 0
#define DMA_MOUSE_CHANNEL 1
#define DMA_IRQ_PRIORITY 0x40  // Medium priority

// Circular buffer structure (word-aligned for optimal DMA performance)
typedef struct __attribute__((aligned(4))) {
    volatile uint32_t read_idx;
    volatile uint32_t write_idx;
    uint32_t size;
    uint32_t mask;
    void* buffer;
} dma_circular_buffer_t;

// Report processing functions
void process_kbd_report(const hid_keyboard_report_t* report);
void process_mouse_report(const hid_mouse_report_t* report);

// DMA initialization and management
void init_hid_dma(void);
void process_queued_reports(void);
bool is_kbd_buffer_empty(void);
bool is_mouse_buffer_empty(void);

// Utility functions
bool find_key_in_report(const hid_keyboard_report_t* report, uint8_t keycode);

#endif // USB_HID_REPORTS_H