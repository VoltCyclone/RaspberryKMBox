/*
 * Hurricane PIOKMbox Firmware
 */

#ifndef USB_HID_INIT_H
#define USB_HID_INIT_H

#include <stdbool.h>
#include "usb_hid_types.h"

// Initialization functions
bool usb_hid_init(void);
bool usb_host_enable_power(void);
bool usb_hid_dma_init(void);
void init_synchronization(void);

// USB stack initialization tracking
void usb_device_mark_initialized(void);
void usb_host_mark_initialized(void);

// USB stack reset functions
bool usb_device_stack_reset(void);
bool usb_host_stack_reset(void);
bool usb_stacks_reset(void);
void usb_stack_error_check(void);

#endif // USB_HID_INIT_H