/*
 * Hurricane PIOKMbox Firmware
 */

#ifndef USB_HID_CALLBACKS_H
#define USB_HID_CALLBACKS_H

#include <stdint.h>
#include "tusb.h"

//--------------------------------------------------------------------+
// CALLBACK FUNCTION PROTOTYPES
//--------------------------------------------------------------------+

// Device callbacks
void tud_mount_cb(void);
void tud_umount_cb(void);
void tud_suspend_cb(bool remote_wakeup_en);
void tud_resume_cb(void);

// HID device callbacks
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len);

// Host callbacks
void tuh_mount_cb(uint8_t dev_addr);
void tuh_umount_cb(uint8_t dev_addr);

// HID host callbacks
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len);
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance);
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);

#endif // USB_HID_CALLBACKS_H