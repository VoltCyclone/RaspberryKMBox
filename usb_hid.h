/*
* Hurricane PIOKMbox Firmware
*/

#ifndef USB_HID_H
#define USB_HID_H

#include <stdint.h>
#include <stdbool.h>
#include "tusb.h"
#include "class/hid/hid.h"
#include "class/hid/hid_device.h"
#include "class/hid/hid_host.h"
#include "kmbox_serial_handler.h"

//--------------------------------------------------------------------+
// HID REPORT DEFINITIONS
//--------------------------------------------------------------------+

// HID Report IDs for device mode
enum {
  REPORT_ID_KEYBOARD = 1,
  REPORT_ID_MOUSE,
  REPORT_ID_CONSUMER_CONTROL,
  REPORT_ID_COUNT
};

//--------------------------------------------------------------------+
// FUNCTION PROTOTYPES
//--------------------------------------------------------------------+

// Device mode functions
void hid_device_task(void);
void send_hid_report(uint8_t report_id);

// Host mode functions
void hid_host_task(void);

// Report processing functions
void process_kbd_report(hid_keyboard_report_t const *report);
void process_mouse_report(hid_mouse_report_t const *report);

// Utility functions
bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode);

// State management
bool usb_hid_init(void);
bool usb_host_enable_power(void);
bool get_caps_lock_state(void);
bool is_mouse_connected(void);
bool is_keyboard_connected(void);

// USB stack initialization tracking
void usb_device_mark_initialized(void);
void usb_host_mark_initialized(void);

// USB stack reset functions
bool usb_device_stack_reset(void);
bool usb_host_stack_reset(void);
bool usb_stacks_reset(void);
void usb_stack_error_check(void);

//--------------------------------------------------------------------+
// USB DESCRIPTORS
//--------------------------------------------------------------------+

// HID report descriptor for device mode
extern uint8_t const desc_hid_report[];

// Device descriptor
extern tusb_desc_device_t const desc_device;

// Configuration descriptor
extern uint8_t const desc_configuration[];

// String descriptors
extern char const* string_desc_arr[];

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

// HID host callbacks
extern uint16_t attached_vid;
extern uint16_t attached_pid;

// Function to get the VID of the attached device
uint16_t get_attached_vid(void);

// Function to get the PID of the attached device
uint16_t get_attached_pid(void);

// Function to set the VID and PID of the attached device
void set_attached_device_vid_pid(uint16_t vid, uint16_t pid);

// Functions to get attached device string descriptors
const char* get_attached_manufacturer(void);
const char* get_attached_product(void);

// Function to get dynamic serial string
const char* get_dynamic_serial_string(void);

// Function to force USB re-enumeration
void force_usb_reenumeration(void);

// TinyUSB Host callbacks
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len);
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance);
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);

// Descriptor callbacks
uint8_t const * tud_descriptor_device_cb(void);
uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance);
uint8_t const * tud_descriptor_configuration_cb(uint8_t index);
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);

#endif // USB_HID_H