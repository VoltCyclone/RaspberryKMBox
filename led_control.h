/*
 * LED Control Module for PIOKMbox
 * 
 * This module handles:
 * - Neopixel status display
 * - LED blinking functionality
 * - System status indication
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "defines.h"

//--------------------------------------------------------------------+
// NEOPIXEL DEFINITIONS
//--------------------------------------------------------------------+

// All color definitions are now in defines.h

// System status enumeration
typedef enum {
    STATUS_BOOTING,
    STATUS_USB_DEVICE_ONLY,
    STATUS_USB_HOST_ONLY,
    STATUS_BOTH_ACTIVE,
    STATUS_MOUSE_CONNECTED,
    STATUS_KEYBOARD_CONNECTED,
    STATUS_BOTH_HID_CONNECTED,
    STATUS_ERROR,
    STATUS_SUSPENDED,
    STATUS_USB_RESET_PENDING,
    STATUS_USB_RESET_SUCCESS,
    STATUS_USB_RESET_FAILED,
    // CP2110 Serial Connection Status
    STATUS_CP2110_WAITING,
    STATUS_CP2110_CONNECTING,
    STATUS_CP2110_CONNECTED,
    STATUS_CP2110_ACTIVE,
    STATUS_CP2110_DISCONNECTED
} system_status_t;

// CP2110 Connection state (for serial handler)
typedef enum {
    CP2110_STATE_WAITING,       // Waiting for initial connection
    CP2110_STATE_CONNECTING,    // Handshake in progress
    CP2110_STATE_CONNECTED,     // Connected and ready
    CP2110_STATE_ACTIVE,        // Actively receiving commands
    CP2110_STATE_DISCONNECTED   // Connection lost (timeout)
} cp2110_connection_state_t;

//--------------------------------------------------------------------+
// FUNCTION PROTOTYPES
//--------------------------------------------------------------------+

// LED blinking functions
void led_blinking_task(void);
void led_set_blink_interval(uint32_t interval_ms);

// Neopixel functions
void neopixel_init(void);
void neopixel_enable_power(void);
void neopixel_set_color(uint32_t color);
void neopixel_set_color_with_brightness(uint32_t color, float brightness);
// Integer brightness variant (0-255) for fixed-point code paths
void neopixel_set_color_with_brightness_u8(uint32_t color, uint8_t brightness);
void neopixel_update_status(void);
void neopixel_status_task(void);
void neopixel_trigger_activity_flash(void);
void neopixel_trigger_mouse_activity(void);
void neopixel_trigger_keyboard_activity(void);
void neopixel_trigger_caps_lock_flash(void);
void neopixel_trigger_usb_connection_flash(void);
void neopixel_trigger_usb_disconnection_flash(void);
void neopixel_trigger_usb_reset_pending(void);
void neopixel_trigger_usb_reset_success(void);
void neopixel_trigger_usb_reset_failed(void);
void neopixel_set_status_override(system_status_t status);
void neopixel_clear_status_override(void);
void neopixel_trigger_rainbow_effect(void);
// Advance the rainbow based on movement delta (x,y). Call from HID path when movement observed.
void neopixel_rainbow_on_movement(int16_t dx, int16_t dy);

// Utility functions
uint32_t neopixel_rgb_to_grb(uint32_t rgb);
uint32_t neopixel_apply_brightness(uint32_t color, float brightness);
uint32_t neopixel_apply_brightness_u8(uint32_t color, uint8_t brightness);
void neopixel_breathing_effect(void);
// Flush any queued LED frames to the PIO FIFO if space is available
void neopixel_flush_queue(void);

#endif // LED_CONTROL_H