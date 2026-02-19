/*
 * Xbox Controller Device Class
 *
 * TinyUSB custom device class that presents as an Xbox controller
 * to the connected console/PC. Handles GIP announce sequence,
 * input report generation with injection merge, and auth proxy.
 * Runs on Core 0.
 */

#ifndef XBOX_DEVICE_H
#define XBOX_DEVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "xbox_gip.h"

// Initialize Xbox device state (call before tud_init)
void xbox_device_init(void);

// Main device task: drain accumulator, merge injection, send reports
// Call from Core 0 main loop when g_xbox_mode is true
void xbox_device_task(void);

// Get Xbox device descriptor (called from tud_descriptor_device_cb)
uint8_t const *xbox_get_device_descriptor(void);

// Get Xbox configuration descriptor (called from tud_descriptor_configuration_cb)
uint8_t const *xbox_get_config_descriptor(void);

// Get Xbox string descriptor (called from tud_descriptor_string_cb)
uint16_t const *xbox_get_string_descriptor(uint8_t index, uint16_t langid);

// Update gamepad injection state from serial commands
void xbox_inject_buttons(uint16_t buttons, uint16_t trigger_left, uint16_t trigger_right);
void xbox_inject_stick_left(int16_t x, int16_t y);
void xbox_inject_stick_right(int16_t x, int16_t y);
void xbox_inject_clear(void);

#endif // XBOX_DEVICE_H
