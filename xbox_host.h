/*
 * Xbox Controller Host Driver
 *
 * TinyUSB custom host class driver for Xbox GIP controllers.
 * Runs on Core 1, detects Xbox controllers by interface class,
 * proxies GIP traffic, and populates the shared gamepad accumulator.
 */

#ifndef XBOX_HOST_H
#define XBOX_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include "xbox_gip.h"

// Initialize Xbox host driver shared state (call before tuh_init)
void xbox_host_init(void);

// Check if Xbox mode just ended and re-enumeration is needed (call from Core 0)
bool xbox_host_check_and_clear_reenum(void);

// Drain device_to_host_queue and send to controller (call from Core 1 loop)
void xbox_host_task(void);

// Get current auth state
xbox_auth_state_t xbox_host_get_auth_state(void);

// Get the gamepad state (for reading physical input on Core 0)
xbox_gamepad_state_t *xbox_host_get_gamepad(void);

// Get the host-to-device queue (controller responses -> console)
gip_packet_queue_t *xbox_host_get_h2d_queue(void);

// Get the device-to-host queue (console commands -> controller)
gip_packet_queue_t *xbox_host_get_d2h_queue(void);

#endif // XBOX_HOST_H
