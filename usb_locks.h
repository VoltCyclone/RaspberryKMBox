/*
 * Hurricane PIOKMbox Firmware
 *
 * This header declares global lock variables used for thread-safe access
 * to shared resources across the codebase.
 */

#ifndef USB_LOCKS_H
#define USB_LOCKS_H

#include "hardware/sync.h"

// Global lock for USB state access
extern spin_lock_t *usb_state_lock;

// Initialize locks
void init_usb_locks(void);

#endif // USB_LOCKS_H