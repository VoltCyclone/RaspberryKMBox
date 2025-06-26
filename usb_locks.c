/*
 * Hurricane PIOKMbox Firmware
 *
 * This file defines global lock variables used for thread-safe access
 * to shared resources across the codebase.
 */

#include "pico/stdlib.h"
#include "hardware/sync.h"

// Global lock for USB state access
spin_lock_t *usb_state_lock;

// Initialize locks
void init_usb_locks(void) {
    // Claim unused spinlock
    uint lock_num = spin_lock_claim_unused(true);
    usb_state_lock = spin_lock_init(lock_num);
}