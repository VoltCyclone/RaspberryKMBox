/*
 * RP2350 Enhanced TinyUSB Host Task Implementation
 * 
 * This header provides declarations for the enhanced TinyUSB host task
 * implementation that integrates RP2350-specific hardware acceleration.
 */

#ifndef RP2350_TUH_TASK_H
#define RP2350_TUH_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef RP2350
#include "rp2350_hw_accel.h"

// Function declarations
bool rp2350_tuh_task_init(void);
void rp2350_enhanced_tuh_task(void);
bool rp2350_tuh_task_hw_accel_enabled(void);
void rp2350_tuh_task_get_stats(hw_accel_stats_t* stats);
bool rp2350_patch_tuh_task(void);

#endif // RP2350

#endif // RP2350_TUH_TASK_H