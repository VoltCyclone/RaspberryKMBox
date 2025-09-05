// state_management.h - Centralized state structures

#ifndef STATE_MANAGEMENT_H
#define STATE_MANAGEMENT_H

#include <stdint.h>
#include <stdbool.h>

//--------------------------------------------------------------------+
// Core System State
//--------------------------------------------------------------------+

typedef struct {
    // Main loop timing state  
    uint32_t last_watchdog_time;
    uint32_t last_visual_time;
    uint32_t last_error_check_time;
    uint32_t last_button_time;
    
    // Reporting timers
    uint32_t watchdog_status_timer;
    
    // Button state
    uint32_t last_button_press_time;
    bool button_pressed_last;
    bool usb_reset_cooldown;
    uint32_t usb_reset_cooldown_start;
    
    // System status flags
    bool device_initialized;
    bool host_initialized;
    bool watchdog_active;
    
} system_state_t;

//--------------------------------------------------------------------+
// State Management Functions
//--------------------------------------------------------------------+

// Initialize system state
void system_state_init(system_state_t* state);

// Get current system state (singleton pattern)
system_state_t* get_system_state(void);

// State update helpers
void system_state_update_timing(system_state_t* state, uint32_t current_time);
bool system_state_should_run_task(const system_state_t* state, uint32_t current_time, 
                                  uint32_t last_run_time, uint32_t interval_ms);

// Performance optimization: batch timer updates
void system_state_batch_update_timers(system_state_t* state, uint32_t current_time,
                                     uint8_t update_flags);

#endif // STATE_MANAGEMENT_H
