/*
 * KMBox Commands Library
 * Handles serial command parsing and custom HID report generation
 */

#ifndef KMBOX_COMMANDS_H
#define KMBOX_COMMANDS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//--------------------------------------------------------------------+
// Button Definitions
//--------------------------------------------------------------------+

typedef enum {
    KMBOX_BUTTON_LEFT = 0,
    KMBOX_BUTTON_RIGHT,
    KMBOX_BUTTON_MIDDLE,
    KMBOX_BUTTON_SIDE1,
    KMBOX_BUTTON_SIDE2,
    KMBOX_BUTTON_COUNT
} kmbox_button_t;

//--------------------------------------------------------------------+
// Button State Management
//--------------------------------------------------------------------+

typedef struct {
    bool is_pressed;
    bool is_forced;  // True if button state is forced by command
    uint32_t release_time;  // Time when forced release should end (0 if not active)
    bool is_clicking;  // True if button is in a click sequence
    uint32_t click_release_start;  // Time when click press ends and release starts
    uint32_t click_end_time;  // Time when entire click sequence ends
    bool is_locked;  // True if button is locked (physical input masked from output)
} button_state_t;

typedef struct {
    button_state_t buttons[KMBOX_BUTTON_COUNT];
    uint8_t physical_buttons;  // Actual physical button states
    uint32_t last_update_time;
    bool button_callback_enabled;  // True if button state change callback is enabled
    uint8_t last_button_state;     // Last reported button state for callback
    
    // Mouse movement state
    int16_t mouse_x_accumulator;  // Accumulated X movement
    int16_t mouse_y_accumulator;  // Accumulated Y movement
    int8_t wheel_accumulator;      // Accumulated wheel movement
    
    // Axis lock states
    bool lock_mx;  // Lock X axis (left/right movement)
    bool lock_my;  // Lock Y axis (up/down movement)
} kmbox_state_t;

//--------------------------------------------------------------------+
// Command Parser State
//--------------------------------------------------------------------+

#define KMBOX_CMD_BUFFER_SIZE 64

typedef struct {
    char buffer[KMBOX_CMD_BUFFER_SIZE];
    uint8_t buffer_pos;
    bool in_command;
    bool skip_next_terminator;  // Skip next terminator if it's part of \r\n
    char last_terminator;       // Track last terminator seen ('\r' or '\n')
    char command_terminator[3]; // Store the line terminator(s) used for current command
    uint8_t terminator_len;     // Length of the terminator (1 for \n or \r, 2 for \r\n)
} kmbox_parser_t;

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

// Initialize the kmbox commands module
void kmbox_commands_init(void);

// Process incoming serial data (call this with each received character)
void kmbox_process_serial_char(char c, uint32_t current_time_ms);

// Process a complete command line (without trailing terminator). The caller
// should pass the line contents (len bytes), the terminator bytes (pointer)
// and terminator length (1 or 2). This allows callers to hand over full
// lines from DMA/ring-buffer with a single call instead of per-byte calls.
void kmbox_process_serial_line(const char *line, size_t len, const char *terminator, uint8_t term_len, uint32_t current_time_ms);

// Update button states and handle timing (call this periodically)
void kmbox_update_states(uint32_t current_time_ms);

// Get the current mouse report based on button states
void kmbox_get_mouse_report(uint8_t* buttons, int8_t* x, int8_t* y, int8_t* wheel, int8_t* pan);

// Add mouse movement
void kmbox_add_mouse_movement(int16_t x, int16_t y);

// Add wheel movement
void kmbox_add_wheel_movement(int8_t wheel);

// Set axis lock state
void kmbox_set_axis_lock(bool lock_x, bool lock_y);

// Get axis lock states
bool kmbox_get_lock_mx(void);
bool kmbox_get_lock_my(void);

// Check if any button is currently forced
bool kmbox_has_forced_buttons(void);

// Get button name string for debugging
const char* kmbox_get_button_name(kmbox_button_t button);

// Update physical button states (call this with actual hardware button states)
void kmbox_update_physical_buttons(uint8_t physical_buttons);

#endif // KMBOX_COMMANDS_H