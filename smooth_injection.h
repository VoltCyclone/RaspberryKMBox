/*
 * Smooth Injection System
 * 
 * Provides ultra-smooth mouse movement injection that blends naturally
 * with physical mouse passthrough. Uses sub-pixel precision and 
 * velocity-aware spreading to eliminate jitter and jumps.
 *
 * Key Features:
 * - Sub-pixel accumulation (16.16 fixed-point precision)
 * - Velocity matching - injected moves blend with current mouse speed
 * - Temporal spreading - large moves spread across multiple HID frames
 * - Per-frame rate limiting - prevents single-frame jumps
 * - Movement queue for smooth async injection
 */

#ifndef SMOOTH_INJECTION_H
#define SMOOTH_INJECTION_H

#include <stdint.h>
#include <stdbool.h>

//--------------------------------------------------------------------+
// Configuration
//--------------------------------------------------------------------+

// Maximum movement per HID frame (prevents jarring jumps)
// At 125Hz (8ms), 16 pixels/frame = ~2000 pixels/sec max smooth speed
#define SMOOTH_MAX_PER_FRAME        16

// Movement queue size (number of pending inject operations)
#define SMOOTH_QUEUE_SIZE           64

// Velocity tracking window (in frames, ~8ms each)
#define SMOOTH_VELOCITY_WINDOW      8

// Fixed-point precision (16.16 format)
#define SMOOTH_FP_SHIFT             16
#define SMOOTH_FP_ONE               (1 << SMOOTH_FP_SHIFT)
#define SMOOTH_FP_HALF              (1 << (SMOOTH_FP_SHIFT - 1))

//--------------------------------------------------------------------+
// Injection Modes
//--------------------------------------------------------------------+

typedef enum {
    // Immediate: Add directly to accumulator (legacy behavior)
    INJECT_MODE_IMMEDIATE = 0,
    
    // Smooth: Spread movement across frames to match max per-frame rate
    INJECT_MODE_SMOOTH,
    
    // Velocity-matched: Blend with current mouse velocity
    INJECT_MODE_VELOCITY_MATCHED,
    
    // Micro: For tiny sub-pixel adjustments (anti-recoil, aim correction)
    INJECT_MODE_MICRO,
} inject_mode_t;

//--------------------------------------------------------------------+
// Easing Modes for Natural Movement
//--------------------------------------------------------------------+

typedef enum {
    EASING_LINEAR = 0,      // No easing (constant velocity)
    EASING_EASE_IN_OUT,     // Slow start, fast middle, slow end (natural)
    EASING_EASE_OUT,        // Quick start, slow end (corrections)
} easing_mode_t;

//--------------------------------------------------------------------+
// Humanization Modes
//--------------------------------------------------------------------+

typedef enum {
    HUMANIZATION_OFF = 0,       // No humanization — raw pass-through
    HUMANIZATION_MICRO,         // Micro-noise only (for pre-humanized input)
    HUMANIZATION_FULL,          // Full humanization (for raw/robotic input)
    HUMANIZATION_MODE_COUNT
} humanization_mode_t;

//--------------------------------------------------------------------+
// Movement Queue Entry
//--------------------------------------------------------------------+

typedef struct {
    int32_t x_fp;           // X movement in fixed-point (16.16)
    int32_t y_fp;           // Y movement in fixed-point (16.16)
    int32_t x_remaining_fp; // Remaining X to inject
    int32_t y_remaining_fp; // Remaining Y to inject
    uint8_t frames_left;    // Frames remaining for this movement
    uint8_t total_frames;   // Total frames for this movement (for easing calc)
    inject_mode_t mode;     // Injection mode
    easing_mode_t easing;   // Easing curve to apply
    bool active;            // Is this entry in use?
    uint8_t onset_delay;    // Frames to wait before starting delivery (onset jitter)
    
    // Overshoot simulation
    bool will_overshoot;    // Will this movement overshoot?
    int32_t overshoot_x_fp; // Overshoot amount X
    int32_t overshoot_y_fp; // Overshoot amount Y
    
    // O(1) queue free: cached index into g_active_nodes[]
    int8_t active_node_idx; // -1 = not linked
} smooth_queue_entry_t;

//--------------------------------------------------------------------+
// Velocity Tracker
//--------------------------------------------------------------------+

typedef struct {
    int16_t x_history[SMOOTH_VELOCITY_WINDOW];
    int16_t y_history[SMOOTH_VELOCITY_WINDOW];
    uint8_t history_index;
    int32_t avg_velocity_x_fp;  // Average X velocity (fixed-point)
    int32_t avg_velocity_y_fp;  // Average Y velocity (fixed-point)
} velocity_tracker_t;

//--------------------------------------------------------------------+
// Smooth Injection State
//--------------------------------------------------------------------+

typedef struct {
    // Sub-pixel accumulators (fixed-point 16.16)
    int32_t x_accumulator_fp;
    int32_t y_accumulator_fp;
    
    // Movement queue
    smooth_queue_entry_t queue[SMOOTH_QUEUE_SIZE];
    uint8_t queue_head;
    uint8_t queue_count;
    
    // Velocity tracking
    velocity_tracker_t velocity;
    
    // Per-frame budget tracking
    int16_t frame_x_used;
    int16_t frame_y_used;
    
    // Statistics
    uint32_t total_injected;
    uint32_t frames_processed;
    uint32_t queue_overflows;
    
    // Configuration
    int16_t max_per_frame;
    bool velocity_matching_enabled;
    
    // Humanization settings
    struct {
        humanization_mode_t mode;      // Current humanization mode
        bool jitter_enabled;           // Enable micro-jitter (hand tremor)
        int32_t jitter_amount_fp;      // Jitter magnitude (fixed-point)
        uint8_t overshoot_chance;      // % chance of overshoot (0-100)
        int32_t overshoot_max_fp;      // Max overshoot distance (fixed-point)
        int32_t vel_slow_threshold_fp; // Velocity threshold for "slow" movement
        int32_t vel_fast_threshold_fp; // Velocity threshold for "fast" movement
        int32_t delivery_error_fp;     // Per-frame delivery error magnitude (sensor noise sim)
        int32_t accum_clamp_fp;        // Max accumulator magnitude (mode-dependent)
        uint8_t onset_jitter_min;      // Min onset delay frames
        uint8_t onset_jitter_max;      // Max onset delay frames
    } humanization;
} smooth_injection_state_t;

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

/**
 * Initialize the smooth injection system
 */
void smooth_injection_init(void);

/**
 * Queue a movement for smooth injection
 * 
 * @param x X movement (positive = right)
 * @param y Y movement (positive = down)
 * @param mode Injection mode (IMMEDIATE, SMOOTH, VELOCITY_MATCHED, MICRO)
 * @return true if queued successfully, false if queue full
 */
bool smooth_inject_movement(int16_t x, int16_t y, inject_mode_t mode);

/**
 * Queue a movement with sub-pixel precision (fixed-point)
 * 
 * @param x_fp X movement in 16.16 fixed-point
 * @param y_fp Y movement in 16.16 fixed-point
 * @param mode Injection mode
 * @return true if queued successfully
 */
bool smooth_inject_movement_fp(int32_t x_fp, int32_t y_fp, inject_mode_t mode);

/**
 * Record physical mouse movement (for velocity tracking)
 * Call this when physical mouse reports are received
 * 
 * @param x Physical X movement
 * @param y Physical Y movement
 */
void smooth_record_physical_movement(int16_t x, int16_t y);

/**
 * Process one frame of smooth injection
 * Call this during each HID report cycle (typically ~125Hz)
 * 
 * @param out_x Output X movement for this frame
 * @param out_y Output Y movement for this frame
 */
void smooth_process_frame(int8_t *out_x, int8_t *out_y);

/**
 * Get current average velocity (for velocity-matched injection)
 * 
 * @param vel_x Output average X velocity (pixels per frame, fixed-point)
 * @param vel_y Output average Y velocity (pixels per frame, fixed-point)
 */
void smooth_get_velocity(int32_t *vel_x_fp, int32_t *vel_y_fp);

/**
 * Set maximum movement per frame
 * 
 * @param max_per_frame Maximum pixels per frame (default: 16)
 */
void smooth_set_max_per_frame(int16_t max_per_frame);

/**
 * Enable/disable velocity matching
 * 
 * @param enabled True to enable velocity matching
 */
void smooth_set_velocity_matching(bool enabled);

/**
 * Clear all pending injections
 */
void smooth_clear_queue(void);

/**
 * Get statistics
 */
void smooth_get_stats(uint32_t *total_injected, uint32_t *frames_processed, 
                      uint32_t *queue_overflows, uint8_t *queue_count);

/**
 * Check if there are pending movements in the queue
 * 
 * @return true if there are movements to process
 */
bool smooth_has_pending(void);

/**
 * Set humanization mode (affects jitter, easing, overshoot)
 * 
 * @param mode Humanization mode (OFF, LOW, MEDIUM, HIGH)
 */
void smooth_set_humanization_mode(humanization_mode_t mode);

/**
 * Get current humanization mode
 * 
 * @return Current humanization mode
 */
humanization_mode_t smooth_get_humanization_mode(void);

/**
 * Cycle to next humanization mode
 * 
 * @return New humanization mode
 */
humanization_mode_t smooth_cycle_humanization_mode(void);

/**
 * Save current humanization mode to flash memory
 * NOTE: This now schedules a deferred save to avoid multicore flash issues
 */
void smooth_save_humanization_mode(void);

/**
 * Load humanization mode from flash memory
 */
void smooth_load_humanization_mode(void);

/**
 * Process deferred flash saves
 * Call this periodically from main loop (safe to call frequently)
 * Performs the actual flash write when conditions are met
 */
void smooth_process_deferred_save(void);

/**
 * Get current maximum movement per frame setting
 * 
 * @return Maximum pixels per frame
 */
int16_t smooth_get_max_per_frame(void);

/**
 * Get current velocity matching state
 * 
 * @return True if velocity matching is enabled
 */
bool smooth_get_velocity_matching(void);

/**
 * Get current injection mode (for stats/display)
 * 
 * @return Last used injection mode
 */
inject_mode_t smooth_get_inject_mode(void);

/**
 * Get humanization parameters for output-stage tremor application
 * 
 * @param jitter_amount_fp Output pointer for jitter amount (16.16 fixed-point)
 * @param jitter_enabled Output pointer for jitter enabled flag
 */
void smooth_get_humanization_params(int32_t *jitter_amount_fp, bool *jitter_enabled);

#endif // SMOOTH_INJECTION_H
