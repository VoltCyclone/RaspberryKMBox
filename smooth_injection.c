/*
 * Smooth Injection System Implementation
 * 
 * Provides ultra-smooth mouse movement injection with sub-pixel precision,
 * velocity tracking, and temporal spreading for seamless blending with
 * physical mouse passthrough.
 */

#include "smooth_injection.h"
#include <string.h>

// Global state
static smooth_injection_state_t g_smooth = {0};

//--------------------------------------------------------------------+
// Fixed-Point Math Helpers
//--------------------------------------------------------------------+

static inline int32_t fp_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> SMOOTH_FP_SHIFT);
}

static inline int32_t fp_div(int32_t a, int32_t b) {
    if (b == 0) return 0;
    return (int32_t)(((int64_t)a << SMOOTH_FP_SHIFT) / b);
}

static inline int32_t int_to_fp(int16_t val) {
    return (int32_t)val << SMOOTH_FP_SHIFT;
}

static inline int16_t fp_to_int(int32_t fp_val) {
    // Round to nearest integer
    if (fp_val >= 0) {
        return (int16_t)((fp_val + SMOOTH_FP_HALF) >> SMOOTH_FP_SHIFT);
    } else {
        return (int16_t)((fp_val - SMOOTH_FP_HALF) >> SMOOTH_FP_SHIFT);
    }
}

static inline int8_t clamp_i8(int32_t val) {
    if (val > 127) return 127;
    if (val < -128) return -128;
    return (int8_t)val;
}

//--------------------------------------------------------------------+
// Velocity Tracking
//--------------------------------------------------------------------+

static void velocity_update(int8_t x, int8_t y) {
    velocity_tracker_t *v = &g_smooth.velocity;
    
    // Store in history
    v->x_history[v->history_index] = x;
    v->y_history[v->history_index] = y;
    v->history_index = (v->history_index + 1) % SMOOTH_VELOCITY_WINDOW;
    
    // Calculate average velocity (simple moving average)
    int32_t sum_x = 0, sum_y = 0;
    for (int i = 0; i < SMOOTH_VELOCITY_WINDOW; i++) {
        sum_x += v->x_history[i];
        sum_y += v->y_history[i];
    }
    
    // Store as fixed-point average
    v->avg_velocity_x_fp = int_to_fp(sum_x) / SMOOTH_VELOCITY_WINDOW;
    v->avg_velocity_y_fp = int_to_fp(sum_y) / SMOOTH_VELOCITY_WINDOW;
}

//--------------------------------------------------------------------+
// Queue Management
//--------------------------------------------------------------------+

static smooth_queue_entry_t* queue_alloc(void) {
    if (g_smooth.queue_count >= SMOOTH_QUEUE_SIZE) {
        g_smooth.queue_overflows++;
        return NULL;
    }
    
    // Find an inactive slot
    for (int i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
        if (!g_smooth.queue[i].active) {
            g_smooth.queue_count++;
            return &g_smooth.queue[i];
        }
    }
    
    return NULL;
}

static void queue_free(smooth_queue_entry_t *entry) {
    if (entry && entry->active) {
        entry->active = false;
        if (g_smooth.queue_count > 0) {
            g_smooth.queue_count--;
        }
    }
}

//--------------------------------------------------------------------+
// Public API Implementation
//--------------------------------------------------------------------+

void smooth_injection_init(void) {
    memset(&g_smooth, 0, sizeof(g_smooth));
    g_smooth.max_per_frame = SMOOTH_MAX_PER_FRAME;
    g_smooth.velocity_matching_enabled = true;
}

bool smooth_inject_movement(int16_t x, int16_t y, inject_mode_t mode) {
    return smooth_inject_movement_fp(int_to_fp(x), int_to_fp(y), mode);
}

bool smooth_inject_movement_fp(int32_t x_fp, int32_t y_fp, inject_mode_t mode) {
    // For immediate mode, just add to accumulator
    if (mode == INJECT_MODE_IMMEDIATE) {
        g_smooth.x_accumulator_fp += x_fp;
        g_smooth.y_accumulator_fp += y_fp;
        g_smooth.total_injected++;
        return true;
    }
    
    // For micro mode, add directly to sub-pixel accumulator (no rate limiting)
    if (mode == INJECT_MODE_MICRO) {
        g_smooth.x_accumulator_fp += x_fp;
        g_smooth.y_accumulator_fp += y_fp;
        g_smooth.total_injected++;
        return true;
    }
    
    // For smooth and velocity-matched modes, queue the movement
    smooth_queue_entry_t *entry = queue_alloc();
    if (!entry) {
        // Queue full - fall back to immediate injection
        g_smooth.x_accumulator_fp += x_fp;
        g_smooth.y_accumulator_fp += y_fp;
        return false;
    }
    
    // Calculate how many frames this movement should span
    int32_t abs_x = x_fp >= 0 ? x_fp : -x_fp;
    int32_t abs_y = y_fp >= 0 ? y_fp : -y_fp;
    int32_t max_component = abs_x > abs_y ? abs_x : abs_y;
    int32_t max_per_frame_fp = int_to_fp(g_smooth.max_per_frame);
    
    uint8_t frames = 1;
    if (max_component > max_per_frame_fp) {
        // Calculate frames needed (round up)
        frames = (uint8_t)((max_component + max_per_frame_fp - 1) / max_per_frame_fp);
        if (frames > 255) frames = 255;
        if (frames < 1) frames = 1;
    }
    
    // For velocity-matched mode, adjust based on current velocity
    if (mode == INJECT_MODE_VELOCITY_MATCHED && g_smooth.velocity_matching_enabled) {
        // Get magnitude of current velocity
        int32_t vel_mag = g_smooth.velocity.avg_velocity_x_fp;
        if (vel_mag < 0) vel_mag = -vel_mag;
        int32_t vel_y_abs = g_smooth.velocity.avg_velocity_y_fp;
        if (vel_y_abs < 0) vel_y_abs = -vel_y_abs;
        if (vel_y_abs > vel_mag) vel_mag = vel_y_abs;
        
        // If mouse is moving fast, allow faster injection
        // If mouse is slow/stopped, spread movement over more frames
        if (vel_mag < int_to_fp(2)) {
            // Mouse nearly stationary - be extra smooth
            frames = frames * 2;
            if (frames > 255) frames = 255;
        } else if (vel_mag > int_to_fp(10)) {
            // Mouse moving fast - can inject faster
            frames = (frames + 1) / 2;
            if (frames < 1) frames = 1;
        }
    }
    
    entry->x_fp = x_fp;
    entry->y_fp = y_fp;
    entry->x_remaining_fp = x_fp;
    entry->y_remaining_fp = y_fp;
    entry->frames_left = frames;
    entry->mode = mode;
    entry->active = true;
    
    g_smooth.total_injected++;
    return true;
}

void smooth_record_physical_movement(int8_t x, int8_t y) {
    velocity_update(x, y);
}

void smooth_process_frame(int8_t *out_x, int8_t *out_y) {
    // Reset per-frame budget
    g_smooth.frame_x_used = 0;
    g_smooth.frame_y_used = 0;
    
    // Process queued movements
    int32_t frame_x_fp = 0;
    int32_t frame_y_fp = 0;
    
    for (int i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
        smooth_queue_entry_t *entry = &g_smooth.queue[i];
        if (!entry->active) continue;
        
        if (entry->frames_left > 0) {
            // Calculate movement for this frame
            int32_t dx_fp = entry->x_remaining_fp / entry->frames_left;
            int32_t dy_fp = entry->y_remaining_fp / entry->frames_left;
            
            // Add to frame total
            frame_x_fp += dx_fp;
            frame_y_fp += dy_fp;
            
            // Update remaining
            entry->x_remaining_fp -= dx_fp;
            entry->y_remaining_fp -= dy_fp;
            entry->frames_left--;
            
            // Check if done
            if (entry->frames_left == 0) {
                // Add any remaining fractional movement
                frame_x_fp += entry->x_remaining_fp;
                frame_y_fp += entry->y_remaining_fp;
                queue_free(entry);
            }
        } else {
            queue_free(entry);
        }
    }
    
    // Add sub-pixel accumulator
    frame_x_fp += g_smooth.x_accumulator_fp;
    frame_y_fp += g_smooth.y_accumulator_fp;
    
    // Convert to integer with sub-pixel tracking
    int16_t out_x_int = fp_to_int(frame_x_fp);
    int16_t out_y_int = fp_to_int(frame_y_fp);
    
    // Apply per-frame rate limiting
    if (out_x_int > g_smooth.max_per_frame) {
        out_x_int = g_smooth.max_per_frame;
    } else if (out_x_int < -g_smooth.max_per_frame) {
        out_x_int = -g_smooth.max_per_frame;
    }
    
    if (out_y_int > g_smooth.max_per_frame) {
        out_y_int = g_smooth.max_per_frame;
    } else if (out_y_int < -g_smooth.max_per_frame) {
        out_y_int = -g_smooth.max_per_frame;
    }
    
    // Update sub-pixel accumulator with remainder
    g_smooth.x_accumulator_fp = frame_x_fp - int_to_fp(out_x_int);
    g_smooth.y_accumulator_fp = frame_y_fp - int_to_fp(out_y_int);
    
    // Output
    *out_x = clamp_i8(out_x_int);
    *out_y = clamp_i8(out_y_int);
    
    g_smooth.frames_processed++;
}

void smooth_get_velocity(int32_t *vel_x_fp, int32_t *vel_y_fp) {
    if (vel_x_fp) *vel_x_fp = g_smooth.velocity.avg_velocity_x_fp;
    if (vel_y_fp) *vel_y_fp = g_smooth.velocity.avg_velocity_y_fp;
}

void smooth_set_max_per_frame(int16_t max_per_frame) {
    g_smooth.max_per_frame = max_per_frame;
}

void smooth_set_velocity_matching(bool enabled) {
    g_smooth.velocity_matching_enabled = enabled;
}

void smooth_clear_queue(void) {
    for (int i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
        g_smooth.queue[i].active = false;
    }
    g_smooth.queue_count = 0;
    g_smooth.x_accumulator_fp = 0;
    g_smooth.y_accumulator_fp = 0;
}

void smooth_get_stats(uint32_t *total_injected, uint32_t *frames_processed, 
                      uint32_t *queue_overflows, uint8_t *queue_count) {
    if (total_injected) *total_injected = g_smooth.total_injected;
    if (frames_processed) *frames_processed = g_smooth.frames_processed;
    if (queue_overflows) *queue_overflows = g_smooth.queue_overflows;
    if (queue_count) *queue_count = g_smooth.queue_count;
}

bool smooth_has_pending(void) {
    if (g_smooth.queue_count > 0) return true;
    if (g_smooth.x_accumulator_fp != 0) return true;
    if (g_smooth.y_accumulator_fp != 0) return true;
    return false;
}
