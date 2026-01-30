/*
 * Smooth Injection System Implementation
 * 
 * Provides ultra-smooth mouse movement injection with sub-pixel precision,
 * velocity tracking, and temporal spreading for seamless blending with
 * physical mouse passthrough.
 * 
 * Humanization features:
 * - Bezier easing curves for natural acceleration/deceleration
 * - Micro-jitter injection to simulate hand tremor
 * - Variable thresholds and timing to avoid fingerprinting
 * - Overshoot/correction patterns for realism
 */

#include "smooth_injection.h"
#include "pico/stdlib.h"
#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"

// Persist state
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define HUMANIZATION_MAGIC 0x484D414E  // "HMAN"

// Forward declaration for internal use
static void smooth_set_humanization_mode_internal(humanization_mode_t mode, bool auto_save);


// Global state
static smooth_injection_state_t g_smooth = {0};

//--------------------------------------------------------------------+
// Fast PRNG for Humanization (xorshift32)
//--------------------------------------------------------------------+
static uint32_t g_free_bitmap = 0xFFFFFFFF;

static uint32_t g_rng_state = 0;

static void rng_seed(uint32_t seed) {
    g_rng_state = seed ? seed : 0x12345678;
}

static inline uint32_t rng_next(void) {
    uint32_t x = g_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng_state = x;
    return x;
}

// Random int in range [min, max] inclusive
static inline int32_t rng_range(int32_t min, int32_t max) {
    if (min >= max) return min;
    uint32_t range = (uint32_t)(max - min + 1);
    return min + (int32_t)(rng_next() % range);
}

// Random fixed-point in range [min_fp, max_fp]
static inline int32_t rng_range_fp(int32_t min_fp, int32_t max_fp) {
    if (min_fp >= max_fp) return min_fp;
    int32_t range = max_fp - min_fp;
    return min_fp + (int32_t)((rng_next() % (uint32_t)range));
}

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
// Bezier Easing Curves for Natural Movement
//--------------------------------------------------------------------+

// Cubic Bezier ease-in-out: slow start, fast middle, slow end (natural human movement)
// t in range [0, SMOOTH_FP_ONE], returns eased t in same range
static int32_t ease_in_out_cubic(int32_t t) {
    // Bezier curve with control points (0,0), (0.42,0), (0.58,1), (1,1)
    // Simplified cubic: if t < 0.5: 4*t^3, else: 1-4*(1-t)^3
    
    int32_t half = SMOOTH_FP_ONE / 2;
    if (t < half) {
        // Acceleration phase: 4*t^3
        int32_t t_scaled = fp_mul(t, int_to_fp(2)); // t*2
        int32_t t2 = fp_mul(t_scaled, t_scaled);
        int32_t t3 = fp_mul(t2, t_scaled);
        return t3 / 2; // 4*t^3 / 2 (since we scaled by 2)
    } else {
        // Deceleration phase: 1 - 4*(1-t)^3
        int32_t inv_t = SMOOTH_FP_ONE - t;
        int32_t inv_scaled = fp_mul(inv_t, int_to_fp(2));
        int32_t inv2 = fp_mul(inv_scaled, inv_scaled);
        int32_t inv3 = fp_mul(inv2, inv_scaled);
        return SMOOTH_FP_ONE - (inv3 / 2);
    }
}

// Ease-out (quick start, slow end) - good for sudden corrections
static int32_t ease_out_quad(int32_t t) {
    // 1 - (1-t)^2
    int32_t inv = SMOOTH_FP_ONE - t;
    int32_t inv2 = fp_mul(inv, inv);
    return SMOOTH_FP_ONE - inv2;
}

// Select easing curve based on movement characteristics
static int32_t apply_easing(int32_t t, easing_mode_t mode) {
    switch (mode) {
        case EASING_LINEAR:     return t;
        case EASING_EASE_IN_OUT: return ease_in_out_cubic(t);
        case EASING_EASE_OUT:   return ease_out_quad(t);
        default:                return t;
    }
}

//--------------------------------------------------------------------+
// Velocity Tracking
//--------------------------------------------------------------------+

static void velocity_update(int8_t x, int8_t y) {
    velocity_tracker_t *v = &g_smooth.velocity;
    
    // Get old value that will be replaced
    int8_t old_x = v->x_history[v->history_index];
    int8_t old_y = v->y_history[v->history_index];
    
    // Store new values in history
    v->x_history[v->history_index] = x;
    v->y_history[v->history_index] = y;
    v->history_index = (v->history_index + 1) % SMOOTH_VELOCITY_WINDOW;
    
    // Update running sum (remove old, add new) - O(1) instead of O(n)
    int32_t sum_x = (v->avg_velocity_x_fp * SMOOTH_VELOCITY_WINDOW) >> SMOOTH_FP_SHIFT;
    int32_t sum_y = (v->avg_velocity_y_fp * SMOOTH_VELOCITY_WINDOW) >> SMOOTH_FP_SHIFT;
    sum_x = sum_x - old_x + x;
    sum_y = sum_y - old_y + y;
    
    // Store as fixed-point average
    v->avg_velocity_x_fp = int_to_fp(sum_x) / SMOOTH_VELOCITY_WINDOW;
    v->avg_velocity_y_fp = int_to_fp(sum_y) / SMOOTH_VELOCITY_WINDOW;
}

// smooth state accessor for external use
typedef struct {
    uint32_t magic;
    humanization_mode_t mode;
    uint32_t checksum;
} humanization_persist_t;

//--------------------------------------------------------------------+
// Queue Management
//--------------------------------------------------------------------+

static smooth_queue_entry_t* queue_alloc(void) {
    if (g_smooth.queue_count >= SMOOTH_QUEUE_SIZE) {
        g_smooth.queue_overflows++;
        return NULL;
    }
    
    // O(1) allocation using bitmask to track free slots
    // Check 8 slots at a time using bit operations
    static uint32_t free_bitmap = 0xFFFFFFFF; // 1 = free, 0 = used
    
    if (free_bitmap == 0) {
        // Rebuild bitmap (should rarely happen)
        free_bitmap = 0;
        for (int i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
            if (!g_smooth.queue[i].active) {
                free_bitmap |= (1u << i);
            }
        }
        if (free_bitmap == 0) return NULL;
    }
    
    // Find first free slot using count trailing zeros (CTZ)
    int idx = __builtin_ctz(free_bitmap);
    free_bitmap &= ~(1u << idx);
    
    g_smooth.queue_count++;
    return &g_smooth.queue[idx];
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
    
    // Seed RNG with hardware time for per-session variation
    uint64_t time_us = time_us_64();
    rng_seed((uint32_t)(time_us ^ (time_us >> 32)));
    
    // Load saved humanization mode, or use default if none saved
    smooth_load_humanization_mode();
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
    
    // Variable max_per_frame with ±1 pixel variation per injection
    int32_t max_per_frame_fp = int_to_fp(g_smooth.max_per_frame) + 
                               rng_range_fp(-SMOOTH_FP_ONE, SMOOTH_FP_ONE);
    
    uint8_t frames = 1;
    if (max_component > max_per_frame_fp) {
        // Calculate frames needed (round up) with slight randomization
        frames = (uint8_t)((max_component + max_per_frame_fp - 1) / max_per_frame_fp);
        // Add ±1 frame variation for movements > 3 frames
        if (frames > 3 && rng_next() % 3 == 0) {
            frames += (rng_next() % 2) ? 1 : -1;
        }
        if (frames > 255) frames = 255;
        if (frames < 1) frames = 1;
    }
    
    // Choose easing curve based on movement size and mode
    easing_mode_t easing = EASING_LINEAR;
    if (max_component > int_to_fp(20)) {
        // Large movements: use smooth ease-in-out
        easing = EASING_EASE_IN_OUT;
    } else if (mode == INJECT_MODE_MICRO) {
        // Micro adjustments: quick ease-out
        easing = EASING_EASE_OUT;
    } else if (rng_next() % 4 == 0) {
        // 25% chance of easing for variety
        easing = EASING_EASE_IN_OUT;
    }
    
    // For velocity-matched mode, adjust based on current velocity
    if (mode == INJECT_MODE_VELOCITY_MATCHED && g_smooth.velocity_matching_enabled) {
        // Get magnitude of current velocity
        int32_t vel_mag = g_smooth.velocity.avg_velocity_x_fp;
        if (vel_mag < 0) vel_mag = -vel_mag;
        int32_t vel_y_abs = g_smooth.velocity.avg_velocity_y_fp;
        if (vel_y_abs < 0) vel_y_abs = -vel_y_abs;
        if (vel_y_abs > vel_mag) vel_mag = vel_y_abs;
        
        // Use variable thresholds instead of fixed values
        int32_t slow_thresh = g_smooth.humanization.vel_slow_threshold_fp;
        int32_t fast_thresh = g_smooth.humanization.vel_fast_threshold_fp;
        
        // If mouse is moving fast, allow faster injection
        // If mouse is slow/stopped, spread movement over more frames
        if (vel_mag < slow_thresh) {
            // Mouse nearly stationary - be extra smooth
            // Random multiplier: 1.5x to 2.5x
            frames = (frames * (uint8_t)rng_range(3, 5)) / 2;
            if (frames > 255) frames = 255;
        } else if (vel_mag > fast_thresh) {
            // Mouse moving fast - can inject faster
            frames = (frames + (uint8_t)rng_range(1, 2)) / 2;
            if (frames < 1) frames = 1;
        }
    }
    
    // Overshoot simulation for realism (occasional overshoot then correction)
    bool will_overshoot = false;
    int32_t overshoot_x_fp = 0, overshoot_y_fp = 0;
    if (g_smooth.humanization.jitter_enabled && 
        max_component > int_to_fp(30) && 
        (rng_next() % 100) < g_smooth.humanization.overshoot_chance) {
        will_overshoot = true;
        // Small overshoot in movement direction
        int32_t overshoot_mag = rng_range_fp(SMOOTH_FP_ONE, g_smooth.humanization.overshoot_max_fp);
        overshoot_x_fp = (x_fp > 0) ? overshoot_mag : ((x_fp < 0) ? -overshoot_mag : 0);
        overshoot_y_fp = (y_fp > 0) ? overshoot_mag : ((y_fp < 0) ? -overshoot_mag : 0);
    }
    
    entry->x_fp = x_fp;
    entry->y_fp = y_fp;
    entry->x_remaining_fp = x_fp;
    entry->y_remaining_fp = y_fp;
    entry->frames_left = frames;
    entry->total_frames = frames;
    entry->mode = mode;
    entry->easing = easing;
    entry->active = true;
    
    // Store overshoot for later injection
    entry->will_overshoot = will_overshoot;
    entry->overshoot_x_fp = overshoot_x_fp;
    entry->overshoot_y_fp = overshoot_y_fp;
    
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
    
    // Early exit if no active entries
    if (g_smooth.queue_count == 0) {
        goto apply_accumulator;
    }
    
    // Process active entries (typically only 1-3 active at a time)
    for (int i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
        smooth_queue_entry_t *entry = &g_smooth.queue[i];
        if (!entry->active) continue;
        
        if (entry->frames_left > 0) {
            // Calculate progress through movement (0.0 to 1.0 in fixed-point)
            uint8_t frames_elapsed = entry->total_frames - entry->frames_left;
            int32_t progress = fp_div(int_to_fp(frames_elapsed), int_to_fp(entry->total_frames));
            
            // Apply easing curve for natural acceleration/deceleration
            int32_t eased_progress = apply_easing(progress, entry->easing);
            int32_t prev_eased = (frames_elapsed > 0) ? 
                apply_easing(fp_div(int_to_fp(frames_elapsed - 1), int_to_fp(entry->total_frames)), entry->easing) : 0;
            
            // Calculate movement for this frame based on easing curve delta
            int32_t progress_delta = eased_progress - prev_eased;
            int32_t dx_fp = fp_mul(entry->x_fp, progress_delta);
            int32_t dy_fp = fp_mul(entry->y_fp, progress_delta);
            
            // Add micro-jitter for human tremor simulation (except for MICRO mode)
            if (g_smooth.humanization.jitter_enabled && entry->mode != INJECT_MODE_MICRO) {
                // Jitter varies: higher during movement, lower when static
                int32_t jitter_amount = g_smooth.humanization.jitter_amount_fp;
                
                // More jitter during mid-movement (when hand is actually moving)
                if (progress > int_to_fp(1)/4 && progress < int_to_fp(3)/4) {
                    jitter_amount = jitter_amount * 3 / 2; // 1.5x jitter during active movement
                }
                
                // Apply jitter with 40% probability per frame
                if (rng_next() % 5 < 2) {
                    dx_fp += rng_range_fp(-jitter_amount, jitter_amount);
                    dy_fp += rng_range_fp(-jitter_amount, jitter_amount);
                }
            }
            
            // Add to frame total
            frame_x_fp += dx_fp;
            frame_y_fp += dy_fp;
            
            // Update remaining (for fractional tracking)
            entry->x_remaining_fp -= dx_fp;
            entry->y_remaining_fp -= dy_fp;
            entry->frames_left--;
            
            // Check if done
            if (entry->frames_left == 0) {
                // Add any remaining fractional movement
                frame_x_fp += entry->x_remaining_fp;
                frame_y_fp += entry->y_remaining_fp;
                
                // Inject overshoot if planned
                if (entry->will_overshoot) {
                    // Queue correction movement (small opposing move)
                    smooth_queue_entry_t *correction = queue_alloc();
                    if (correction) {
                        correction->x_fp = -entry->overshoot_x_fp;
                        correction->y_fp = -entry->overshoot_y_fp;
                        correction->x_remaining_fp = -entry->overshoot_x_fp;
                        correction->y_remaining_fp = -entry->overshoot_y_fp;
                        correction->frames_left = (uint8_t)rng_range(2, 4); // Correct over 2-4 frames
                        correction->total_frames = correction->frames_left;
                        correction->mode = INJECT_MODE_SMOOTH;
                        correction->easing = EASING_EASE_OUT; // Quick correction
                        correction->active = true;
                        correction->will_overshoot = false;
                    }
                    // Add overshoot to current frame
                    frame_x_fp += entry->overshoot_x_fp;
                    frame_y_fp += entry->overshoot_y_fp;
                }
                
                queue_free(entry);
            }
        } else {
            queue_free(entry);
        }
    }
    
apply_accumulator:
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
    g_free_bitmap = 0xFFFFFFFF;
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

static void smooth_set_humanization_mode_internal(humanization_mode_t mode, bool auto_save) {
    if (mode >= HUMANIZATION_MODE_COUNT) mode = HUMANIZATION_MEDIUM;
    
    humanization_mode_t old_mode = g_smooth.humanization.mode;
    g_smooth.humanization.mode = mode;
    
    switch (mode) {
        case HUMANIZATION_OFF:
            // Disable all humanization
            g_smooth.max_per_frame = 16; // Fixed
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = false;
            g_smooth.humanization.jitter_amount_fp = 0;
            g_smooth.humanization.overshoot_chance = 0;
            g_smooth.humanization.overshoot_max_fp = 0;
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(2);
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(10);
            break;
            
        case HUMANIZATION_LOW:
            // Minimal humanization
            g_smooth.max_per_frame = (int16_t)rng_range(15, 17); // ±1
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(1) / 2; // ±0.5px
            g_smooth.humanization.overshoot_chance = 5; // 5% chance
            g_smooth.humanization.overshoot_max_fp = int_to_fp(1); // Max 1px
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(rng_range(2, 3));
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(rng_range(9, 11));
            break;
            
        case HUMANIZATION_MEDIUM:
            // Balanced humanization (default)
            g_smooth.max_per_frame = (int16_t)rng_range(14, 18); // ±2
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(1); // ±1px
            g_smooth.humanization.overshoot_chance = 15; // 15% chance
            g_smooth.humanization.overshoot_max_fp = int_to_fp(2); // Max 2px
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(rng_range(2, 3));
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(rng_range(9, 12));
            break;
            
        case HUMANIZATION_HIGH:
            // Maximum variation
            g_smooth.max_per_frame = (int16_t)rng_range(12, 20); // ±4
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(2); // ±2px
            g_smooth.humanization.overshoot_chance = 25; // 25% chance
            g_smooth.humanization.overshoot_max_fp = int_to_fp(3); // Max 3px
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(rng_range(1, 4));
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(rng_range(8, 15));
            break;
            
        default:
            smooth_set_humanization_mode_internal(HUMANIZATION_MEDIUM, auto_save);
            return;
    }
    
    // Save to flash if mode actually changed and auto_save is enabled
    if (auto_save && old_mode != mode) {
        smooth_save_humanization_mode();
    }
}

void smooth_set_humanization_mode(humanization_mode_t mode) {
    smooth_set_humanization_mode_internal(mode, true);
}

humanization_mode_t smooth_get_humanization_mode(void) {
    return g_smooth.humanization.mode;
}

humanization_mode_t smooth_cycle_humanization_mode(void) {
    humanization_mode_t new_mode = (humanization_mode_t)((g_smooth.humanization.mode + 1) % HUMANIZATION_MODE_COUNT);
    smooth_set_humanization_mode(new_mode);
    return new_mode;
}

// save smooth state to flash

void smooth_save_humanization_mode(void) {
    humanization_persist_t data = {
        .magic = HUMANIZATION_MAGIC,
        .mode = g_smooth.humanization.mode,
        .checksum = HUMANIZATION_MAGIC ^ g_smooth.humanization.mode
    };
    
    // Ensure data structure is properly aligned and sized for flash
    static_assert(sizeof(humanization_persist_t) <= FLASH_SECTOR_SIZE, "Data too large for flash sector");
    static_assert((sizeof(humanization_persist_t) % 4) == 0, "Data must be 4-byte aligned for flash");
    
    uint32_t ints = save_and_disable_interrupts();
    
    // Erase the sector (required before programming)
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    
    // Program the data
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t*)&data, sizeof(data));
    
    restore_interrupts(ints);
}

void smooth_load_humanization_mode(void) {
    const humanization_persist_t *data = (const humanization_persist_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    
    // Validate magic number, checksum, and mode range
    if (data->magic == HUMANIZATION_MAGIC && 
        data->checksum == (HUMANIZATION_MAGIC ^ data->mode) &&
        data->mode < HUMANIZATION_MODE_COUNT) {
        
        // Load without auto-save to avoid recursion
        smooth_set_humanization_mode_internal(data->mode, false);
    } else {
        // No valid data found, use default (without auto-save during init)
        smooth_set_humanization_mode_internal(HUMANIZATION_MEDIUM, false);
    }
}
