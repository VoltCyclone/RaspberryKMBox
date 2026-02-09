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
#include "humanization_lut.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "pico/rand.h"

// Persist state
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define HUMANIZATION_MAGIC 0x484D414E  // "HMAN"

// Deferred flash save state (to avoid crashing multicore during button press)
static volatile bool g_save_pending = false;
static volatile uint32_t g_save_request_time = 0;
#define SAVE_DEFER_MS 2000  // Wait 2 seconds after last mode change before saving

// Safety tracking for failed flash operations
static uint8_t g_flash_save_failures = 0;
#define MAX_FLASH_FAILURES 3  // Disable flash saves after this many failures

// Forward declaration for internal use
static void smooth_set_humanization_mode_internal(humanization_mode_t mode, bool auto_save);


// Global state
static smooth_injection_state_t g_smooth = {0};

// Track last injection mode for stats
static inject_mode_t g_last_inject_mode = INJECT_MODE_SMOOTH;

// Active queue linked list for O(1) iteration
// Instead of scanning all SMOOTH_QUEUE_SIZE slots, maintain a list of active entries
typedef struct active_queue_node {
    uint8_t index;  // Index into g_smooth.queue[]
    struct active_queue_node *next;
} active_queue_node_t;

static active_queue_node_t g_active_nodes[SMOOTH_QUEUE_SIZE];
static active_queue_node_t *g_active_head = NULL;
static uint32_t g_active_node_bitmap = 0;  // Single uint32_t instead of byte array

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

// RP2350 DSP-optimized fixed-point multiply (16.16 format)
// Uses SMULL to get full 64-bit product, then extracts middle 32 bits (>> 16).
// SMMULR only returns the top 32 bits (>> 32), losing 16 bits of precision
// which causes small values (e.g., 0.5 * 0.5) to round to zero.
static inline int32_t __not_in_flash_func(fp_mul)(int32_t a, int32_t b) {
    int32_t hi;
    uint32_t lo;
    __asm__ volatile (
        "smull %0, %1, %2, %3"
        : "=r" (lo), "=r" (hi)
        : "r" (a), "r" (b)
    );
    return (int32_t)((uint32_t)(hi << SMOOTH_FP_SHIFT) | (lo >> SMOOTH_FP_SHIFT));
}

static __force_inline int32_t fp_div(int32_t a, int32_t b) {
    if (b == 0) return 0;
    return (int32_t)(((int64_t)a << SMOOTH_FP_SHIFT) / b);
}

static __force_inline int32_t int_to_fp(int16_t val) {
    return (int32_t)val << SMOOTH_FP_SHIFT;
}

static __force_inline int16_t fp_to_int(int32_t fp_val) {
    // Round to nearest integer
    if (fp_val >= 0) {
        return (int16_t)((fp_val + SMOOTH_FP_HALF) >> SMOOTH_FP_SHIFT);
    } else {
        return (int16_t)((fp_val - SMOOTH_FP_HALF) >> SMOOTH_FP_SHIFT);
    }
}

static __force_inline int8_t clamp_i8(int32_t val) {
    if (val > 127) return 127;
    if (val < -128) return -128;
    return (int8_t)val;
}

//--------------------------------------------------------------------+
// Easing Curves - Now using precomputed LUT for performance
//--------------------------------------------------------------------+

// apply_easing now uses lut_apply_easing() from humanization_lut.h
// This provides ~10x speedup and smoother interpolation
#define apply_easing(t, mode) lut_apply_easing((t), (mode))

//--------------------------------------------------------------------+
// Velocity Tracking
//--------------------------------------------------------------------+

// Persistent fixed-point accumulators to avoid precision loss during conversion
static int32_t g_velocity_sum_x_fp = 0;
static int32_t g_velocity_sum_y_fp = 0;

static void velocity_update(int16_t x, int16_t y) {
    velocity_tracker_t *v = &g_smooth.velocity;
    
    // Get old value that will be replaced
    int16_t old_x = v->x_history[v->history_index];
    int16_t old_y = v->y_history[v->history_index];
    
    // Store new values in history
    v->x_history[v->history_index] = x;
    v->y_history[v->history_index] = y;
    v->history_index = (v->history_index + 1) % SMOOTH_VELOCITY_WINDOW;
    
    // Update running sum in fixed-point (remove old, add new) - O(1)
    // Keep accumulators in fixed-point to preserve precision across cycles
    g_velocity_sum_x_fp = g_velocity_sum_x_fp - int_to_fp(old_x) + int_to_fp(x);
    g_velocity_sum_y_fp = g_velocity_sum_y_fp - int_to_fp(old_y) + int_to_fp(y);
    
    // Store as fixed-point average (no precision loss)
    v->avg_velocity_x_fp = g_velocity_sum_x_fp / SMOOTH_VELOCITY_WINDOW;
    v->avg_velocity_y_fp = g_velocity_sum_y_fp / SMOOTH_VELOCITY_WINDOW;
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
    
    // O(1) allocation using global bitmask to track free slots
    // Check 8 slots at a time using bit operations
    // Note: Uses global g_free_bitmap (declared at line 42) for proper synchronization
    
    if (g_free_bitmap == 0) {
        // Rebuild bitmap (should rarely happen)
        g_free_bitmap = 0;
        for (int i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
            if (!g_smooth.queue[i].active) {
                g_free_bitmap |= (1u << i);
            }
        }
        if (g_free_bitmap == 0) return NULL;
    }
    
    // Find first free slot using count trailing zeros (CTZ)
    int idx = __builtin_ctz(g_free_bitmap);
    g_free_bitmap &= ~(1u << idx);
    
    g_smooth.queue_count++;
    
    // Add to active linked list for fast iteration - O(1) using CTZ
    if (g_active_node_bitmap == 0xFFFFFFFF) {
        // Shouldn't happen since we check queue_count, but handle gracefully
        return &g_smooth.queue[idx];
    }
    
    // O(1) node allocation using count trailing zeros
    int node_idx = __builtin_ctz(~g_active_node_bitmap);
    g_active_node_bitmap |= (1u << node_idx);
    
    // Link node
    g_active_nodes[node_idx].index = idx;
    g_active_nodes[node_idx].next = g_active_head;
    g_active_head = &g_active_nodes[node_idx];
    
    return &g_smooth.queue[idx];
}

static void queue_free(smooth_queue_entry_t *entry) {
    if (entry && entry->active) {
        entry->active = false;
        if (g_smooth.queue_count > 0) {
            g_smooth.queue_count--;
        }
        // Restore bit in free bitmap for reallocation
        int idx = entry - g_smooth.queue;
        if (idx >= 0 && idx < SMOOTH_QUEUE_SIZE) {
            g_free_bitmap |= (1u << idx);
            
            // Remove from active linked list
            active_queue_node_t **current = &g_active_head;
            while (*current) {
                if ((*current)->index == idx) {
                    active_queue_node_t *to_free = *current;
                    *current = (*current)->next;
                    // Mark node as free in bitmap - O(1)
                    int node_idx = to_free - g_active_nodes;
                    g_active_node_bitmap &= ~(1u << node_idx);
                    break;
                }
                current = &(*current)->next;
            }
        }
    }
}

//--------------------------------------------------------------------+
// Public API Implementation
//--------------------------------------------------------------------+

void smooth_injection_init(void) {
    memset(&g_smooth, 0, sizeof(g_smooth));
    
    // Reset global state that's outside g_smooth structure
    g_free_bitmap = 0xFFFFFFFF;  // All slots free
    g_velocity_sum_x_fp = 0;     // Reset velocity accumulators
    g_velocity_sum_y_fp = 0;
    
    // Initialize active linked list
    g_active_head = NULL;
    g_active_node_bitmap = 0;  // All nodes free
    
    // Seed RNG with hardware TRNG for true entropy (RP2350 has hardware TRNG)
    // Falls back to ROSC + time entropy on RP2040
    uint32_t hw_entropy = get_rand_32();
    rng_seed(hw_entropy);
    
    // Load saved humanization mode, or use default if none saved
    smooth_load_humanization_mode();
}

bool smooth_inject_movement(int16_t x, int16_t y, inject_mode_t mode) {
    return smooth_inject_movement_fp(int_to_fp(x), int_to_fp(y), mode);
}

//--------------------------------------------------------------------+
// Internal: Queue a single sub-step movement entry
// This is the core queuing logic, separated from subdivision.
//--------------------------------------------------------------------+
static bool queue_single_substep(int32_t x_fp, int32_t y_fp, inject_mode_t mode,
                                  uint8_t delay_frames) {
    smooth_queue_entry_t *entry = queue_alloc();
    if (!entry) {
        // Queue full - fall back to immediate injection
        g_smooth.x_accumulator_fp += x_fp;
        g_smooth.y_accumulator_fp += y_fp;
        return false;
    }
    
    // Calculate how many frames this sub-step should span
    int32_t abs_x = x_fp >= 0 ? x_fp : -x_fp;
    int32_t abs_y = y_fp >= 0 ? y_fp : -y_fp;
    int32_t max_component = abs_x > abs_y ? abs_x : abs_y;
    
    // Variable max_per_frame with ±1 pixel variation per sub-step
    int32_t max_per_frame_fp = int_to_fp(g_smooth.max_per_frame) + 
                               rng_range_fp(-SMOOTH_FP_ONE, SMOOTH_FP_ONE);
    
    uint8_t frames = 1;
    if (max_component > max_per_frame_fp) {
        frames = (uint8_t)((max_component + max_per_frame_fp - 1) / max_per_frame_fp);
        
        int32_t movement_px = fp_to_int(max_component);
        int32_t frame_multiplier = lut_get_frame_spread_for_movement(movement_px);
        frames = (uint8_t)((frames * frame_multiplier) >> SMOOTH_FP_SHIFT);
        
        if (frames < 1) frames = 1;
    }
    
    // Add natural latency jitter (0-1 frame)
    if (mode != INJECT_MODE_IMMEDIATE) {
        frames = frames + (uint8_t)rng_range(0, 1);
    }
    
    // Add inter-substep delay to stagger execution (sequential, not parallel)
    frames = frames + delay_frames;
    
    // Choose easing curve with per-substep variety
    easing_mode_t easing = EASING_LINEAR;
    if (max_component > int_to_fp(10)) {
        // Larger sub-steps: smooth easing
        easing = EASING_EASE_IN_OUT;
    } else if (rng_next() % 3 == 0) {
        // 33% chance of ease-out for variety within sub-steps
        easing = EASING_EASE_OUT;
    } else if (rng_next() % 4 == 0) {
        easing = EASING_EASE_IN_OUT;
    }
    
    // For velocity-matched mode, adjust based on current velocity
    if (mode == INJECT_MODE_VELOCITY_MATCHED && g_smooth.velocity_matching_enabled) {
        int32_t vel_mag = g_smooth.velocity.avg_velocity_x_fp;
        if (vel_mag < 0) vel_mag = -vel_mag;
        int32_t vel_y_abs = g_smooth.velocity.avg_velocity_y_fp;
        if (vel_y_abs < 0) vel_y_abs = -vel_y_abs;
        if (vel_y_abs > vel_mag) vel_mag = vel_y_abs;
        
        int32_t slow_thresh = g_smooth.humanization.vel_slow_threshold_fp;
        int32_t fast_thresh = g_smooth.humanization.vel_fast_threshold_fp;
        
        if (vel_mag < slow_thresh) {
            uint16_t temp = (frames * (uint8_t)rng_range(3, 5)) / 2;
            frames = (temp > 255) ? 255 : (uint8_t)temp;
        } else if (vel_mag > fast_thresh) {
            frames = (frames + (uint8_t)rng_range(1, 2)) / 2;
            if (frames < 1) frames = 1;
        }
    }
    
    // Overshoot only on the final sub-step (handled by caller)
    entry->x_fp = x_fp;
    entry->y_fp = y_fp;
    entry->x_remaining_fp = x_fp;
    entry->y_remaining_fp = y_fp;
    entry->frames_left = frames;
    entry->total_frames = frames;
    entry->mode = mode;
    entry->easing = easing;
    entry->active = true;
    entry->will_overshoot = false;
    entry->overshoot_x_fp = 0;
    entry->overshoot_y_fp = 0;
    
    return true;
}

//--------------------------------------------------------------------+
// Movement Subdivision for Humanization
//
// Instead of queuing a single large movement, break it into 4-8
// smaller sub-movements with randomized split ratios and independent
// timing/easing per step. This creates a more organic movement
// signature that's much harder to fingerprint.
//
// Subdivision strategy:
// - Movements >= 3px: subdivide into 4-6 sub-steps
// - Each sub-step gets a random fraction (~25% ± jitter) of the total
// - Sub-steps are staggered with small frame delays between them
// - The final sub-step gets the exact remainder (no drift)
// - Overshoot is only applied to the final sub-step
//--------------------------------------------------------------------+

// Minimum movement magnitude (in pixels) to trigger subdivision
#define SUBSTEP_MIN_MOVEMENT_PX     3

// Number of sub-steps to split into (base value, randomized)
#define SUBSTEP_COUNT_BASE          4
#define SUBSTEP_COUNT_EXTRA_MAX     4   // Up to +4 extra = 4-8 total

// Frame delay between consecutive sub-steps (randomized)
#define SUBSTEP_DELAY_MIN           1
#define SUBSTEP_DELAY_MAX           3

bool smooth_inject_movement_fp(int32_t x_fp, int32_t y_fp, inject_mode_t mode) {
    // Track injection mode for stats
    g_last_inject_mode = mode;
    
    // For immediate mode, just add to accumulator (no subdivision)
    if (mode == INJECT_MODE_IMMEDIATE) {
        g_smooth.x_accumulator_fp += x_fp;
        g_smooth.y_accumulator_fp += y_fp;
        g_smooth.total_injected++;
        return true;
    }
    
    // For micro mode, add directly (no subdivision - sub-pixel precision needed)
    if (mode == INJECT_MODE_MICRO) {
        g_smooth.x_accumulator_fp += x_fp;
        g_smooth.y_accumulator_fp += y_fp;
        g_smooth.total_injected++;
        return true;
    }
    
    // Determine if we should subdivide this movement
    int32_t abs_x = x_fp >= 0 ? x_fp : -x_fp;
    int32_t abs_y = y_fp >= 0 ? y_fp : -y_fp;
    int32_t max_component = abs_x > abs_y ? abs_x : abs_y;
    int32_t movement_px = fp_to_int(max_component);
    
    // Only subdivide when:
    // 1) Humanization is enabled (not OFF)
    // 2) Movement is large enough to split meaningfully
    // 3) We have enough queue space for the sub-steps
    bool should_subdivide = (g_smooth.humanization.mode != HUMANIZATION_OFF) &&
                            (movement_px >= SUBSTEP_MIN_MOVEMENT_PX) &&
                            (g_smooth.queue_count + SUBSTEP_COUNT_BASE <= SMOOTH_QUEUE_SIZE);
    
    if (!should_subdivide) {
        // Fall back to single-entry path (legacy behavior)
        // Calculate overshoot for the single entry
        bool will_overshoot = false;
        int32_t overshoot_x_fp = 0, overshoot_y_fp = 0;
        if (g_smooth.humanization.jitter_enabled && 
            movement_px >= 30 && movement_px <= 120 && 
            (rng_next() % 100) < g_smooth.humanization.overshoot_chance) {
            will_overshoot = true;
            int32_t overshoot_base = lut_get_overshoot(movement_px);
            int32_t overshoot_mag = fp_mul(overshoot_base, g_smooth.humanization.overshoot_max_fp);
            if (overshoot_mag > g_smooth.humanization.overshoot_max_fp) {
                overshoot_mag = g_smooth.humanization.overshoot_max_fp;
            }
            overshoot_x_fp = (x_fp > 0) ? overshoot_mag : ((x_fp < 0) ? -overshoot_mag : 0);
            overshoot_y_fp = (y_fp > 0) ? overshoot_mag : ((y_fp < 0) ? -overshoot_mag : 0);
        }
        
        bool ok = queue_single_substep(x_fp, y_fp, mode, 0);
        if (ok && will_overshoot) {
            // Patch overshoot onto the entry we just created (it's the most recent)
            // Find it via the active list head (queue_single_substep adds to head)
            if (g_active_head) {
                smooth_queue_entry_t *last = &g_smooth.queue[g_active_head->index];
                last->will_overshoot = true;
                last->overshoot_x_fp = overshoot_x_fp;
                last->overshoot_y_fp = overshoot_y_fp;
            }
        }
        g_smooth.total_injected++;
        return ok;
    }
    
    //--------------------------------------------------------------------+
    // Subdivided injection path
    //--------------------------------------------------------------------+
    
    // Determine number of sub-steps based on humanization level
    uint8_t num_substeps = SUBSTEP_COUNT_BASE;
    switch (g_smooth.humanization.mode) {
        case HUMANIZATION_LOW:
            num_substeps += (uint8_t)rng_range(0, 1);  // 4-5 sub-steps
            break;
        case HUMANIZATION_MEDIUM:
            num_substeps += (uint8_t)rng_range(0, 2);  // 4-6 sub-steps
            break;
        case HUMANIZATION_HIGH:
            num_substeps += (uint8_t)rng_range(1, SUBSTEP_COUNT_EXTRA_MAX);  // 5-8 sub-steps
            break;
        default:
            break;
    }
    
    // Clamp to available queue space
    uint8_t available = SMOOTH_QUEUE_SIZE - g_smooth.queue_count;
    if (num_substeps > available) {
        num_substeps = available;
    }
    if (num_substeps < 2) {
        // Not enough space, fall back to single entry
        bool ok = queue_single_substep(x_fp, y_fp, mode, 0);
        g_smooth.total_injected++;
        return ok;
    }
    
    // Generate randomized split ratios that sum to SMOOTH_FP_ONE
    // Each sub-step gets roughly 1/N of the total, ±30% jitter
    int32_t ratios[8];  // Max 8 sub-steps
    int32_t base_ratio = SMOOTH_FP_ONE / num_substeps;
    int32_t jitter_range = base_ratio * 3 / 10;  // ±30% of base
    int32_t ratio_sum = 0;
    
    for (uint8_t i = 0; i < num_substeps - 1; i++) {
        int32_t r = base_ratio + rng_range_fp(-jitter_range, jitter_range);
        if (r < SMOOTH_FP_ONE / (num_substeps * 4)) {
            r = SMOOTH_FP_ONE / (num_substeps * 4);  // Floor: at least 25% of equal share
        }
        ratios[i] = r;
        ratio_sum += r;
    }
    // Final sub-step gets the exact remainder (prevents drift)
    ratios[num_substeps - 1] = SMOOTH_FP_ONE - ratio_sum;
    if (ratios[num_substeps - 1] < 0) {
        // Safety: redistribute if jitter pushed us over
        ratios[num_substeps - 1] = SMOOTH_FP_ONE / num_substeps;
    }
    
    // Calculate overshoot (apply only to the final sub-step)
    bool will_overshoot = false;
    int32_t overshoot_x_fp = 0, overshoot_y_fp = 0;
    if (g_smooth.humanization.jitter_enabled && 
        movement_px >= 30 && movement_px <= 120 && 
        (rng_next() % 100) < g_smooth.humanization.overshoot_chance) {
        will_overshoot = true;
        int32_t overshoot_base = lut_get_overshoot(movement_px);
        int32_t overshoot_mag = fp_mul(overshoot_base, g_smooth.humanization.overshoot_max_fp);
        if (overshoot_mag > g_smooth.humanization.overshoot_max_fp) {
            overshoot_mag = g_smooth.humanization.overshoot_max_fp;
        }
        overshoot_x_fp = (x_fp > 0) ? overshoot_mag : ((x_fp < 0) ? -overshoot_mag : 0);
        overshoot_y_fp = (y_fp > 0) ? overshoot_mag : ((y_fp < 0) ? -overshoot_mag : 0);
    }
    
    // Queue each sub-step with staggered delays
    int32_t x_remaining = x_fp;
    int32_t y_remaining = y_fp;
    uint8_t cumulative_delay = 0;
    bool all_ok = true;
    
    for (uint8_t i = 0; i < num_substeps; i++) {
        int32_t step_x, step_y;
        
        if (i == num_substeps - 1) {
            // Final sub-step: use exact remainder to prevent any drift
            step_x = x_remaining;
            step_y = y_remaining;
        } else {
            step_x = fp_mul(x_fp, ratios[i]);
            step_y = fp_mul(y_fp, ratios[i]);
            x_remaining -= step_x;
            y_remaining -= step_y;
        }
        
        // Skip sub-steps that are essentially zero
        if (step_x == 0 && step_y == 0) continue;
        
        // Stagger: each sub-step gets a small random delay on top of cumulative
        uint8_t inter_delay = (i == 0) ? 0 : (uint8_t)rng_range(SUBSTEP_DELAY_MIN, SUBSTEP_DELAY_MAX);
        cumulative_delay += inter_delay;
        
        bool ok = queue_single_substep(step_x, step_y, mode, cumulative_delay);
        
        if (ok && i == num_substeps - 1 && will_overshoot) {
            // Attach overshoot to the final sub-step
            if (g_active_head) {
                smooth_queue_entry_t *last = &g_smooth.queue[g_active_head->index];
                last->will_overshoot = true;
                last->overshoot_x_fp = overshoot_x_fp;
                last->overshoot_y_fp = overshoot_y_fp;
            }
        }
        
        if (!ok) {
            all_ok = false;
            // Remaining movement was added to accumulator by queue_single_substep
            // Don't try to queue more sub-steps
            // But add the rest of the unqueued movement to the accumulator
            if (i < num_substeps - 1) {
                g_smooth.x_accumulator_fp += x_remaining;
                g_smooth.y_accumulator_fp += y_remaining;
            }
            break;
        }
    }
    
    g_smooth.total_injected++;
    return all_ok;
}

void smooth_record_physical_movement(int16_t x, int16_t y) {
    velocity_update(x, y);
}

void __not_in_flash_func(smooth_process_frame)(int8_t *out_x, int8_t *out_y) {
    // Super-fast path for empty queue with no accumulator
    if (g_smooth.queue_count == 0 && 
        g_smooth.x_accumulator_fp == 0 && 
        g_smooth.y_accumulator_fp == 0) {
        *out_x = 0;
        *out_y = 0;
        g_smooth.frames_processed++;
        return;
    }
    
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
    
    // Safety: if queue_count > 0 but linked list is empty, reset to prevent hang
    if (g_active_head == NULL) {
        g_smooth.queue_count = 0;
        g_free_bitmap = 0xFFFFFFFF;
        g_active_node_bitmap = 0;
        goto apply_accumulator;
    }
    
    // Process active entries using linked list (O(n) where n = active count, not queue size)
    active_queue_node_t *node = g_active_head;
    while (node) {
        smooth_queue_entry_t *entry = &g_smooth.queue[node->index];
        active_queue_node_t *next_node = node->next;  // Save next before potential free
        
        if (entry->frames_left > 0) {
            // Calculate progress using LUT (eliminates division in hot path)
            uint8_t frames_elapsed = entry->total_frames - entry->frames_left;
            int32_t progress = lut_get_progress(entry->total_frames, frames_elapsed);
            
            // Apply easing curve using LUT with interpolation
            int32_t eased_progress = apply_easing(progress, entry->easing);
            int32_t prev_eased = (frames_elapsed > 0) ? 
                apply_easing(lut_get_progress(entry->total_frames, frames_elapsed - 1), entry->easing) : 0;
            
            // Calculate movement for this frame based on easing curve delta
            int32_t progress_delta = eased_progress - prev_eased;
            int32_t dx_fp = fp_mul(entry->x_fp, progress_delta);
            int32_t dy_fp = fp_mul(entry->y_fp, progress_delta);
            
            // Add micro-jitter for human tremor simulation (except for MICRO mode)
            // Use movement-aware jitter scaling from LUT:
            // - Small movements (0-20px): More jitter (stationary hand, precise)
            // - Medium movements (20-100px): Moderate jitter
            // - Large movements (100+px): Less jitter (intentional, tremor suppressed)
            if (g_smooth.humanization.jitter_enabled && entry->mode != INJECT_MODE_MICRO) {
                // Calculate movement magnitude once
                int32_t move_mag = (entry->x_fp < 0 ? -entry->x_fp : entry->x_fp);
                int32_t y_mag = (entry->y_fp < 0 ? -entry->y_fp : entry->y_fp);
                if (y_mag > move_mag) move_mag = y_mag;
                int32_t move_px = fp_to_int(move_mag);
                
                // Get current velocity magnitude for suppression
                int32_t vel_mag = g_smooth.velocity.avg_velocity_x_fp;
                if (vel_mag < 0) vel_mag = -vel_mag;
                int32_t vel_y_abs = g_smooth.velocity.avg_velocity_y_fp;
                if (vel_y_abs < 0) vel_y_abs = -vel_y_abs;
                if (vel_y_abs > vel_mag) vel_mag = vel_y_abs;
                
                // Suppress jitter when velocity is very low (movement has ended)
                // Rapid decay: if velocity < 1px/frame, reduce jitter dramatically
                int32_t velocity_scale = SMOOTH_FP_ONE;
                if (vel_mag < int_to_fp(1)) {
                    // Scale from 1.0x down to 0.1x as velocity approaches 0
                    velocity_scale = (vel_mag * 9) / int_to_fp(1) + (SMOOTH_FP_ONE / 10);
                }
                
                // Get movement-aware jitter scale from LUT
                int32_t movement_jitter_scale = lut_get_jitter_scale_for_movement(move_px);
                int32_t jitter_scale = fp_mul(g_smooth.humanization.jitter_amount_fp, movement_jitter_scale);
                
                // Apply velocity-based suppression (stops jitter when movement ends)
                jitter_scale = fp_mul(jitter_scale, velocity_scale);
                
                // Get jitter from precomputed LUT
                int32_t jitter_x, jitter_y;
                lut_get_jitter(g_smooth.frames_processed, jitter_scale, &jitter_x, &jitter_y);
                dx_fp += jitter_x;
                dy_fp += jitter_y;
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
        
        node = next_node;  // Move to next active entry
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
    // CRITICAL: Only keep the sub-pixel fractional residual, not excess from rate limiting.
    // Rate-limited excess was already accounted for in the queue entries' remaining fields.
    // Keeping it here would cause it to trickle out slowly after the queue drains, appearing
    // as a movement hang/drift.
    g_smooth.x_accumulator_fp = frame_x_fp - int_to_fp(out_x_int);
    g_smooth.y_accumulator_fp = frame_y_fp - int_to_fp(out_y_int);
    
    // Clamp accumulator to prevent unbounded growth from rate limiting
    // Max 2 pixels of residual (in fixed-point) prevents movement hang
    const int32_t max_accum = int_to_fp(2);
    if (g_smooth.x_accumulator_fp > max_accum) g_smooth.x_accumulator_fp = max_accum;
    else if (g_smooth.x_accumulator_fp < -max_accum) g_smooth.x_accumulator_fp = -max_accum;
    if (g_smooth.y_accumulator_fp > max_accum) g_smooth.y_accumulator_fp = max_accum;
    else if (g_smooth.y_accumulator_fp < -max_accum) g_smooth.y_accumulator_fp = -max_accum;
    
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
    // Reset active linked list to prevent stale pointer traversal
    g_active_head = NULL;
    g_active_node_bitmap = 0;
    // Also reset velocity tracking accumulators for consistency
    g_velocity_sum_x_fp = 0;
    g_velocity_sum_y_fp = 0;
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
            // Minimal humanization - very light, only on synthetic movement
            g_smooth.max_per_frame = 16; // Fixed
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(1) / 32; // ±0.03px (tiny)
            g_smooth.humanization.overshoot_chance = 0;
            g_smooth.humanization.overshoot_max_fp = 0;
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(rng_range(2, 3));
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(rng_range(9, 11));
            break;
            
        case HUMANIZATION_MEDIUM:
            // Balanced - light humanization on synthetic movements
            g_smooth.max_per_frame = 16; // Fixed
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(1) / 16; // ±0.06px (light)
            g_smooth.humanization.overshoot_chance = 0;
            g_smooth.humanization.overshoot_max_fp = 0;
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(rng_range(2, 3));
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(rng_range(9, 12));
            break;
            
        case HUMANIZATION_HIGH:
            // Maximum - moderate humanization on synthetic movements
            g_smooth.max_per_frame = 16; // Fixed
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(1) / 10; // ±0.1px (moderate)
            g_smooth.humanization.overshoot_chance = 0;
            g_smooth.humanization.overshoot_max_fp = 0;
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(rng_range(1, 4));
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(rng_range(8, 15));
            break;
            
        default:
            smooth_set_humanization_mode_internal(HUMANIZATION_MEDIUM, auto_save);
            return;
    }
    
    // Save to flash if mode actually changed and auto_save is enabled
    // NOTE: We defer the save to avoid flash write issues with multicore
    if (auto_save && old_mode != mode) {
        g_save_pending = true;
        g_save_request_time = to_ms_since_boot(get_absolute_time());
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
// Uses pico_flash's flash_safe_execute() for safe multicore flash operations.
// This handles Core1 lockout automatically via multicore_lockout.
extern volatile bool g_flash_operation_in_progress;

// Callback parameter for flash_safe_execute
typedef struct {
    humanization_persist_t data;
} flash_save_param_t;

// Flash write callback — called by flash_safe_execute with IRQs disabled
// and Core1 safely paused. Runs from RAM.
static void __not_in_flash_func(flash_write_callback)(void *param) {
    flash_save_param_t *p = (flash_save_param_t *)param;
    
    // Erase the sector (required before programming)
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    
    // Program the data
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t*)&p->data, sizeof(p->data));
}

// Internal flash save - uses flash_safe_execute for automatic multicore safety
static void __not_in_flash_func(smooth_save_humanization_mode_internal)(void) {
    // Ensure data structure is properly aligned and sized for flash
    static_assert(sizeof(humanization_persist_t) <= FLASH_SECTOR_SIZE, "Data too large for flash sector");
    static_assert((sizeof(humanization_persist_t) % 4) == 0, "Data must be 4-byte aligned for flash");
    
    flash_save_param_t param = {
        .data = {
            .magic = HUMANIZATION_MAGIC,
            .mode = g_smooth.humanization.mode,
            .checksum = HUMANIZATION_MAGIC ^ g_smooth.humanization.mode
        }
    };
    
    int result = flash_safe_execute(flash_write_callback, &param, 500);
    if (result != PICO_OK) {
        // flash_safe_execute handles all the multicore coordination;
        // if it fails, increment our failure counter
        g_flash_save_failures++;
    }
}

void smooth_save_humanization_mode(void) {
    // Request deferred save instead of immediate save
    g_save_pending = true;
    g_save_request_time = to_ms_since_boot(get_absolute_time());
}

// Core1 acknowledgment flag — kept for backward compat but flash_safe_execute
// handles multicore coordination automatically now.
extern volatile bool g_core1_flash_acknowledged;

// Call this periodically from main loop.
// flash_safe_execute() handles Core1 lockout automatically via multicore_lockout.
void smooth_process_deferred_save(void) {
    if (!g_save_pending) return;
    
    // Safety check - disable flash saves if we've had too many failures
    if (g_flash_save_failures >= MAX_FLASH_FAILURES) {
        printf("Flash saves disabled after %d failures - mode changes are temporary\n", g_flash_save_failures);
        g_save_pending = false;
        return;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Wait for the defer period to allow rapid button presses
    if ((current_time - g_save_request_time) < SAVE_DEFER_MS) return;
    
    // flash_safe_execute handles all multicore coordination:
    // - Pauses Core1 via multicore_lockout
    // - Disables interrupts
    // - Calls our callback
    // - Resumes Core1 and restores interrupts
    smooth_save_humanization_mode_internal();
    
    if (g_flash_save_failures == 0 || g_flash_save_failures < MAX_FLASH_FAILURES) {
        g_save_pending = false;
        g_flash_save_failures = 0;  // Reset failure counter on success
        printf("Humanization mode saved to flash successfully\n");
    } else {
        // Retry later
        g_save_request_time = current_time;
    }
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

//--------------------------------------------------------------------+
// Getter Functions for Settings Query
//--------------------------------------------------------------------+

int16_t smooth_get_max_per_frame(void) {
    return g_smooth.max_per_frame;
}

bool smooth_get_velocity_matching(void) {
    return g_smooth.velocity_matching_enabled;
}

inject_mode_t smooth_get_inject_mode(void) {
    return g_last_inject_mode;
}
