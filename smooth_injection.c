/*
 * Smooth Injection System Implementation
 * 
 * Provides ultra-smooth mouse movement injection with sub-pixel precision,
 * velocity tracking, and temporal spreading for seamless blending with
 * physical mouse passthrough.
 * 
 * Humanization features:
 * - FPU-optimized runtime tremor generation (no LUTs, no periodicity)
 * - Layered sine oscillators at physiological tremor frequencies (8-25Hz)
 * - Perpendicular jitter for realistic path scatter
 * - Variable thresholds and timing to avoid fingerprinting
 * - Overshoot/correction patterns for realism
 */

#include "smooth_injection.h"
#include "humanization_fpu.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
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
    struct active_queue_node *prev;  // O(1) doubly-linked list unlink
} active_queue_node_t;

static active_queue_node_t g_active_nodes[SMOOTH_QUEUE_SIZE];
static active_queue_node_t *g_active_head = NULL;
static uint64_t g_active_node_bitmap = 0;  // 64-bit bitmap for 64-entry queue

//--------------------------------------------------------------------+
// Fast PRNG for Humanization (xorshift32)
//--------------------------------------------------------------------+
static uint64_t g_free_bitmap = 0xFFFFFFFFFFFFFFFFULL;

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
// Easing Curves - FPU direct computation (no LUTs on M33)
//--------------------------------------------------------------------+

// Easing is now computed directly using apply_easing_fpu() inline
// M33 FPU makes direct computation faster than LUT lookups

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
                g_free_bitmap |= (1ULL << i);
            }
        }
        if (g_free_bitmap == 0) return NULL;
    }

    // Find first free slot using count trailing zeros (CTZ) - 64-bit
    int idx = __builtin_ctzll(g_free_bitmap);
    g_free_bitmap &= ~(1ULL << idx);
    
    g_smooth.queue_count++;
    
    // Add to active linked list for fast iteration - O(1) using CTZ
    if (g_active_node_bitmap == 0xFFFFFFFFFFFFFFFFULL) {
        // Shouldn't happen since we check queue_count, but handle gracefully
        return &g_smooth.queue[idx];
    }

    // O(1) node allocation using count trailing zeros - 64-bit
    int node_idx = __builtin_ctzll(~g_active_node_bitmap);
    g_active_node_bitmap |= (1ULL << node_idx);

    // Link node (doubly-linked for O(1) free)
    g_active_nodes[node_idx].index = idx;
    g_active_nodes[node_idx].next = g_active_head;
    g_active_nodes[node_idx].prev = NULL;
    if (g_active_head) {
        g_active_head->prev = &g_active_nodes[node_idx];
    }
    g_active_head = &g_active_nodes[node_idx];
    
    // Cache node index in queue entry for O(1) free (Fix #11)
    g_smooth.queue[idx].active_node_idx = (int8_t)node_idx;
    
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
            g_free_bitmap |= (1ULL << idx);

            // O(1) removal from active doubly-linked list
            int8_t node_idx = entry->active_node_idx;
            if (node_idx >= 0 && node_idx < SMOOTH_QUEUE_SIZE) {
                active_queue_node_t *target = &g_active_nodes[node_idx];

                // Direct O(1) unlink via prev/next pointers
                if (target->prev) {
                    target->prev->next = target->next;
                } else {
                    // target is the head
                    g_active_head = target->next;
                }
                if (target->next) {
                    target->next->prev = target->prev;
                }

                // Mark node as free in bitmap
                g_active_node_bitmap &= ~(1ULL << node_idx);
            }
            entry->active_node_idx = -1;
        }
    }
}

//--------------------------------------------------------------------+
// Public API Implementation
//--------------------------------------------------------------------+

void smooth_injection_init(void) {
    memset(&g_smooth, 0, sizeof(g_smooth));
    
    // Reset global state that's outside g_smooth structure
    g_free_bitmap = 0xFFFFFFFFFFFFFFFFULL;  // All 64 slots free
    g_velocity_sum_x_fp = 0;     // Reset velocity accumulators
    g_velocity_sum_y_fp = 0;
    
    // Initialize active linked list
    g_active_head = NULL;
    g_active_node_bitmap = 0;  // All nodes free
    
    // Seed RNG with hardware TRNG for true entropy (RP2350 has hardware TRNG)
    uint32_t hw_entropy = get_rand_32();
    rng_seed(hw_entropy);
    
    // Initialize FPU-based humanization
    humanization_fpu_init(hw_entropy);
    
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
                                  uint8_t delay_frames, bool is_first_substep) {
    smooth_queue_entry_t *entry = queue_alloc();
    if (!entry) {
        // Queue full - fall back to immediate injection
        g_smooth.x_accumulator_fp += x_fp;
        g_smooth.y_accumulator_fp += y_fp;
        return false;
    }
    
    // Fix #3: Apply delivery error (sensor noise simulation)
    // Small ±1-3% error that the autopilot's feedback loop absorbs naturally
    if (g_smooth.humanization.delivery_error_fp > 0) {
        int32_t error_scale = rng_range_fp(-g_smooth.humanization.delivery_error_fp,
                                            g_smooth.humanization.delivery_error_fp);
        x_fp += fp_mul(x_fp, error_scale);
        y_fp += fp_mul(y_fp, error_scale);
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
        
        // FPU inline frame spread calculation (no LUT)
        float movement_px = (float)max_component / SMOOTH_FP_ONE;
        float multiplier;
        if (movement_px < 20.0f)
            multiplier = 1.3f - (movement_px / 20.0f) * 0.1f;
        else if (movement_px < 60.0f)
            multiplier = 1.2f - ((movement_px - 20.0f) / 40.0f) * 0.2f;
        else if (movement_px < 100.0f)
            multiplier = 1.0f - ((movement_px - 60.0f) / 40.0f) * 0.2f;
        else if (movement_px < 140.0f)
            multiplier = 0.8f - ((movement_px - 100.0f) / 40.0f) * 0.1f;
        else
            multiplier = 0.7f;
        
        frames = (uint8_t)(frames * multiplier);
        if (frames < 1) frames = 1;
    }
    
    // Add natural latency jitter (0-1 frame)
    if (mode != INJECT_MODE_IMMEDIATE) {
        frames = frames + (uint8_t)rng_range(0, 1);
    }
    
    // Add inter-substep delay to stagger execution (sequential, not parallel)
    frames = frames + delay_frames;
    
    // Fix #1: Movement onset jitter - apply to first sub-step only
    // Shifts entire movement start in time to break timing correlation
    uint8_t onset_delay = 0;
    if (is_first_substep && g_smooth.humanization.onset_jitter_max > 0) {
        onset_delay = (uint8_t)rng_range(g_smooth.humanization.onset_jitter_min,
                                          g_smooth.humanization.onset_jitter_max);
    }
    
    // Fix #4: Fix easing curve selection bias - single RNG draw, correct distribution
    // Old code: two draws created biased distribution (33% ease-out, then 25% of remaining)
    // New code: one draw with explicit probability buckets
    easing_mode_t easing = EASING_LINEAR;
    if (max_component > int_to_fp(10)) {
        // Larger sub-steps: smooth easing
        easing = EASING_EASE_IN_OUT;
    } else {
        uint32_t r = rng_next() % 12;
        if (r < 4) {
            easing = EASING_EASE_OUT;      // 33% chance
        } else if (r < 7) {
            easing = EASING_EASE_IN_OUT;   // 25% chance
        }
        // else: 42% linear (default)
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
    
    // Fix #10: Cap velocity-matched frame count to 80 max (~640ms at 125Hz)
    // Prevents 255-frame movements that take >2 seconds and feel unresponsive
    if (frames > 80) frames = 80;
    
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
    entry->onset_delay = onset_delay;   // Fix #1: onset jitter
    entry->will_overshoot = false;
    entry->overshoot_x_fp = 0;
    entry->overshoot_y_fp = 0;
    // active_node_idx set by queue_alloc()
    
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
    // 4) Queue is less than 75% full (back-pressure protection)
    //    This prevents queue starvation under sustained high-rate Ferrum traffic
    bool should_subdivide = (g_smooth.humanization.mode == HUMANIZATION_FULL) &&
                            (movement_px >= SUBSTEP_MIN_MOVEMENT_PX) &&
                            (g_smooth.queue_count + SUBSTEP_COUNT_BASE <= SMOOTH_QUEUE_SIZE) &&
                            (g_smooth.queue_count < (SMOOTH_QUEUE_SIZE * 3 / 4));
    
    if (!should_subdivide) {
        // Fall back to single-entry path (legacy behavior)
        // Calculate overshoot for the single entry
        bool will_overshoot = false;
        int32_t overshoot_x_fp = 0, overshoot_y_fp = 0;
        if (g_smooth.humanization.jitter_enabled && 
            movement_px >= 15 && movement_px <= 120 && 
            (rng_next() % 100) < g_smooth.humanization.overshoot_chance) {
            will_overshoot = true;
            // FPU inline overshoot calculation: 2-5% of movement magnitude (human range)
            float magnitude_flt = (float)movement_px;
            float overshoot_pct = 0.02f + (magnitude_flt / 300.0f) * 0.03f;  // 2-5%
            float overshoot_px = fmaxf(0.5f, magnitude_flt * overshoot_pct);  // Floor 0.5px
            int32_t overshoot_base = (int32_t)(overshoot_px * SMOOTH_FP_ONE);
            int32_t overshoot_mag = fp_mul(overshoot_base, g_smooth.humanization.overshoot_max_fp);
            if (overshoot_mag > g_smooth.humanization.overshoot_max_fp) {
                overshoot_mag = g_smooth.humanization.overshoot_max_fp;
            }
            overshoot_x_fp = (x_fp > 0) ? overshoot_mag : ((x_fp < 0) ? -overshoot_mag : 0);
            overshoot_y_fp = (y_fp > 0) ? overshoot_mag : ((y_fp < 0) ? -overshoot_mag : 0);
        }
        
        bool ok = queue_single_substep(x_fp, y_fp, mode, 0, true);
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
    
    // Determine number of sub-steps (only reached in FULL mode)
    uint8_t num_substeps = SUBSTEP_COUNT_BASE + (uint8_t)rng_range(0, 3);  // 4-7 sub-steps
    
    // Clamp to available queue space
    uint8_t available = SMOOTH_QUEUE_SIZE - g_smooth.queue_count;
    if (num_substeps > available) {
        num_substeps = available;
    }
    if (num_substeps < 2) {
        // Not enough space, fall back to single entry
        bool ok = queue_single_substep(x_fp, y_fp, mode, 0, true);
        g_smooth.total_injected++;
        return ok;
    }
    
    // Generate randomized split ratios that sum to SMOOTH_FP_ONE
    // Each sub-step gets roughly 1/N of the total, ±30% jitter
    int32_t ratios[8];  // Max 8 sub-steps
    int32_t base_ratio = SMOOTH_FP_ONE / num_substeps;
    int32_t jitter_range = base_ratio * 3 / 10;  // ±30% of base
    int32_t ratio_sum = 0;
    int32_t min_ratio = SMOOTH_FP_ONE / (num_substeps * 4);  // Floor: 25% of equal share
    
    for (uint8_t i = 0; i < num_substeps - 1; i++) {
        int32_t r = base_ratio + rng_range_fp(-jitter_range, jitter_range);
        if (r < min_ratio) {
            r = min_ratio;
        }
        // Fix #9: Clamp running sum so remainder stays positive
        // If adding this ratio would leave less than min_ratio for remaining steps,
        // scale it down to preserve headroom
        int32_t remaining_steps = num_substeps - 1 - i;
        int32_t max_allowed = SMOOTH_FP_ONE - ratio_sum - (remaining_steps * min_ratio);
        if (r > max_allowed && max_allowed > min_ratio) {
            r = max_allowed;
        }
        ratios[i] = r;
        ratio_sum += r;
    }
    // Final sub-step gets the exact remainder (prevents drift)
    ratios[num_substeps - 1] = SMOOTH_FP_ONE - ratio_sum;
    if (ratios[num_substeps - 1] < min_ratio) {
        // Safety: if jitter still pushed us over, normalize all ratios
        ratio_sum = 0;
        for (uint8_t i = 0; i < num_substeps; i++) {
            ratios[i] = base_ratio;
            ratio_sum += base_ratio;
        }
        ratios[num_substeps - 1] += (SMOOTH_FP_ONE - ratio_sum);
    }
    
    // Calculate overshoot (apply only to the final sub-step)
    bool will_overshoot = false;
    int32_t overshoot_x_fp = 0, overshoot_y_fp = 0;
    if (g_smooth.humanization.jitter_enabled && 
        movement_px >= 15 && movement_px <= 120 && 
        (rng_next() % 100) < g_smooth.humanization.overshoot_chance) {
        will_overshoot = true;
        // FPU inline overshoot calculation: 2-5% of movement magnitude (human range)
        float magnitude_flt = (float)movement_px;
        float overshoot_pct = 0.02f + (magnitude_flt / 300.0f) * 0.03f;  // 2-5%
        float overshoot_px = fmaxf(0.5f, magnitude_flt * overshoot_pct);  // Floor 0.5px
        int32_t overshoot_base = (int32_t)(overshoot_px * SMOOTH_FP_ONE);
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
        
        bool ok = queue_single_substep(step_x, step_y, mode, cumulative_delay, (i == 0));
        
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
    
    // Per-frame overshoot correction counter (reset each frame)
    uint8_t corrections_this_frame = 0;
    
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
        g_free_bitmap = 0xFFFFFFFFFFFFFFFFULL;
        g_active_node_bitmap = 0ULL;
        goto apply_accumulator;
    }
    
    // Process active entries using linked list (O(n) where n = active count, not queue size)
    // Safety: limit iterations to prevent infinite loop from corrupted linked list
    active_queue_node_t *node = g_active_head;
    uint8_t iteration_limit = SMOOTH_QUEUE_SIZE + 4;  // Allow slightly more than max for overshoot corrections
    while (node && iteration_limit > 0) {
        iteration_limit--;
        smooth_queue_entry_t *entry = &g_smooth.queue[node->index];
        active_queue_node_t *next_node = node->next;  // Save next before potential free
        
        // Fix #1: Onset jitter - wait before starting delivery
        if (entry->onset_delay > 0) {
            entry->onset_delay--;
            node = next_node;
            continue;
        }
        
        if (entry->frames_left > 0) {
            // Calculate progress using FPU (faster than LUT on M33)
            // Offset by +1 so the first frame (frames_elapsed=0) produces
            // non-zero output.  Without this, progress_delta is always 0 on
            // the first frame, which means no HID report is sent when the
            // physical mouse is idle — the cursor won't move until frame 2.
            uint8_t frames_elapsed = entry->total_frames - entry->frames_left;
            float progress_flt = (float)(frames_elapsed + 1) / (float)(entry->total_frames + 1);
            float prev_progress = (float)frames_elapsed / (float)(entry->total_frames + 1);
            
            // Apply easing curve using FPU (3 cycles for multiply on M33)
            float eased_progress = apply_easing_fpu(progress_flt, entry->easing);
            float prev_eased = apply_easing_fpu(prev_progress, entry->easing);
            float progress_delta_flt = eased_progress - prev_eased;
            
            // Convert to fixed-point for accumulator
            int32_t progress_delta = (int32_t)(progress_delta_flt * SMOOTH_FP_ONE);
            
            // === MOVEMENT DELTA (tracked - affects remaining) ===
            int32_t movement_dx_fp = fp_mul(entry->x_fp, progress_delta);
            int32_t movement_dy_fp = fp_mul(entry->y_fp, progress_delta);
            
            // Add movement to frame output
            frame_x_fp += movement_dx_fp;
            frame_y_fp += movement_dy_fp;
            
            // Update remaining — queue tracks only intentional eased movement.
            // All tremor/noise is applied at the output stage by
            // apply_output_humanization(), so no noise accounting here.
            entry->x_remaining_fp -= movement_dx_fp;
            entry->y_remaining_fp -= movement_dy_fp;
            entry->frames_left--;
            
            // Check if done
            if (entry->frames_left == 0) {
                // Add any remaining fractional movement
                frame_x_fp += entry->x_remaining_fp;
                frame_y_fp += entry->y_remaining_fp;
                
                // Inject overshoot if planned
                // Safety: limit corrections per frame and skip if queue is nearly full
                // to prevent queue explosion under high command rates
                if (entry->will_overshoot && 
                    corrections_this_frame < 2 &&
                    g_smooth.queue_count < SMOOTH_QUEUE_SIZE - 4) {
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
                        corrections_this_frame++;
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
    
    // Safety: if we hit the iteration limit, the linked list may be corrupted
    // Reset the queue to recover from the hang state
    if (iteration_limit == 0 && node != NULL) {
        for (int i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
            g_smooth.queue[i].active = false;
        }
        g_smooth.queue_count = 0;
        g_free_bitmap = 0xFFFFFFFFFFFFFFFFULL;
        g_active_head = NULL;
        g_active_node_bitmap = 0ULL;
        frame_x_fp = 0;
        frame_y_fp = 0;
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
    
    // Fix #13: Mode-dependent accumulator clamp
    // OFF = no clamp (unlimited), LOW = ±4px, MED = ±3px, HIGH = ±2px
    const int32_t max_accum = g_smooth.humanization.accum_clamp_fp;
    if (max_accum > 0) {
        if (g_smooth.x_accumulator_fp > max_accum) g_smooth.x_accumulator_fp = max_accum;
        else if (g_smooth.x_accumulator_fp < -max_accum) g_smooth.x_accumulator_fp = -max_accum;
        if (g_smooth.y_accumulator_fp > max_accum) g_smooth.y_accumulator_fp = max_accum;
        else if (g_smooth.y_accumulator_fp < -max_accum) g_smooth.y_accumulator_fp = -max_accum;
    }
    
    // Output
    *out_x = clamp_i8(out_x_int);
    *out_y = clamp_i8(out_y_int);
    
    // Fix: When queue is fully drained and output rounds to zero, flush
    // sub-pixel accumulator residuals. Without this, tiny residuals (<1px)
    // keep smooth_has_pending() returning true forever, which causes the
    // output humanization to keep applying tremor → infinite mouse shaking.
    if (g_smooth.queue_count == 0 && out_x_int == 0 && out_y_int == 0) {
        g_smooth.x_accumulator_fp = 0;
        g_smooth.y_accumulator_fp = 0;
    }
    
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
    g_free_bitmap = 0xFFFFFFFFFFFFFFFFULL;
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
    if (mode >= HUMANIZATION_MODE_COUNT) mode = HUMANIZATION_FULL;
    
    humanization_mode_t old_mode = g_smooth.humanization.mode;
    g_smooth.humanization.mode = mode;
    
    // Fix #12: Reseed RNG from hardware TRNG on mode change for fresh per-session randomization
    rng_seed(get_rand_32());
    
    switch (mode) {
        case HUMANIZATION_OFF:
            // Disable all humanization - pure digital pass-through
            g_smooth.max_per_frame = 16;                                // Fix #2: fixed
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = false;
            g_smooth.humanization.jitter_amount_fp = 0;
            g_smooth.humanization.overshoot_chance = 0;
            g_smooth.humanization.overshoot_max_fp = 0;
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(2);
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(10);
            g_smooth.humanization.delivery_error_fp = 0;                // Fix #3: no error
            g_smooth.humanization.accum_clamp_fp = 0;                   // Fix #13: no clamp (unlimited)
            g_smooth.humanization.onset_jitter_min = 0;                 // Fix #1: no onset delay
            g_smooth.humanization.onset_jitter_max = 0;
            break;
            
        case HUMANIZATION_MICRO:
            // Micro-noise only — for pre-humanized input
            // Only adds sub-pixel tremor + sensor noise below the PC's correction threshold
            g_smooth.max_per_frame = 16;                                // Fixed — don't alter delivery rate
            g_smooth.velocity_matching_enabled = false;                 // Input already has natural velocity
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(1) / 2;  // 0.5px base tremor
            g_smooth.humanization.overshoot_chance = 0;                 // No overshoot
            g_smooth.humanization.overshoot_max_fp = 0;
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(2);
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(10);
            g_smooth.humanization.delivery_error_fp = SMOOTH_FP_ONE / 100;  // ±1% sensor noise
            g_smooth.humanization.accum_clamp_fp = int_to_fp(8);        // ±8px — generous clamp prevents
                                                                        // unbounded drift from delivery error
                                                                        // residuals while trusting input
            g_smooth.humanization.onset_jitter_min = 0;                 // No onset delay
            g_smooth.humanization.onset_jitter_max = 0;
            break;

        case HUMANIZATION_FULL:
            // Full humanization — for raw/robotic input
            // Subdivision, easing, onset delay, overshoot — the works
            g_smooth.max_per_frame = (int16_t)rng_range(12, 20);        // Per-session variation
            g_smooth.velocity_matching_enabled = true;
            g_smooth.humanization.jitter_enabled = true;
            g_smooth.humanization.jitter_amount_fp = int_to_fp(1);      // 1.0px base tremor
            g_smooth.humanization.overshoot_chance = 10;                // 10% chance
            g_smooth.humanization.overshoot_max_fp = int_to_fp(2);      // max 2px
            g_smooth.humanization.vel_slow_threshold_fp = int_to_fp(rng_range(1, 4));
            g_smooth.humanization.vel_fast_threshold_fp = int_to_fp(rng_range(8, 14));
            g_smooth.humanization.delivery_error_fp = SMOOTH_FP_ONE / 50;  // ±2%
            g_smooth.humanization.accum_clamp_fp = int_to_fp(16);       // ±16px — matches max_per_frame,
                                                                        // lets accumulator drain naturally
                                                                        // when queue overflow dumps to accum
            g_smooth.humanization.onset_jitter_min = 1;                 // 1-4 frames
            g_smooth.humanization.onset_jitter_max = 4;
            break;
            
        default:
            smooth_set_humanization_mode_internal(HUMANIZATION_FULL, auto_save);
            return;
    }
    
    // NOTE: Runtime flash saves are DISABLED to prevent device hangs.
    // flash_safe_execute() pauses Core1 (USB host) via multicore_lockout
    // during the ~100ms flash erase/program, which causes the USB host
    // stack to miss timing-critical operations and hang the device.
    // Humanization mode will reset to default (FULL) on reboot.
    // TODO: Re-enable when a safe async flash write mechanism is available.
    (void)auto_save;
    (void)old_mode;
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

    // Increased timeout from 500ms to 2000ms to handle busy Core1
    // Core1 may be processing intensive USB host operations
    int result = flash_safe_execute(flash_write_callback, &param, 2000);
    if (result != PICO_OK) {
        // flash_safe_execute handles all the multicore coordination;
        // if it fails, increment our failure counter
        g_flash_save_failures++;
    }
}

void smooth_save_humanization_mode(void) {
    // Runtime flash saves are DISABLED — see comment in smooth_set_humanization_mode_internal.
    // This prevents device hangs caused by multicore_lockout pausing the USB host core.
}

// Core1 acknowledgment flag — kept for backward compat but flash_safe_execute
// handles multicore coordination automatically now.
extern volatile bool g_core1_flash_acknowledged;

// Call this periodically from main loop.
// Runtime flash saves are DISABLED to prevent device hangs.
// See comment in smooth_set_humanization_mode_internal for details.
void smooth_process_deferred_save(void) {
    // No-op: flash saves disabled to prevent multicore_lockout hanging the USB host core.
    // Humanization mode persists in RAM only; resets to default on reboot.
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
        smooth_set_humanization_mode_internal(HUMANIZATION_FULL, false);
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

void smooth_get_humanization_params(int32_t *jitter_amount_fp, bool *jitter_enabled) {
    if (jitter_amount_fp) {
        *jitter_amount_fp = g_smooth.humanization.jitter_amount_fp;
    }
    if (jitter_enabled) {
        *jitter_enabled = g_smooth.humanization.jitter_enabled;
    }
}
