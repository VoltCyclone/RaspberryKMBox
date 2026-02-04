/*
 * Precomputed Lookup Tables for Humanization
 * 
 * These tables eliminate runtime math operations for critical path
 * humanization features, providing smoother and faster injection.
 * 
 * Tables included:
 * - Easing curves (ease-in-out cubic, ease-out quad)
 * - Progress fractions for common frame counts
 * - Pre-generated jitter patterns
 * - Sine wave approximation for smooth micro-tremor
 */

#ifndef HUMANIZATION_LUT_H
#define HUMANIZATION_LUT_H

#include <stdint.h>
#include "smooth_injection.h"

//--------------------------------------------------------------------+
// Configuration
//--------------------------------------------------------------------+

// Easing table resolution (256 entries = 8-bit precision)
#define EASING_LUT_SIZE         256
#define EASING_LUT_SHIFT        8   // log2(256)

// Jitter pattern buffer size (cycles through for variety)
#define JITTER_LUT_SIZE         64

// Sine table for micro-tremor (one quadrant, mirrored)
#define SINE_LUT_SIZE           64

// Maximum frames we precompute progress fractions for
#define MAX_PRECOMPUTED_FRAMES  32

//--------------------------------------------------------------------+
// Easing Lookup Tables (16.16 fixed-point output)
//--------------------------------------------------------------------+

// Ease-in-out cubic: slow start, fast middle, slow end
// Input: index 0-255 (t = index/255)
// Output: eased value in 16.16 fixed-point
extern const int32_t g_ease_in_out_cubic_lut[EASING_LUT_SIZE];

// Ease-out quadratic: fast start, slow end (for corrections)
extern const int32_t g_ease_out_quad_lut[EASING_LUT_SIZE];

// Linear (identity, for completeness)
extern const int32_t g_ease_linear_lut[EASING_LUT_SIZE];

//--------------------------------------------------------------------+
// Progress Fraction Tables
// 
// For a movement spread over N frames, progress[frame] = frame/N
// Pre-computed to avoid division in hot path
//--------------------------------------------------------------------+

// Progress fractions: g_progress_lut[total_frames][current_frame]
// Returns progress in 16.16 fixed-point (0 to SMOOTH_FP_ONE)
// For total_frames > MAX_PRECOMPUTED_FRAMES, fall back to runtime calc
extern const int32_t g_progress_lut[MAX_PRECOMPUTED_FRAMES + 1][MAX_PRECOMPUTED_FRAMES + 1];

//--------------------------------------------------------------------+
// Jitter Pattern Tables
//
// Pre-generated "random" jitter values that cycle for natural variation
// Uses multiple patterns to avoid obvious repetition
//--------------------------------------------------------------------+

// Jitter X offsets (in 16.16 fixed-point, range approx ±1.0)
extern const int32_t g_jitter_x_lut[JITTER_LUT_SIZE];

// Jitter Y offsets (different pattern than X for 2D variation)
extern const int32_t g_jitter_y_lut[JITTER_LUT_SIZE];

// Jitter application mask (determines which frames get jitter)
// Bit pattern: 1 = apply jitter, 0 = skip
// Split into lo/hi 32-bit halves for RP2040 efficiency (no native 64-bit ops)
extern const uint32_t g_jitter_mask_lo_lut[4];  // Bits 0-31
extern const uint32_t g_jitter_mask_hi_lut[4];  // Bits 32-63

//--------------------------------------------------------------------+
// Sine Table for Micro-Tremor
//
// Quarter-wave sine for smooth oscillation simulation
// Full wave: sin(0..2π) constructed by mirroring
//--------------------------------------------------------------------+

// Sine values for 0 to π/2 (first quadrant)
// Output in 16.16 fixed-point, range [0, SMOOTH_FP_ONE]
extern const int32_t g_sine_quarter_lut[SINE_LUT_SIZE];

//--------------------------------------------------------------------+
// Overshoot Magnitude Table
//
// Pre-calculated overshoot amounts indexed by movement magnitude
// Avoids runtime random range calculations
//--------------------------------------------------------------------+

#define OVERSHOOT_LUT_SIZE      16

// Overshoot amounts for movement magnitudes (index = magnitude/8 pixels)
// Output in 16.16 fixed-point
extern const int32_t g_overshoot_lut[OVERSHOOT_LUT_SIZE];

//--------------------------------------------------------------------+
// Acceleration/Deceleration Curve Table
//
// Natural movement starts slow, accelerates, then decelerates
// This table gives the velocity multiplier at each phase of movement
// Useful for natural movement starts/stops
//--------------------------------------------------------------------+

#define ACCEL_LUT_SIZE          32

// Velocity curve: index 0 = start, index 31 = end
// Output in 16.16 fixed-point (0 to SMOOTH_FP_ONE)
// Profile: slow start, fast middle (~1.2x), slow end
extern const int32_t g_accel_curve_lut[ACCEL_LUT_SIZE];

//--------------------------------------------------------------------+
// Sub-Pixel Accumulation Dither Pattern
//
// When accumulating fractional pixels across frames, systematic
// patterns can be detected. This table provides pseudo-random
// offsets to break up patterns while maintaining accuracy over time.
//--------------------------------------------------------------------+

#define SUBPIXEL_DITHER_SIZE    16

// Dither offsets for sub-pixel accumulation (in 16.16 fixed-point)
// Values sum to ~0 over the full cycle to avoid drift
extern const int32_t g_subpixel_dither_lut[SUBPIXEL_DITHER_SIZE];

//--------------------------------------------------------------------+
// Movement-Type Jitter Scale LUT
//
// Human hands have different tremor characteristics based on movement type:
// - Small adjustments (0-20px): High tremor (stationary hand, precise)
// - Medium moves (20-100px): Moderate tremor (controlled movement)
// - Large flicks (100-200px): Low tremor (intentional, tremor suppressed)
// - Very large (200+px): Minimal tremor (fast, ballistic movement)
//
// Index = movement_magnitude / 10 (clamped to 0-31)
// Output: jitter scale multiplier in 16.16 fixed-point
//--------------------------------------------------------------------+

#define JITTER_SCALE_LUT_SIZE   32

// Jitter scale by movement size (decreases with larger movements)
// Range: 1.5x (small) to 0.1x (large)
extern const int32_t g_jitter_scale_by_movement_lut[JITTER_SCALE_LUT_SIZE];

//--------------------------------------------------------------------+
// Movement-Type Frame Spread LUT
//
// Different movement types need different timing:
// - Small: Spread over more frames (careful, slow)
// - Medium: Balanced spread
// - Large: Faster spread (ballistic, quick)
//
// Index = movement_magnitude / 10 (clamped to 0-31)
// Output: frame multiplier in 16.16 fixed-point
//--------------------------------------------------------------------+

#define FRAME_SPREAD_LUT_SIZE   32

// Frame spread multiplier by movement size
// Range: 1.3x (small, spread out) to 0.7x (large, compress)
extern const int32_t g_frame_spread_by_movement_lut[FRAME_SPREAD_LUT_SIZE];

//--------------------------------------------------------------------+
// Fast Lookup Functions (inline for zero overhead)
//--------------------------------------------------------------------+

/**
 * Fast easing lookup with interpolation
 * @param t Progress in 16.16 fixed-point [0, SMOOTH_FP_ONE]
 * @param mode Easing mode
 * @return Eased progress in 16.16 fixed-point
 */
static inline int32_t lut_apply_easing(int32_t t, easing_mode_t mode) {
    // Clamp t to valid range
    if (t <= 0) return 0;
    if (t >= SMOOTH_FP_ONE) return SMOOTH_FP_ONE;
    
    // Convert t to table index (0-255)
    // t is 16.16, we want 8-bit index
    uint32_t index = (uint32_t)t >> (SMOOTH_FP_SHIFT - EASING_LUT_SHIFT);
    if (index >= EASING_LUT_SIZE - 1) index = EASING_LUT_SIZE - 2;
    
    // Select table based on mode
    const int32_t *table;
    switch (mode) {
        case EASING_LINEAR:      table = g_ease_linear_lut; break;
        case EASING_EASE_IN_OUT: table = g_ease_in_out_cubic_lut; break;
        case EASING_EASE_OUT:    table = g_ease_out_quad_lut; break;
        default:                 table = g_ease_linear_lut; break;  // Fallback for safety
    }
    
    // Linear interpolation between table entries for smoothness
    int32_t v0 = table[index];
    int32_t v1 = table[index + 1];
    
    // Fractional part for interpolation (lower 8 bits of shifted t)
    uint32_t frac = ((uint32_t)t >> (SMOOTH_FP_SHIFT - EASING_LUT_SHIFT - 8)) & 0xFF;
    
    // Interpolate: v0 + (v1 - v0) * frac / 256
    return v0 + (((v1 - v0) * (int32_t)frac) >> 8);
}

/**
 * Fast progress lookup (no division needed)
 * @param total_frames Total frames for movement
 * @param current_frame Current frame (0 to total_frames-1)
 * @return Progress in 16.16 fixed-point
 */
static inline int32_t lut_get_progress(uint8_t total_frames, uint8_t current_frame) {
    if (total_frames == 0) return SMOOTH_FP_ONE;
    if (total_frames <= MAX_PRECOMPUTED_FRAMES && current_frame <= total_frames) {
        return g_progress_lut[total_frames][current_frame];
    }
    // Fallback for large frame counts (rare)
    return ((int32_t)current_frame << SMOOTH_FP_SHIFT) / total_frames;
}

/**
 * Fast jitter lookup (cycles through pre-generated patterns)
 * @param frame_counter Global frame counter
 * @param jitter_scale Multiplier for jitter amount (16.16 fixed-point)
 * @param out_x Output X jitter
 * @param out_y Output Y jitter
 */
static inline void lut_get_jitter(uint32_t frame_counter, int32_t jitter_scale,
                                   int32_t *out_x, int32_t *out_y) {
    // Select pattern and check if jitter applies this frame
    // Uses split 32-bit masks for RP2040 efficiency (no native 64-bit ops)
    uint32_t pattern_idx = (frame_counter >> 6) & 0x3;  // Change pattern every 64 frames
    uint32_t bit_idx = frame_counter & 63;
    
    // Select lo or hi mask based on bit position
    uint32_t mask = (bit_idx < 32) ? g_jitter_mask_lo_lut[pattern_idx] 
                                   : g_jitter_mask_hi_lut[pattern_idx];
    
    if (!((mask >> (bit_idx & 31)) & 1)) {
        // No jitter this frame
        *out_x = 0;
        *out_y = 0;
        return;
    }
    
    uint32_t lut_idx = frame_counter & (JITTER_LUT_SIZE - 1);
    
    // Scale and return jitter using 32-bit math
    // Jitter LUT values are in range ±3277 (fits in 16 bits signed)
    // jitter_scale is typically < 65536
    // Scale down jitter_scale to 8.8 fixed point for 32-bit safe multiply
    int32_t jx_raw = g_jitter_x_lut[lut_idx];  // ±3277 max
    int32_t jy_raw = g_jitter_y_lut[lut_idx];
    
    int32_t scale_8_8 = jitter_scale >> 8;  // Convert to 8.8 fixed point
    
    // 16-bit * 16-bit = 32-bit, safe on all platforms
    *out_x = (jx_raw * scale_8_8) >> 8;  // Result back to 16.16
    *out_y = (jy_raw * scale_8_8) >> 8;
}

/**
 * Get jitter scale based on movement magnitude
 * Small movements = more jitter (1.5x), large = less jitter (0.1x)
 * 
 * @param movement_magnitude Movement size in pixels
 * @return Jitter scale multiplier in 16.16 fixed-point
 */
static inline int32_t lut_get_jitter_scale_for_movement(int32_t movement_magnitude) {
    // Convert magnitude to LUT index (0-31)
    uint32_t idx = (uint32_t)movement_magnitude / 10;
    if (idx >= JITTER_SCALE_LUT_SIZE) idx = JITTER_SCALE_LUT_SIZE - 1;
    return g_jitter_scale_by_movement_lut[idx];
}

/**
 * Get frame spread multiplier based on movement magnitude
 * Small movements = slower (1.3x frames), large = faster (0.7x frames)
 * 
 * @param movement_magnitude Movement size in pixels
 * @return Frame multiplier in 16.16 fixed-point
 */
static inline int32_t lut_get_frame_spread_for_movement(int32_t movement_magnitude) {
    // Convert magnitude to LUT index (0-31)
    uint32_t idx = (uint32_t)movement_magnitude / 10;
    if (idx >= FRAME_SPREAD_LUT_SIZE) idx = FRAME_SPREAD_LUT_SIZE - 1;
    return g_frame_spread_by_movement_lut[idx];
}

/**
 * Get micro-tremor offset using sine approximation
 * Creates smooth oscillating "hand shake" effect
 * Human hand tremor naturally decreases during fast/intentional movements
 * 
 * @param time_ms Current time in milliseconds
 * @param amplitude Base tremor amplitude in 16.16 fixed-point
 * @param velocity_scale Velocity-based scaling: SMOOTH_FP_ONE = stationary (full tremor),
 *                       0 = fast motion (no tremor). Simulates focus/intent suppression.
 * @param out_x Output X tremor
 * @param out_y Output Y tremor
 */
static inline void lut_get_tremor(uint32_t time_ms, int32_t amplitude,
                                   int32_t velocity_scale,
                                   int32_t *out_x, int32_t *out_y) {
    // X tremor: ~7Hz oscillation (period ~143ms, using 128ms for power-of-2)
    // Y tremor: ~11.7Hz oscillation (period ~85ms for natural 2D Lissajous motion)
    
    // Phase for X: 256 / 2 = 128ms period → ~7.8Hz
    uint32_t phase_x = (time_ms << 1) & ((SINE_LUT_SIZE * 4) - 1);  // 0-255 range
    
    // Phase for Y: 256 / 3 ≈ 85ms period → ~11.7Hz
    // Note: No shift - multiply by 3 directly for correct frequency
    uint32_t phase_y = (time_ms * 3) & ((SINE_LUT_SIZE * 4) - 1);
    
    // Convert phase to quadrant and index
    uint32_t quadrant_x = phase_x >> 6;  // 0-3
    uint32_t idx_x = phase_x & (SINE_LUT_SIZE - 1);
    
    uint32_t quadrant_y = phase_y >> 6;
    uint32_t idx_y = phase_y & (SINE_LUT_SIZE - 1);
    
    // Mirror index for quadrants 1 and 3
    if (quadrant_x & 1) idx_x = SINE_LUT_SIZE - 1 - idx_x;
    if (quadrant_y & 1) idx_y = SINE_LUT_SIZE - 1 - idx_y;
    
    int32_t sin_x = g_sine_quarter_lut[idx_x];
    int32_t sin_y = g_sine_quarter_lut[idx_y];
    
    // Negate for quadrants 2 and 3
    if (quadrant_x >= 2) sin_x = -sin_x;
    if (quadrant_y >= 2) sin_y = -sin_y;
    
    // Apply velocity-based scaling (tremor suppression during fast movement)
    // Then scale by base amplitude
    int32_t scaled_amplitude = ((int64_t)amplitude * velocity_scale) >> SMOOTH_FP_SHIFT;
    
    *out_x = ((int64_t)sin_x * scaled_amplitude) >> SMOOTH_FP_SHIFT;
    *out_y = ((int64_t)sin_y * scaled_amplitude) >> SMOOTH_FP_SHIFT;
}

/**
 * Get pre-computed overshoot amount based on movement magnitude
 * @param magnitude Movement magnitude in pixels (integer)
 * @return Overshoot amount in 16.16 fixed-point
 */
static inline int32_t lut_get_overshoot(int32_t magnitude) {
    if (magnitude < 0) magnitude = -magnitude;
    uint32_t idx = (uint32_t)magnitude >> 3;  // Divide by 8
    if (idx >= OVERSHOOT_LUT_SIZE) idx = OVERSHOOT_LUT_SIZE - 1;
    return g_overshoot_lut[idx];
}

/**
 * Get velocity multiplier for natural acceleration/deceleration
 * @param progress Movement progress in 16.16 fixed-point [0, SMOOTH_FP_ONE]
 * @return Velocity multiplier in 16.16 fixed-point
 */
static inline int32_t lut_get_accel_curve(int32_t progress) {
    if (progress <= 0) return g_accel_curve_lut[0];
    if (progress >= SMOOTH_FP_ONE) return g_accel_curve_lut[ACCEL_LUT_SIZE - 1];
    
    // Convert progress to table index
    uint32_t idx = ((uint32_t)progress * (ACCEL_LUT_SIZE - 1)) >> SMOOTH_FP_SHIFT;
    if (idx >= ACCEL_LUT_SIZE - 1) idx = ACCEL_LUT_SIZE - 2;
    
    // Linear interpolation
    int32_t v0 = g_accel_curve_lut[idx];
    int32_t v1 = g_accel_curve_lut[idx + 1];
    
    // Fractional part for interpolation
    uint32_t frac_bits = SMOOTH_FP_SHIFT - 5;  // 5 = log2(32)
    uint32_t frac = ((uint32_t)progress >> (frac_bits - 8)) & 0xFF;
    
    return v0 + (((v1 - v0) * (int32_t)frac) >> 8);
}

/**
 * Get sub-pixel dither offset to break up accumulation patterns
 * @param frame_counter Current frame number
 * @return Dither offset in 16.16 fixed-point (small value ±0.25 pixels)
 */
static inline int32_t lut_get_subpixel_dither(uint32_t frame_counter) {
    return g_subpixel_dither_lut[frame_counter & (SUBPIXEL_DITHER_SIZE - 1)];
}

/**
 * Calculate velocity scale for tremor suppression
 * Maps velocity magnitude to suppression factor
 * @param velocity_mag Velocity magnitude in 16.16 fixed-point (pixels/frame)
 * @return Scale factor: SMOOTH_FP_ONE at rest, approaches 0 at high velocity
 */
static inline int32_t lut_calc_tremor_suppression(int32_t velocity_mag) {
    // Tremor fully present below 2 pixels/frame
    // Tremor suppressed above 8 pixels/frame
    // Linear ramp between
    const int32_t LOW_THRESH = 2 << SMOOTH_FP_SHIFT;   // 2.0 in 16.16
    const int32_t HIGH_THRESH = 8 << SMOOTH_FP_SHIFT;  // 8.0 in 16.16
    
    if (velocity_mag < 0) velocity_mag = -velocity_mag;
    
    if (velocity_mag <= LOW_THRESH) {
        return SMOOTH_FP_ONE;  // Full tremor
    }
    if (velocity_mag >= HIGH_THRESH) {
        return SMOOTH_FP_ONE >> 3;  // ~12.5% tremor (not zero, subtle shake remains)
    }
    
    // Linear interpolation between thresholds
    int32_t range = HIGH_THRESH - LOW_THRESH;
    int32_t pos = velocity_mag - LOW_THRESH;
    int32_t suppression = SMOOTH_FP_ONE - ((pos * (SMOOTH_FP_ONE - (SMOOTH_FP_ONE >> 3))) / range);
    
    return suppression;
}

#endif // HUMANIZATION_LUT_H
