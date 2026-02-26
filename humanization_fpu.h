/*
 * FPU-Optimized Humanization for RP2350 Cortex-M33
 * 
 * Takes advantage of single-precision FPU instead of wasteful LUT approach.
 * Provides non-periodic tremor simulation using layered sine oscillators.
 */

#ifndef HUMANIZATION_FPU_H
#define HUMANIZATION_FPU_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//--------------------------------------------------------------------+
// Runtime Tremor Generator (No LUTs, No Periodicity)
//--------------------------------------------------------------------+

/**
 * Initialize humanization RNG with hardware entropy
 */
void humanization_fpu_init(uint32_t seed);

/**
 * Generate next jitter sample (runtime, non-periodic)
 * 
 * Combines layered sine oscillators at physiological tremor frequencies
 * (8-25Hz band) with LFSR noise to eliminate detectable periodicity.
 * 
 * @param scale Jitter magnitude multiplier
 * @param perp_x Output perpendicular X component
 * @param perp_y Output perpendicular Y component
 */
void humanization_get_tremor(float scale, float *perp_x, float *perp_y);

/**
 * Get movement-dependent jitter scale
 * Piecewise linear function (no LUT)
 * 
 * @param magnitude Movement magnitude in pixels
 * @return Jitter scale multiplier
 */
static inline float humanization_jitter_scale(float magnitude) {
    // Small movements: more visible tremor
    if (magnitude < 20.0f) return 1.8f;
    // Medium: gradual reduction
    if (magnitude < 60.0f) return 1.8f - (magnitude - 20.0f) * 0.015f;  // → 1.2
    // Large: suppressed tremor
    if (magnitude < 120.0f) return 1.2f - (magnitude - 60.0f) * 0.0083f; // → 0.7
    // Very large: floor at 0.4x
    return fmaxf(0.4f, 0.7f - (magnitude - 120.0f) * 0.0015f);
}

/**
 * Minimum-jerk velocity profile
 * Natural acceleration/deceleration curve
 */
static inline float min_jerk_velocity(float t) {
    float one_minus_t = 1.0f - t;
    return 30.0f * t * t * one_minus_t * one_minus_t;
}

#endif // HUMANIZATION_FPU_H
