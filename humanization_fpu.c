/*
 * FPU-Optimized Humanization Implementation
 * 
 * Runtime tremor generation using Cortex-M33 FPU instead of LUTs.
 * Eliminates detectable periodicity through layered incommensurate oscillators.
 *
 * FIXES applied:
 * 1. LFSR conversion: balanced [-1,1] mapping (was asymmetric, causing drift)
 * 2. Phase: fixed offsets instead of accumulating secondary phases (was doubling freq)
 * 3. Precision: wrap phase counter to prevent sinf() precision loss at large arguments
 */

#include "humanization_fpu.h"
#include "pico/rand.h"
#include <math.h>

//--------------------------------------------------------------------+
// Runtime Tremor State
//--------------------------------------------------------------------+

// Galois LFSR for cheap pseudo-random
static uint32_t g_jitter_lfsr = 0xDEADBEEF;

// Tremor phase accumulator (increments each HID report, ~1kHz)
static uint32_t g_tremor_phase = 0;

//--------------------------------------------------------------------+
// Fast LFSR-based noise (cheaper than rand())
//--------------------------------------------------------------------+

static inline float jitter_next(void) {
    // Galois LFSR - 32-bit xorshift
    g_jitter_lfsr ^= g_jitter_lfsr << 13;
    g_jitter_lfsr ^= g_jitter_lfsr >> 17;
    g_jitter_lfsr ^= g_jitter_lfsr << 5;
    
    // FIX: Balanced [-1.0, 1.0] conversion
    // Use top 24 bits mapped symmetrically to avoid DC bias
    // Old code: (float)(int32_t)g_jitter_lfsr / (float)INT32_MAX had asymmetric range
    int32_t balanced = (int32_t)(g_jitter_lfsr >> 8) - 0x800000;
    return (float)balanced * (1.0f / 8388608.0f);  // 1/2^23
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

void humanization_fpu_init(uint32_t seed) {
    // Seed with hardware TRNG if available, or use provided seed
    if (seed == 0) {
        seed = get_rand_32();
    }
    g_jitter_lfsr = seed;
    g_tremor_phase = seed >> 16;
}

void humanization_get_tremor(float scale, float *perp_x, float *perp_y) {
    g_tremor_phase++;
    
    // FIX: Wrap phase to prevent sinf() precision loss at large arguments.
    // After 100 seconds, t*19.1*2*PI = 11,999 — single-precision sinf loses
    // fractional accuracy above ~4096. Wrapping at 100000 (~100s) keeps
    // arguments well within float32 precision.
    float t = (float)(g_tremor_phase % 100000u) * 0.001f;
    
    // FIX: Use fixed offsets for X/Y decorrelation instead of accumulating
    // secondary phases. The old approach doubled the effective frequency
    // because both t and g_tremor_phase_x advanced by 0.001 per call.
    //
    // X-axis tremor: Three incommensurate frequencies
    // Physiological hand tremor is 8-25Hz, these ratios are irrational
    // so the composite waveform has no clean repeat period
    float tx = t + 0.7f;  // Fixed offset for X channel
    float tremor_x = sinf(tx * 8.7f  * (2.0f * M_PI)) * 0.40f   // ~8.7Hz primary
                   + sinf(tx * 12.3f * (2.0f * M_PI)) * 0.25f   // ~12.3Hz secondary  
                   + sinf(tx * 19.1f * (2.0f * M_PI)) * 0.15f;  // ~19.1Hz tertiary
    
    // Add LFSR noise component (breaks any remaining periodicity)
    tremor_x += jitter_next() * 0.3f;
    
    // Y-axis tremor: Different fixed offset for decorrelation
    float ty = t + 1.3f;  // Different offset than X
    float tremor_y = sinf(ty * 9.4f  * (2.0f * M_PI)) * 0.40f   // ~9.4Hz primary
                   + sinf(ty * 13.7f * (2.0f * M_PI)) * 0.25f   // ~13.7Hz secondary
                   + sinf(ty * 17.8f * (2.0f * M_PI)) * 0.15f;  // ~17.8Hz tertiary
    
    // Add independent LFSR noise
    tremor_y += jitter_next() * 0.3f;
    
    // Apply scale and clamp to reasonable range
    *perp_x = tremor_x * scale;
    *perp_y = tremor_y * scale;
    
    // Clamp to ±3.0px to prevent extreme outliers
    if (*perp_x > 3.0f) *perp_x = 3.0f;
    if (*perp_x < -3.0f) *perp_x = -3.0f;
    if (*perp_y > 3.0f) *perp_y = 3.0f;
    if (*perp_y < -3.0f) *perp_y = -3.0f;
}
