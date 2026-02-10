#!/usr/bin/env python3
"""
Generate precomputed lookup tables for humanization features.

This script generates mathematically correct tables for:
- Easing curves (ease-in-out cubic, ease-out quad, linear)
- Progress fractions
- Jitter patterns
- Sine quarter-wave
- Overshoot magnitudes
- Acceleration curves
- Sub-pixel dither patterns

Run this script to regenerate humanization_lut.c with correct values.
"""

import math
import random

# Fixed-point configuration (16.16 format)
FP_SHIFT = 16
FP_ONE = 1 << FP_SHIFT  # 65536

# Table sizes
EASING_LUT_SIZE = 256
JITTER_LUT_SIZE = 64
SINE_LUT_SIZE = 64
MAX_PRECOMPUTED_FRAMES = 32
OVERSHOOT_LUT_SIZE = 16
ACCEL_LUT_SIZE = 32
SUBPIXEL_DITHER_SIZE = 16
JITTER_SCALE_LUT_SIZE = 32
FRAME_SPREAD_LUT_SIZE = 32


def ease_in_out_cubic(t):
    """Cubic ease-in-out: slow start, fast middle, slow end."""
    if t < 0.5:
        return 4 * t * t * t
    else:
        return 1 - pow(-2 * t + 2, 3) / 2


def ease_out_quad(t):
    """Quadratic ease-out: fast start, slow end."""
    return 1 - (1 - t) * (1 - t)


def generate_easing_table(ease_func, name):
    """Generate a 256-entry easing table."""
    values = []
    for i in range(EASING_LUT_SIZE):
        t = i / (EASING_LUT_SIZE - 1)
        eased = ease_func(t)
        fp_value = int(round(eased * FP_ONE))
        # Clamp to valid range
        fp_value = max(0, min(FP_ONE, fp_value))
        values.append(fp_value)
    return values


def generate_linear_table():
    """Generate linear easing (identity function)."""
    values = []
    for i in range(EASING_LUT_SIZE):
        t = i / (EASING_LUT_SIZE - 1)
        fp_value = int(round(t * FP_ONE))
        values.append(fp_value)
    return values


def generate_progress_table():
    """Generate progress fraction lookup table."""
    # g_progress_lut[total_frames][current_frame] = current_frame / total_frames
    table = []
    for total in range(MAX_PRECOMPUTED_FRAMES + 1):
        row = []
        for current in range(MAX_PRECOMPUTED_FRAMES + 1):
            if total == 0:
                value = FP_ONE if current == 0 else 0
            elif current >= total:
                value = FP_ONE
            else:
                value = int(round((current / total) * FP_ONE))
            row.append(value)
        table.append(row)
    return table


def generate_jitter_table():
    """Generate pseudo-random jitter patterns that sum to ~0."""
    random.seed(42)  # Reproducible
    
    # Generate X jitter - Base amplitude 1.0px for realistic hand tremor
    # Anti-cheat expects 0.5-1.5px perpendicular wobble
    jitter_x = []
    for i in range(JITTER_LUT_SIZE):
        # Use deterministic pattern based on index
        angle = (i * 7 + 3) * 2 * math.pi / JITTER_LUT_SIZE
        value = math.sin(angle) * 1.0  # Range [-1.0, 1.0] pixels
        jitter_x.append(int(round(value * FP_ONE)))
    
    # Generate Y jitter - different phase for 2D realism
    jitter_y = []
    for i in range(JITTER_LUT_SIZE):
        angle = (i * 11 + 7) * 2 * math.pi / JITTER_LUT_SIZE
        value = math.sin(angle) * 1.0  # Range [-1.0, 1.0] pixels
        jitter_y.append(int(round(value * FP_ONE)))
    
    return jitter_x, jitter_y


def generate_jitter_masks():
    """Generate jitter application masks (~40% density)."""
    # Split into lo (bits 0-31) and hi (bits 32-63) for RP2040 efficiency
    patterns_lo = []
    patterns_hi = []
    
    # Pattern 1: Every 3rd frame approximately
    p1 = 0x924924924924924  # 100100100... pattern
    patterns_lo.append(p1 & 0xFFFFFFFF)
    patterns_hi.append((p1 >> 32) & 0xFFFFFFFF)
    
    # Pattern 2: Shifted by 1
    p2 = 0x492492492492492
    patterns_lo.append(p2 & 0xFFFFFFFF)
    patterns_hi.append((p2 >> 32) & 0xFFFFFFFF)
    
    # Pattern 3: Shifted by 2
    p3 = 0x249249249249249
    patterns_lo.append(p3 & 0xFFFFFFFF)
    patterns_hi.append((p3 >> 32) & 0xFFFFFFFF)
    
    # Pattern 4: Different rhythm
    p4 = 0x1248124812481248
    patterns_lo.append(p4 & 0xFFFFFFFF)
    patterns_hi.append((p4 >> 32) & 0xFFFFFFFF)
    
    return patterns_lo, patterns_hi


def generate_sine_quarter():
    """Generate quarter-wave sine table (0 to π/2)."""
    values = []
    for i in range(SINE_LUT_SIZE):
        angle = (i / (SINE_LUT_SIZE - 1)) * (math.pi / 2)
        value = math.sin(angle)
        values.append(int(round(value * FP_ONE)))
    return values


def generate_overshoot_table():
    """Generate overshoot magnitudes indexed by movement size."""
    values = []
    for i in range(OVERSHOOT_LUT_SIZE):
        # Larger movements get larger overshoots
        # Range: 0.25 to 2.0 pixels
        magnitude = 0.25 + (i / (OVERSHOOT_LUT_SIZE - 1)) * 1.75
        values.append(int(round(magnitude * FP_ONE)))
    return values


def generate_accel_curve():
    """Generate acceleration/deceleration curve (bell-shaped)."""
    # Based on minimum-jerk trajectory: v(t) = 30*t²*(1-t)²
    # Normalized so peak is around 1.5x
    values = []
    for i in range(ACCEL_LUT_SIZE):
        t = i / (ACCEL_LUT_SIZE - 1)
        # Minimum-jerk velocity profile
        v = 30 * t * t * (1 - t) * (1 - t)
        # Normalize: peak of 30*t²*(1-t)² at t=0.5 is 30*0.25*0.25 = 1.875
        # Scale to reasonable multiplier (peak ~1.5)
        normalized = v / 1.875 * 1.5
        values.append(int(round(normalized * FP_ONE)))
    return values


def generate_subpixel_dither():
    """Generate sub-pixel dither pattern that sums to 0."""
    # Balanced pattern for breaking up systematic accumulation
    values = [
        0.125, -0.1875, 0.25, -0.0625,
        -0.125, 0.1875, -0.25, 0.0625,
        -0.0625, 0.25, -0.125, -0.1875,
        0.125, -0.25, 0.1875, 0.0625
    ]
    # Verify sum is ~0
    assert abs(sum(values)) < 0.001, f"Dither sum not zero: {sum(values)}"
    return [int(round(v * FP_ONE)) for v in values]


def generate_jitter_scale_by_movement():
    """Generate jitter scale LUT based on movement magnitude.
    
    Small movements (0-20px): Higher jitter (0.8x-0.7x)
    Medium movements (20-100px): Moderate jitter (0.6x-0.2x)
    Large movements (100+px): Minimal jitter (0.1x-0.05x)
    """
    values = []
    
    for i in range(JITTER_SCALE_LUT_SIZE):
        # Each index represents 10px of movement (0-10px, 10-20px, etc.)
        movement_px = i * 10
        
        if movement_px < 20:
            # 0-20px: 1.5x to 1.2x (precise movements have more visible tremor)
            scale = 1.5 - (movement_px / 20) * 0.3
        elif movement_px < 40:
            # 20-40px: 1.2x to 0.9x
            scale = 1.2 - ((movement_px - 20) / 20) * 0.3
        elif movement_px < 60:
            # 40-60px: 0.9x to 0.6x
            scale = 0.9 - ((movement_px - 40) / 20) * 0.3
        elif movement_px < 100:
            # 60-100px: 0.6x to 0.3x
            scale = 0.6 - ((movement_px - 60) / 40) * 0.3
        elif movement_px < 140:
            # 100-140px: 0.3x to 0.15x (fast flicks suppress tremor)
            scale = 0.3 - ((movement_px - 100) / 40) * 0.15
        else:
            # 140+px: 0.15x (minimal but non-zero)
            scale = 0.15
        
        values.append(int(round(scale * FP_ONE)))
    
    return values


def generate_frame_spread_by_movement():
    """Generate frame spread multiplier LUT based on movement magnitude.
    
    Small movements (0-20px): Slower, more frames (1.3x-1.2x)
    Medium movements (20-100px): Balanced (1.2x-0.8x)
    Large movements (100+px): Faster, fewer frames (0.7x)
    """
    values = []
    
    for i in range(FRAME_SPREAD_LUT_SIZE):
        # Each index represents 10px of movement
        movement_px = i * 10
        
        if movement_px < 20:
            # 0-20px: 1.3x to 1.2x (slow, careful)
            multiplier = 1.3 - (movement_px / 20) * 0.1
        elif movement_px < 60:
            # 20-60px: 1.2x to 1.0x
            multiplier = 1.2 - ((movement_px - 20) / 40) * 0.2
        elif movement_px < 100:
            # 60-100px: 1.0x to 0.8x
            multiplier = 1.0 - ((movement_px - 60) / 40) * 0.2
        elif movement_px < 140:
            # 100-140px: 0.8x to 0.7x (fast, ballistic)
            multiplier = 0.8 - ((movement_px - 100) / 40) * 0.1
        else:
            # 140+px: 0.7x (maintain fast)
            multiplier = 0.7
        
        values.append(int(round(multiplier * FP_ONE)))
    
    return values


def format_array(values, per_line=8, indent="    "):
    """Format array values for C code."""
    lines = []
    for i in range(0, len(values), per_line):
        chunk = values[i:i+per_line]
        line = indent + ", ".join(f"{v}" for v in chunk)
        if i + per_line < len(values):
            line += ","
        lines.append(line)
    return "\n".join(lines)


def format_2d_array(table, indent="    "):
    """Format 2D array for C code."""
    lines = []
    for i, row in enumerate(table):
        row_str = ", ".join(f"{v}" for v in row)
        comment = f"// total_frames = {i}"
        if i < len(table) - 1:
            lines.append(f"{indent}{{{row_str}}},  {comment}")
        else:
            lines.append(f"{indent}{{{row_str}}}   {comment}")
    return "\n".join(lines)


def generate_c_file():
    """Generate the complete humanization_lut.c file."""
    
    # Generate all tables
    ease_in_out = generate_easing_table(ease_in_out_cubic, "ease_in_out_cubic")
    ease_out = generate_easing_table(ease_out_quad, "ease_out_quad")
    linear = generate_linear_table()
    progress = generate_progress_table()
    jitter_x, jitter_y = generate_jitter_table()
    masks_lo, masks_hi = generate_jitter_masks()
    sine = generate_sine_quarter()
    overshoot = generate_overshoot_table()
    accel = generate_accel_curve()
    dither = generate_subpixel_dither()
    jitter_scale = generate_jitter_scale_by_movement()
    frame_spread = generate_frame_spread_by_movement()
    
    # Build C file content
    c_code = '''/*
 * Precomputed Lookup Tables for Humanization
 * 
 * AUTO-GENERATED by tools/generate_lut.py
 * Do not edit manually - regenerate with: python3 tools/generate_lut.py
 * 
 * These tables are stored in flash (.rodata) and consume no RAM.
 */

#include "humanization_lut.h"
#include "smooth_injection.h"

//--------------------------------------------------------------------+
// Ease-In-Out Cubic Table
//
// Formula: if t < 0.5: 4*t³
//          else: 1 - (-2t + 2)³ / 2
// 
// Provides natural acceleration/deceleration like human movement
// 256 entries, input t = index/255, output in 16.16 fixed-point
//--------------------------------------------------------------------+

const int32_t g_ease_in_out_cubic_lut[EASING_LUT_SIZE] = {
'''
    c_code += format_array(ease_in_out) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Ease-Out Quadratic Table
//
// Formula: 1 - (1-t)²
// 
// Quick start, gradual slowdown - good for corrections
//--------------------------------------------------------------------+

const int32_t g_ease_out_quad_lut[EASING_LUT_SIZE] = {
'''
    c_code += format_array(ease_out) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Linear Easing Table (identity function)
//--------------------------------------------------------------------+

const int32_t g_ease_linear_lut[EASING_LUT_SIZE] = {
'''
    c_code += format_array(linear) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Progress Lookup Table
//
// g_progress_lut[total_frames][current_frame] = current_frame / total_frames
// in 16.16 fixed-point format
// 
// Eliminates division in the hot path for frame spreading
//--------------------------------------------------------------------+

const int32_t g_progress_lut[MAX_PRECOMPUTED_FRAMES + 1][MAX_PRECOMPUTED_FRAMES + 1] = {
'''
    c_code += format_2d_array(progress) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Jitter Pattern Tables
//
// Pre-generated pseudo-random jitter values for hand tremor simulation
// Values in 16.16 fixed-point, designed to sum to ~0 over cycles
//--------------------------------------------------------------------+

// X-axis jitter
const int32_t g_jitter_x_lut[JITTER_LUT_SIZE] = {
'''
    c_code += format_array(jitter_x) + "\n};\n\n"

    c_code += '''// Y-axis jitter (different pattern for 2D realism)
const int32_t g_jitter_y_lut[JITTER_LUT_SIZE] = {
'''
    c_code += format_array(jitter_y) + "\n};\n\n"

    c_code += '''// Jitter application masks - split into lo/hi 32-bit halves for RP2040 efficiency
// ~40% density with varying patterns to avoid repetition detection

// Low 32 bits (frames 0-31)
const uint32_t g_jitter_mask_lo_lut[4] = {
'''
    c_code += format_array([f"0x{v:08X}" for v in masks_lo], per_line=4) + "\n};\n\n"

    c_code += '''// High 32 bits (frames 32-63)
const uint32_t g_jitter_mask_hi_lut[4] = {
'''
    c_code += format_array([f"0x{v:08X}" for v in masks_hi], per_line=4) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Quarter-Wave Sine Table
//
// sin(θ) for θ = 0 to π/2 in 64 steps
// Output in 16.16 fixed-point [0, 65536]
// Full sine reconstructed by mirroring
//--------------------------------------------------------------------+

const int32_t g_sine_quarter_lut[SINE_LUT_SIZE] = {
'''
    c_code += format_array(sine) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Overshoot Magnitude Table
//
// Pre-calculated overshoot amounts indexed by movement magnitude
// Larger movements have proportionally larger overshoots (realistic)
// Values in 16.16 fixed-point (0.25 to 2.0 pixels)
//--------------------------------------------------------------------+

const int32_t g_overshoot_lut[OVERSHOOT_LUT_SIZE] = {
'''
    c_code += format_array(overshoot) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Acceleration/Deceleration Curve Table
//
// Natural movement velocity profile: slow start, fast middle, slow end
// Based on minimum-jerk trajectory: v(t) = 30*t²*(1-t)²
// Normalized with peak multiplier ~1.5
//--------------------------------------------------------------------+

const int32_t g_accel_curve_lut[ACCEL_LUT_SIZE] = {
'''
    c_code += format_array(accel) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Sub-Pixel Dither Pattern
//
// Breaks up systematic patterns in fractional pixel accumulation
// Values sum to 0 over the full cycle to avoid drift
// Range: approximately ±0.25 pixels
//--------------------------------------------------------------------+

const int32_t g_subpixel_dither_lut[SUBPIXEL_DITHER_SIZE] = {
'''
    c_code += format_array(dither) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Movement-Type Jitter Scale LUT
//
// Tremor characteristics by movement type:
// Index 0-1 (0-20px): 0.8x-0.7x jitter (small, precise adjustments)
// Index 2-5 (20-60px): 0.7x-0.3x jitter (medium, controlled)
// Index 6-10 (60-110px): 0.3x-0.1x jitter (large, intentional)
// Index 11+ (110+px): 0.1x-0.05x jitter (fast flicks, tremor suppressed)
//--------------------------------------------------------------------+

const int32_t g_jitter_scale_by_movement_lut[JITTER_SCALE_LUT_SIZE] = {
'''
    c_code += format_array(jitter_scale) + "\n};\n\n"

    c_code += '''//--------------------------------------------------------------------+
// Movement-Type Frame Spread LUT
//
// Frame timing by movement type:
// Small movements: Spread over MORE frames (careful, precise)
// Large movements: Spread over FEWER frames (fast, ballistic)
//--------------------------------------------------------------------+

const int32_t g_frame_spread_by_movement_lut[FRAME_SPREAD_LUT_SIZE] = {
'''
    c_code += format_array(frame_spread) + "\n};\n"

    return c_code


def main():
    c_code = generate_c_file()
    
    output_path = "../humanization_lut.c"
    with open(output_path, "w") as f:
        f.write(c_code)
    
    print(f"Generated {output_path}")
    print(f"  - Easing tables: {EASING_LUT_SIZE} entries each")
    print(f"  - Progress table: {MAX_PRECOMPUTED_FRAMES+1}x{MAX_PRECOMPUTED_FRAMES+1} entries")
    print(f"  - Jitter tables: {JITTER_LUT_SIZE} entries each")
    print(f"  - Sine table: {SINE_LUT_SIZE} entries")
    print(f"  - Overshoot table: {OVERSHOOT_LUT_SIZE} entries")
    print(f"  - Accel curve: {ACCEL_LUT_SIZE} entries")
    print(f"  - Dither pattern: {SUBPIXEL_DITHER_SIZE} entries")
    print(f"  - Jitter scale LUT: {JITTER_SCALE_LUT_SIZE} entries")
    print(f"  - Frame spread LUT: {FRAME_SPREAD_LUT_SIZE} entries")


if __name__ == "__main__":
    main()
