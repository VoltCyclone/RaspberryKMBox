/**
 * DCP Hardware Acceleration Helpers for RP2350
 *
 * Uses the Double Co-Processor for higher precision math.
 * All double-precision handled through DCP register macros (no VFP d-regs).
 */

.syntax unified
.cpu cortex-m33
.thumb

#include "hardware/dcp_instr.inc.S"
#include "hardware/dcp_canned.inc.S"

// =====================================================================
// dcp_adc_to_temp
//   float dcp_adc_to_temp(uint16_t raw_adc)
//   temp = 27.0 - (raw * (3.3/4096) - 0.706) / 0.001721
// =====================================================================
.global dcp_adc_to_temp
.thumb_func
dcp_adc_to_temp:
    push {r4-r12, r14}

    // Convert uint16 raw to double in r0:r1
    dcp_uint2double_m r0, r1, r0

    // Load 3.3/4096.0 = 0.000805664062500
    // double = 0x3F4A4DD2F1A9FBE7
    movw r2, #0xFBE7
    movt r2, #0xF1A9
    movw r3, #0x4DD2
    movt r3, #0x3F4A

    // voltage = raw * (3.3/4096)
    dcp_dmul_m r4, r5, r0, r1, r2, r3, r6, r7, r8, r9, r10, r11, r12

    // Load 0.706 = 0x3FE68F5C28F5C28F
    movw r0, #0xC28F
    movt r0, #0x28F5
    movw r1, #0x8F5C
    movt r1, #0x3FE6

    // voltage - 0.706
    dcp_dsub_m r4, r5, r4, r5, r0, r1

    // Load 0.001721 = 0x3F5C2B020C49BA5E
    movw r0, #0xBA5E
    movt r0, #0x0C49
    movw r1, #0x2B02
    movt r1, #0x3F5C

    // (voltage - 0.706) / 0.001721
    dcp_ddiv_m r6, r7, r4, r5, r0, r1, r8, r9, r10, r11, r12

    // Load 27.0 = 0x403B000000000000
    movw r0, #0x0000
    movt r0, #0x0000
    movw r1, #0x0000
    movt r1, #0x403B

    // 27.0 - result
    dcp_dsub_m r4, r5, r0, r1, r6, r7

    // Convert double to float
    dcp_double2float_m r0, r4, r5

    pop {r4-r12, r15}

// =====================================================================
// dcp_scale_channel
//   uint8_t dcp_scale_channel(uint8_t val, float brightness)
//   result = val * brightness (clamped to 255)
// =====================================================================
.global dcp_scale_channel
.thumb_func
dcp_scale_channel:
    push {r4-r12, r14}

    // Convert uint8 color component to double
    dcp_uint2double_m r4, r5, r0

    // Convert float brightness to double
    dcp_float2double_m r6, r7, r1

    // Multiply
    dcp_dmul_m r0, r1, r4, r5, r6, r7, r2, r3, r8, r9, r10, r11, r12

    // Convert double result back to uint
    dcp_double2uint_m r0, r0, r1

    // Clamp to 255
    cmp r0, #255
    it hi
    movhi r0, #255

    pop {r4-r12, r15}

// =====================================================================
// dcp_apply_brightness_rgb
//   uint32_t dcp_apply_brightness_rgb(uint32_t color, float brightness)
//   Scale each R/G/B channel by brightness
// =====================================================================
.global dcp_apply_brightness_rgb
.thumb_func
dcp_apply_brightness_rgb:
    push {r4-r6, r14}
    mov r4, r0                    // save color
    mov r5, r1                    // save brightness

    // Red channel
    ubfx r0, r4, #16, #8
    mov r1, r5
    bl dcp_scale_channel
    lsl r6, r0, #16

    // Green channel
    ubfx r0, r4, #8, #8
    mov r1, r5
    bl dcp_scale_channel
    orr r6, r6, r0, lsl #8

    // Blue channel
    ubfx r0, r4, #0, #8
    mov r1, r5
    bl dcp_scale_channel
    orr r0, r6, r0

    pop {r4-r6, r15}
