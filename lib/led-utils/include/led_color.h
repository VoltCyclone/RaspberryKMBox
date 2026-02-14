/**
 * Shared LED Color Utilities
 *
 * Integer-only HSV->RGB conversion and brightness scaling for WS2812.
 * Used by both the KMBox NeoPixel driver and the bridge NeoPixel DMA driver.
 */

#ifndef LED_COLOR_H
#define LED_COLOR_H

#include <stdint.h>

/**
 * Convert HSV to RGB (integer math, no FPU).
 *
 * @param h  Hue 0-359
 * @param s  Saturation 0-255
 * @param v  Value (brightness) 0-255
 * @param r  Output red 0-255
 * @param g  Output green 0-255
 * @param b  Output blue 0-255
 */
static inline void led_hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                                   uint8_t *r, uint8_t *g, uint8_t *b) {
    if (s == 0) { *r = *g = *b = v; return; }

    uint8_t  region    = h / 60;
    uint16_t remainder = (h - (region * 60)) * 6;  // 0-354 * 6 = 0-2124

    uint8_t p = (uint8_t)(((uint16_t)v * (255 - s)) >> 8);
    uint8_t q = (uint8_t)(((uint16_t)v * (255 - ((s * remainder) >> 8))) >> 8);
    uint8_t t = (uint8_t)(((uint16_t)v * (255 - ((s * (1530 - remainder)) >> 8))) >> 8);

    switch (region) {
        case 0:  *r = v; *g = t; *b = p; break;
        case 1:  *r = q; *g = v; *b = p; break;
        case 2:  *r = p; *g = v; *b = t; break;
        case 3:  *r = p; *g = q; *b = v; break;
        case 4:  *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

/**
 * Packed variant: returns 0x00RRGGBB.
 */
static inline uint32_t led_hsv_to_rgb_packed(uint16_t h, uint8_t s, uint8_t v) {
    uint8_t r, g, b;
    led_hsv_to_rgb(h, s, v, &r, &g, &b);
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

/**
 * Apply 0-255 brightness to an RGB triplet (integer math).
 */
static inline void led_apply_brightness_u8(uint8_t *r, uint8_t *g, uint8_t *b,
                                            uint8_t bright) {
    *r = (uint8_t)(((uint16_t)*r * bright) >> 8);
    *g = (uint8_t)(((uint16_t)*g * bright) >> 8);
    *b = (uint8_t)(((uint16_t)*b * bright) >> 8);
}

/**
 * Apply 0-255 brightness to a packed 0x00RRGGBB color, return packed result.
 */
static inline uint32_t led_apply_brightness_packed(uint32_t color, uint8_t bright) {
    uint8_t r = (uint8_t)(((uint16_t)((color >> 16) & 0xFF) * bright) >> 8);
    uint8_t g = (uint8_t)(((uint16_t)((color >>  8) & 0xFF) * bright) >> 8);
    uint8_t b = (uint8_t)(((uint16_t)( color        & 0xFF) * bright) >> 8);
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

#endif // LED_COLOR_H
