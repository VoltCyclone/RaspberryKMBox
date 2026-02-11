/**
 * @file neopixel_dma.h
 * @brief Zero-CPU NeoPixel (WS2812) driver for KMBox Bridge
 *
 * Architecture:
 *   1. A volatile shadow register holds the current GRB color (pre-shifted for PIO).
 *   2. The hot path writes here with a single 32-bit store (~1 cycle).
 *   3. A repeating hardware timer fires at ~30 Hz and triggers a one-shot DMA
 *      transfer from the shadow register into the WS2812 PIO TX FIFO.
 *   4. Breathing, rainbow, and activity-flash effects are computed entirely inside
 *      the timer ISR — the main loop never touches PIO or DMA for LED updates.
 *
 * This is the same proven pattern used by the KMBox firmware's led_control.c,
 * adapted for the bridge's simpler status model.
 */

#ifndef NEOPIXEL_DMA_H
#define NEOPIXEL_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// ── Status model (matches bridge main.c bridge_status_t) ────────────
typedef enum {
    NEO_STATUS_BOOTING = 0,
    NEO_STATUS_IDLE,
    NEO_STATUS_CDC_CONNECTED,
    NEO_STATUS_TRACKING,
    NEO_STATUS_DISABLED,
    NEO_STATUS_ERROR,
    NEO_STATUS_COUNT
} neo_status_t;

// ── Effect selection ────────────────────────────────────────────────
typedef enum {
    NEO_EFFECT_NONE = 0,   // Solid color
    NEO_EFFECT_BREATHING,  // Sine-ish brightness pulse
    NEO_EFFECT_RAINBOW,    // Hue rotation at full brightness
} neo_effect_t;

// ── Public API ──────────────────────────────────────────────────────

/**
 * @brief Initialise the NeoPixel subsystem: PIO program, DMA channel, 30 Hz timer.
 *
 * @param pio   PIO instance to use (usually pio0).
 * @param pin   GPIO pin connected to the WS2812 data line.
 *
 * Must be called once during boot, before any other neopixel_dma_* calls.
 */
void neopixel_dma_init(PIO pio, uint pin);

/**
 * @brief Set the current status (selects base color + default effect).
 *
 * The timer ISR applies the color on the next refresh tick (~33 ms max).
 * This is a single volatile store — safe to call from any context.
 */
void neopixel_dma_set_status(neo_status_t status);

/**
 * @brief Override the active effect (breathing / rainbow / none).
 *
 * Pass NEO_EFFECT_NONE to return to the status-default effect.
 */
void neopixel_dma_set_effect(neo_effect_t effect);

/**
 * @brief Set an explicit RGB color (bypasses status color for one refresh cycle).
 *
 * Useful for one-off flashes.  The status color resumes on the next tick
 * unless you call this again.
 */
void neopixel_dma_set_rgb(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Trigger a brief activity flash (overrides status color for ~100 ms).
 *
 * @param r, g, b  Flash color.
 */
void neopixel_dma_activity_flash(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Trigger an activity flash with configurable duration.
 *
 * @param r, g, b     Flash color.
 * @param duration_ms How long the flash lasts before reverting to status color.
 */
void neopixel_dma_activity_flash_ms(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);

/**
 * @brief Get the PIO instance and state-machine index used by this module.
 *
 * Exposed so other modules can avoid claiming the same resources.
 */
void neopixel_dma_get_hw(PIO *out_pio, uint *out_sm);

#endif // NEOPIXEL_DMA_H
