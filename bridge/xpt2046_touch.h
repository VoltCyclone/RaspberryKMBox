/**
 * XPT2046 Touch Controller Driver
 * 
 * Resistive touch controller commonly paired with ILI9341 displays.
 * Communicates over SPI using a separate CS pin.
 */

#ifndef XPT2046_TOUCH_H
#define XPT2046_TOUCH_H

#include <stdint.h>
#include <stdbool.h>

// Touch point structure
typedef struct {
    uint16_t x;         // Raw X coordinate (0-4095)
    uint16_t y;         // Raw Y coordinate (0-4095)
    uint16_t pressure;  // Pressure (Z) value
    bool valid;         // True if touch is detected
} touch_point_t;

// Calibration structure for mapping raw values to screen coordinates
typedef struct {
    int16_t x_min;
    int16_t x_max;
    int16_t y_min;
    int16_t y_max;
    bool swap_xy;       // True if X and Y axes are swapped
    bool invert_x;      // True if X axis is inverted
    bool invert_y;      // True if Y axis is inverted
} touch_calibration_t;

// Touch event callback type
typedef void (*touch_callback_t)(void);

/**
 * Initialize touch controller
 * 
 * @param spi_dev SPI device (e.g., spi1)
 * @param cs_pin Chip select pin for touch controller
 * @param irq_pin Touch IRQ pin (optional, use -1 if not connected)
 * @return true if successful
 */
bool xpt2046_init(void *spi_dev, uint8_t cs_pin, int8_t irq_pin);

/**
 * Register a callback for touch events (triggered on touch release)
 * Callback is called from ISR context, keep it fast!
 * 
 * @param callback Function to call on touch release, or NULL to disable
 */
void xpt2046_set_callback(touch_callback_t callback);

/**
 * Read raw touch coordinates
 * 
 * @param point Pointer to touch_point_t to fill
 * @return true if touch detected
 */
bool xpt2046_read_raw(touch_point_t *point);

/**
 * Check if screen is currently touched (fast check using IRQ or quick read)
 * 
 * @return true if touched
 */
bool xpt2046_is_touched(void);

/**
 * Set calibration parameters
 * 
 * @param cal Pointer to calibration structure
 */
void xpt2046_set_calibration(const touch_calibration_t *cal);

/**
 * Convert raw touch coordinates to screen coordinates
 * 
 * @param raw Raw touch point
 * @param screen_x Pointer to store screen X (0 to screen width)
 * @param screen_y Pointer to store screen Y (0 to screen height)
 * @param screen_width Display width in pixels
 * @param screen_height Display height in pixels
 */
void xpt2046_raw_to_screen(const touch_point_t *raw, uint16_t *screen_x, 
                           uint16_t *screen_y, uint16_t screen_width, 
                           uint16_t screen_height);

#endif // XPT2046_TOUCH_H
