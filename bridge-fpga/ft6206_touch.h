/**
 * FT6206 Capacitive Touch Controller Driver
 * 
 * I2C-based capacitive touch controller used on Adafruit ILI9341 
 * Arduino TFT shields. Communicates over I2C (SDA/SCL on Arduino A4/A5).
 * 
 * I2C Address: 0x38 (fixed)
 * Supports up to 2 simultaneous touch points.
 */

#ifndef FT6206_TOUCH_H
#define FT6206_TOUCH_H

#include <stdint.h>
#include <stdbool.h>

// Touch point structure
typedef struct {
    uint16_t x;         // Screen X coordinate
    uint16_t y;         // Screen Y coordinate
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

// Touch event callback type - receives tap coordinates
typedef void (*touch_callback_t)(uint16_t x, uint16_t y);

/**
 * Initialize FT6206 touch controller over I2C
 * 
 * @param i2c_dev I2C device (e.g., i2c0)
 * @param sda_pin I2C SDA pin
 * @param scl_pin I2C SCL pin
 * @return true if FT6206 detected and initialized
 */
bool ft6206_init(void *i2c_dev, uint8_t sda_pin, uint8_t scl_pin);

/**
 * Register a callback for touch events (triggered on touch release)
 * Called from polling context (not ISR).
 * Callback receives the (x, y) screen coordinates of the tap.
 * 
 * @param callback Function to call on touch tap, or NULL to disable
 */
void ft6206_set_callback(touch_callback_t callback);

/**
 * Poll for touch events. Call this regularly from the main loop.
 * Handles debouncing, press/release detection, and callback invocation.
 */
void ft6206_poll(void);

/**
 * Read current touch point
 * 
 * @param point Pointer to touch_point_t to fill
 * @return true if touch detected
 */
bool ft6206_read(touch_point_t *point);

/**
 * Check if screen is currently touched
 * 
 * @return true if touched
 */
bool ft6206_is_touched(void);

/**
 * Set calibration parameters for raw→screen coordinate mapping
 * 
 * @param cal Pointer to calibration structure
 */
void ft6206_set_calibration(const touch_calibration_t *cal);

/**
 * Get the FT6206 chip vendor ID (should be 0x11 for FT6206)
 * 
 * @return Vendor ID byte, or 0 if read failed
 */
uint8_t ft6206_get_vendor_id(void);

#endif // FT6206_TOUCH_H
