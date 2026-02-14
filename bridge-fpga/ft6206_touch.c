/**
 * FT6206 Capacitive Touch Controller Driver Implementation
 * 
 * I2C driver for the FT6206/FT6236 capacitive touch controller
 * found on Adafruit 2.8" ILI9341 Arduino TFT shields.
 * 
 * Protocol:
 *   - I2C address 0x38
 *   - Register 0x02: number of active touch points (0-2)
 *   - Register 0x03-0x06: touch point 1 (x_hi, x_lo, y_hi, y_lo)
 *   - Register 0xA8: vendor/chip ID (0x11 for FT6206)
 *   - Register 0x80: touch threshold (default 128)
 */

#include "ft6206_touch.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdio.h>

// FT6206 I2C address (7-bit, fixed)
#define FT6206_ADDR             0x38

// FT6206 Registers
#define FT6206_REG_MODE         0x00    // Device mode (0=working, others=test)
#define FT6206_REG_GEST_ID      0x01    // Gesture ID
#define FT6206_REG_NUM_TOUCHES  0x02    // Number of touch points (0-2)
#define FT6206_REG_P1_XH        0x03    // Touch point 1: X high byte [7:6]=event, [3:0]=X[11:8]
#define FT6206_REG_P1_XL        0x04    // Touch point 1: X low byte X[7:0]
#define FT6206_REG_P1_YH        0x05    // Touch point 1: Y high byte [7:6]=touch_id, [3:0]=Y[11:8]
#define FT6206_REG_P1_YL        0x06    // Touch point 1: Y low byte Y[7:0]
#define FT6206_REG_THRESHHOLD   0x80    // Touch detection threshold
#define FT6206_REG_CTRL         0x86    // Control register
#define FT6206_REG_TIMEPERIOD   0x87    // Active period (x10 ms)
#define FT6206_REG_VENDID       0xA8    // Vendor ID (should be 0x11)
#define FT6206_REG_CHIPID       0xA3    // Chip ID (0x06 for FT6206, 0x36 for FT6236)
#define FT6206_REG_FIRMVERS     0xA6    // Firmware version

// Touch event types (bits [7:6] of XH register)
#define FT6206_EVENT_PRESS_DOWN 0x00
#define FT6206_EVENT_LIFT_UP    0x01
#define FT6206_EVENT_CONTACT    0x02
#define FT6206_EVENT_NONE       0x03

// Timing
#define FT6206_DEBOUNCE_MS      100     // Minimum time between tap events
#define FT6206_I2C_TIMEOUT_US   5000    // I2C transaction timeout

// State
static i2c_inst_t *touch_i2c = NULL;
static bool initialized = false;
static touch_callback_t touch_callback = NULL;

// Touch state machine for tap detection (polled, not interrupt-driven)
static bool last_touched = false;
static uint32_t last_touch_time = 0;
static uint32_t touch_down_time = 0;
static touch_point_t last_touch_point = {0, 0, false};  // Coordinates at touch-down

// Calibration (defaults match Adafruit 2.8" ILI9341 capacitive shield)
static touch_calibration_t calibration = {
    .x_min = 0,
    .x_max = 240,
    .y_min = 0,
    .y_max = 320,
    .swap_xy = false,
    .invert_x = false,
    .invert_y = true     // Display Y=0 is top, but touch Y=0 is bottom on Metro RP2350
};

//--------------------------------------------------------------------+
// I2C Helpers
//--------------------------------------------------------------------+

static bool ft6206_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_timeout_us(touch_i2c, FT6206_ADDR, buf, 2, false, FT6206_I2C_TIMEOUT_US);
    return (ret == 2);
}

static bool ft6206_read_reg(uint8_t reg, uint8_t *value) {
    int ret = i2c_write_timeout_us(touch_i2c, FT6206_ADDR, &reg, 1, true, FT6206_I2C_TIMEOUT_US);
    if (ret != 1) return false;
    ret = i2c_read_timeout_us(touch_i2c, FT6206_ADDR, value, 1, false, FT6206_I2C_TIMEOUT_US);
    return (ret == 1);
}

static bool ft6206_read_regs(uint8_t start_reg, uint8_t *buf, size_t len) {
    int ret = i2c_write_timeout_us(touch_i2c, FT6206_ADDR, &start_reg, 1, true, FT6206_I2C_TIMEOUT_US);
    if (ret != 1) return false;
    ret = i2c_read_timeout_us(touch_i2c, FT6206_ADDR, buf, len, false, FT6206_I2C_TIMEOUT_US);
    return (ret == (int)len);
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

bool ft6206_init(void *i2c_dev, uint8_t sda_pin, uint8_t scl_pin) {
    if (i2c_dev == NULL) {
        return false;
    }
    
    touch_i2c = (i2c_inst_t *)i2c_dev;
    
    // Initialize I2C at 400kHz (FT6206 supports up to 400kHz)
    i2c_init(touch_i2c, 400 * 1000);
    
    // Configure GPIO pins for I2C function
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // Small delay for bus to settle
    sleep_ms(10);
    
    // Verify the FT6206 is present by reading vendor ID
    uint8_t vendor_id = 0;
    if (!ft6206_read_reg(FT6206_REG_VENDID, &vendor_id)) {
        printf("[FT6206] I2C read failed - touch controller not detected\n");
        return false;
    }
    
    // FT6206 vendor ID is typically 0x11
    // FT6236 may report different IDs - accept any non-zero response
    if (vendor_id == 0x00 || vendor_id == 0xFF) {
        printf("[FT6206] Invalid vendor ID: 0x%02X - touch controller not responding\n", vendor_id);
        return false;
    }
    
    // Read chip ID for diagnostics
    uint8_t chip_id = 0;
    ft6206_read_reg(FT6206_REG_CHIPID, &chip_id);
    
    uint8_t fw_ver = 0;
    ft6206_read_reg(FT6206_REG_FIRMVERS, &fw_ver);
    
    printf("[FT6206] Touch controller detected: vendor=0x%02X chip=0x%02X fw=0x%02X\n",
           vendor_id, chip_id, fw_ver);
    
    // Set touch detection threshold (lower = more sensitive)
    ft6206_write_reg(FT6206_REG_THRESHHOLD, 80);
    
    // Set to active polling mode, 10ms period
    ft6206_write_reg(FT6206_REG_TIMEPERIOD, 1);  // 10ms active period
    
    initialized = true;
    last_touched = false;
    touch_down_time = 0;
    
    return true;
}

void ft6206_set_callback(touch_callback_t callback) {
    touch_callback = callback;
}

bool ft6206_is_touched(void) {
    if (!initialized) return false;
    
    uint8_t num_touches = 0;
    if (!ft6206_read_reg(FT6206_REG_NUM_TOUCHES, &num_touches)) {
        return false;
    }
    
    // FT6206 reports 0-2 touches; anything > 2 is invalid
    return (num_touches > 0 && num_touches <= 2);
}

bool ft6206_read(touch_point_t *point) {
    if (!initialized || point == NULL) {
        if (point) point->valid = false;
        return false;
    }
    
    // Read touch registers in a single burst (reg 0x02 through 0x06 = 5 bytes)
    uint8_t buf[5];
    if (!ft6206_read_regs(FT6206_REG_NUM_TOUCHES, buf, 5)) {
        point->valid = false;
        return false;
    }
    
    uint8_t num_touches = buf[0];
    if (num_touches == 0 || num_touches > 2) {
        point->valid = false;
        return false;
    }
    
    // Parse touch point 1 from buf[1..4] (registers 0x03-0x06)
    uint8_t event = (buf[1] >> 6) & 0x03;
    uint16_t x = ((buf[1] & 0x0F) << 8) | buf[2];
    uint16_t y = ((buf[3] & 0x0F) << 8) | buf[4];
    
    // Apply calibration
    int32_t cx = x;
    int32_t cy = y;
    
    if (calibration.swap_xy) {
        int32_t tmp = cx;
        cx = cy;
        cy = tmp;
    }
    
    // Map to calibrated range
    cx = ((cx - calibration.x_min) * 240) / (calibration.x_max - calibration.x_min);
    cy = ((cy - calibration.y_min) * 320) / (calibration.y_max - calibration.y_min);
    
    if (calibration.invert_x) cx = 239 - cx;
    if (calibration.invert_y) cy = 319 - cy;
    
    // Clamp
    if (cx < 0) cx = 0;
    if (cx > 239) cx = 239;
    if (cy < 0) cy = 0;
    if (cy > 319) cy = 319;
    
    point->x = (uint16_t)cx;
    point->y = (uint16_t)cy;
    point->valid = true;
    
    return true;
}

void ft6206_poll(void) {
    if (!initialized) return;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool currently_touched = ft6206_is_touched();
    
    if (currently_touched && !last_touched) {
        // Touch just started — capture coordinates at press time
        touch_down_time = now;
        ft6206_read(&last_touch_point);
    } else if (!currently_touched && last_touched) {
        // Touch just released — this is a tap event
        uint32_t duration = now - touch_down_time;
        
        // Debounce: ignore taps that are too short (< 30ms, likely noise)
        // or too long (> 1000ms, likely a press-and-hold)
        if (duration >= 30 && duration <= 1000) {
            // Rate limit: ignore if too soon after last tap
            if (now - last_touch_time >= FT6206_DEBOUNCE_MS) {
                last_touch_time = now;
                
                if (touch_callback && last_touch_point.valid) {
                    touch_callback(last_touch_point.x, last_touch_point.y);
                }
            }
        }
    }
    
    last_touched = currently_touched;
}

void ft6206_set_calibration(const touch_calibration_t *cal) {
    if (cal != NULL) {
        memcpy(&calibration, cal, sizeof(touch_calibration_t));
    }
}

uint8_t ft6206_get_vendor_id(void) {
    if (!initialized) return 0;
    uint8_t id = 0;
    ft6206_read_reg(FT6206_REG_VENDID, &id);
    return id;
}
