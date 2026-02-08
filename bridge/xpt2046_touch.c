/**
 * XPT2046 Touch Controller Driver Implementation
 */

#include "xpt2046_touch.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <string.h>

// Forward declaration
static void touch_irq_handler(uint gpio, uint32_t events);

// XPT2046 Commands
#define XPT2046_CMD_X       0xD0  // Read X position (12-bit)
#define XPT2046_CMD_Y       0x90  // Read Y position (12-bit)
#define XPT2046_CMD_Z1      0xB0  // Read Z1 (pressure)
#define XPT2046_CMD_Z2      0xC0  // Read Z2 (pressure)

// Touch detection threshold
#define TOUCH_THRESHOLD     400   // Minimum pressure to consider as touch

static spi_inst_t *touch_spi = NULL;
static uint8_t touch_cs_pin = 0;
static int8_t touch_irq_pin = -1;
static bool initialized = false;

// Interrupt handling
static touch_callback_t touch_callback = NULL;
static volatile bool touch_pressed = false;
static volatile bool touch_event_pending = false;
static uint32_t touch_press_time = 0;
#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 1000

// Default calibration (will need adjustment per display)
static touch_calibration_t calibration = {
    .x_min = 200,
    .x_max = 3900,
    .y_min = 200,
    .y_max = 3900,
    .swap_xy = false,
    .invert_x = false,
    .invert_y = false
};

bool xpt2046_init(void *spi_dev, uint8_t cs_pin, int8_t irq_pin) {
    if (spi_dev == NULL) {
        return false;
    }
    
    touch_spi = (spi_inst_t *)spi_dev;
    touch_cs_pin = cs_pin;
    touch_irq_pin = irq_pin;
    
    // Configure CS pin
    gpio_init(touch_cs_pin);
    gpio_set_dir(touch_cs_pin, GPIO_OUT);
    gpio_put(touch_cs_pin, 1);  // CS high (inactive)
    
    // Configure IRQ pin if provided
    if (touch_irq_pin >= 0) {
        gpio_init(touch_irq_pin);
        gpio_set_dir(touch_irq_pin, GPIO_IN);
        gpio_pull_up(touch_irq_pin);
        
        // Enable interrupt on both edges (press and release)
        gpio_set_irq_enabled_with_callback(touch_irq_pin, 
            GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
            true, &touch_irq_handler);
    }
    
    initialized = true;
    return true;
}

// Read a single 12-bit value from touch controller
static uint16_t xpt2046_read_value(uint8_t command) {
    if (!initialized) return 0;
    
    uint8_t tx_buf[3] = {command, 0x00, 0x00};
    uint8_t rx_buf[3] = {0};
    
    gpio_put(touch_cs_pin, 0);  // CS low (active)
    sleep_us(1);
    
    spi_write_read_blocking(touch_spi, tx_buf, rx_buf, 3);
    
    gpio_put(touch_cs_pin, 1);  // CS high (inactive)
    sleep_us(1);
    
    // Extract 12-bit value from response
    uint16_t value = ((rx_buf[1] << 8) | rx_buf[2]) >> 3;
    return value & 0x0FFF;
}

bool xpt2046_is_touched(void) {
    if (!initialized) return false;
    
    // If IRQ pin is connected, use it for fast detection
    if (touch_irq_pin >= 0) {
        return !gpio_get(touch_irq_pin);  // IRQ is active low
    }
    
    // Otherwise, do a quick Z read to check pressure
    uint16_t z = xpt2046_read_value(XPT2046_CMD_Z1);
    return (z > TOUCH_THRESHOLD);
}

bool xpt2046_read_raw(touch_point_t *point) {
    if (!initialized || point == NULL) {
        return false;
    }
    
    // Read Z1 and Z2 for pressure calculation
    uint16_t z1 = xpt2046_read_value(XPT2046_CMD_Z1);
    uint16_t z2 = xpt2046_read_value(XPT2046_CMD_Z2);
    
    // Calculate pressure (simplified formula)
    int32_t pressure = z1 + 4095 - z2;
    
    if (pressure < TOUCH_THRESHOLD) {
        point->valid = false;
        return false;
    }
    
    // Read position multiple times and average for stability
    uint32_t x_sum = 0, y_sum = 0;
    const int samples = 4;
    
    for (int i = 0; i < samples; i++) {
        x_sum += xpt2046_read_value(XPT2046_CMD_X);
        y_sum += xpt2046_read_value(XPT2046_CMD_Y);
    }
    
    point->x = x_sum / samples;
    point->y = y_sum / samples;
    point->pressure = (uint16_t)pressure;
    point->valid = true;
    
    return true;
}

void xpt2046_set_calibration(const touch_calibration_t *cal) {
    if (cal != NULL) {
        memcpy(&calibration, cal, sizeof(touch_calibration_t));
    }
}

void xpt2046_raw_to_screen(const touch_point_t *raw, uint16_t *screen_x, 
                           uint16_t *screen_y, uint16_t screen_width, 
                           uint16_t screen_height) {
    if (raw == NULL || !raw->valid) {
        *screen_x = 0;
        *screen_y = 0;
        return;
    }
    
    int32_t x = raw->x;
    int32_t y = raw->y;
    
    // Swap axes if needed
    if (calibration.swap_xy) {
        int32_t temp = x;
        x = y;
        y = temp;
    }
    
    // Map raw coordinates to screen coordinates
    x = ((x - calibration.x_min) * screen_width) / (calibration.x_max - calibration.x_min);
    y = ((y - calibration.y_min) * screen_height) / (calibration.y_max - calibration.y_min);
    
    // Invert axes if needed
    if (calibration.invert_x) {
        x = screen_width - x;
    }
    if (calibration.invert_y) {
        y = screen_height - y;
    }
    
    // Clamp to screen bounds
    if (x < 0) x = 0;
    if (x >= screen_width) x = screen_width - 1;
    if (y < 0) y = 0;
    if (y >= screen_height) y = screen_height - 1;
    
    *screen_x = (uint16_t)x;
    *screen_y = (uint16_t)y;
}

void xpt2046_set_callback(touch_callback_t callback) {
    touch_callback = callback;
}

// GPIO IRQ handler - called from interrupt context
static void touch_irq_handler(uint gpio, uint32_t events) {
    if (!initialized || gpio != touch_irq_pin) {
        return;
    }
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (events & GPIO_IRQ_EDGE_FALL) {
        // Touch pressed (IRQ goes low)
        touch_pressed = true;
        touch_press_time = now;
    } 
    else if (events & GPIO_IRQ_EDGE_RISE) {
        // Touch released (IRQ goes high)
        if (touch_pressed) {
            uint32_t duration = now - touch_press_time;
            
            // Only trigger if press was long enough (debounce) and not too long
            if (duration >= DEBOUNCE_MS && duration < LONG_PRESS_MS) {
                touch_event_pending = true;
                
                // Call callback if registered
                if (touch_callback) {
                    touch_callback();
                }
            }
            
            touch_pressed = false;
        }
    }
}
