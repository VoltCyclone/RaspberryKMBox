/*
 * Hurricane PIOKMBox Firmware
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/vreg.h"
#include "hardware/adc.h"
#include "pico/unique_id.h"

#include "tusb.h"
#include "defines.h"
#include "config.h"
#include "timing_config.h"
#include "led_control.h"
#include "usb_hid.h"
#include "watchdog.h"
#include "state_management.h"
#include "kmbox_serial_handler.h"
#include "smooth_injection.h"
#include "peri_clock.h"

#if PIO_USB_AVAILABLE
#include "pio_usb.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "tusb.h"
#endif

//--------------------------------------------------------------------+
// Type Definitions and Structures
//--------------------------------------------------------------------+

typedef struct {
    uint32_t watchdog_status_timer;
    uint32_t last_button_press_time;
    bool button_pressed_last;
    bool usb_reset_cooldown;
    uint32_t usb_reset_cooldown_start;
} main_loop_state_t;

// Remove unused structures
//typedef struct {
//    bool button_pressed;
//    uint32_t current_time;
//    uint32_t hold_duration;
//} button_state_t;

//--------------------------------------------------------------------+
// Constants and Configuration
//--------------------------------------------------------------------+
static const uint32_t WATCHDOG_STATUS_INTERVAL_MS = WATCHDOG_STATUS_REPORT_INTERVAL_MS;

//--------------------------------------------------------------------+
// Global state for flash operation coordination
//--------------------------------------------------------------------+
volatile bool g_flash_operation_in_progress = false;
volatile bool g_core1_flash_acknowledged = false;

//--------------------------------------------------------------------+
// Function Prototypes
//--------------------------------------------------------------------+

#if PIO_USB_AVAILABLE
static void core1_main(void);
static void core1_task_loop(void);
#endif

static bool initialize_system(void);
static bool initialize_usb_device(void);
static void main_application_loop(void);

// Button handling functions
static void process_button_input(system_state_t* state, uint32_t current_time);

// Reporting functions
static void report_watchdog_status(uint32_t current_time, uint32_t* watchdog_status_timer);

// Utility functions
static inline bool is_time_elapsed(uint32_t current_time, uint32_t last_time, uint32_t interval);

//--------------------------------------------------------------------+
// Core1 Main (USB Host Task)
//--------------------------------------------------------------------+

#if PIO_USB_AVAILABLE
// Separate initialization concerns into focused functions

typedef enum {
    INIT_SUCCESS,
    INIT_FAILURE,
    INIT_RETRY_NEEDED
} init_result_t;

typedef struct {
    int attempt;
    int max_attempts;
    uint32_t base_delay_ms;
    uint32_t last_heartbeat_time;
} init_context_t;

typedef struct {
    uint32_t last_heartbeat_ms;
    uint32_t heartbeat_counter;
} core1_state_t;


static void core1_main(void) {
    // Small delay to let core0 stabilize
    sleep_ms(10);
    
    // Initialize flash_safe_execute support on Core1.
    // This allows Core0's flash_safe_execute() to safely pause Core1
    // during flash operations (replaces manual g_flash_operation_in_progress polling).
    flash_safe_execute_core_init();
    
    // CRITICAL: Configure PIO USB BEFORE tuh_init() - this is the key!
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = PIN_USB_HOST_DP;
    pio_cfg.pinout = PIO_USB_PINOUT_DPDM;
    
    // Configure host stack with PIO USB configuration
    tuh_configure(USB_HOST_PORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    
    // CRITICAL: Use Report protocol by default (not Boot protocol).
    // Boot protocol is simpler but many devices (Logitech receivers, gaming mice)
    // only work correctly in Report protocol mode with proper report descriptors.
    // This must be set BEFORE tuh_init().
    tuh_hid_set_default_protocol(HID_PROTOCOL_REPORT);
    
    // Initialize host stack on core1
    tuh_init(USB_HOST_PORT);
    
    // Mark host as initialized
    usb_host_mark_initialized();
    
    // Start the main host task loop
    core1_task_loop();
}

static void core1_task_loop(void) {
    // Optimize heartbeat checking - use larger counter intervals
    uint32_t heartbeat_counter = 0;
    uint32_t last_heartbeat_ms = 0;
    
    // Performance optimization: reduce heartbeat frequency checks
    const uint32_t heartbeat_check_threshold = CORE1_HEARTBEAT_CHECK_LOOPS * 4; // 4x less frequent
    
    while (true) {
        // NOTE: Flash safety is handled automatically by flash_safe_execute_core_init().
        // The SDK's multicore_lockout mechanism pauses this core via SIO FIFO IRQ
        // when Core0 calls flash_safe_execute(), so no manual polling needed.
        
        tuh_task();
        
        // Heartbeat check optimization - much less frequent timing calls
        if (++heartbeat_counter >= heartbeat_check_threshold) {
            const uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if ((current_time - last_heartbeat_ms) >= WATCHDOG_HEARTBEAT_INTERVAL_MS) {
                watchdog_core1_heartbeat();
                last_heartbeat_ms = current_time;
            }
            heartbeat_counter = 0;
        }
    }
}

#endif // PIO_USB_AVAILABLE
//--------------------------------------------------------------------+
// System Initialization Functions
//--------------------------------------------------------------------+

static bool initialize_system(void) {
    // Initialize stdio first for early debug output
    stdio_init_all();
    
    // Add startup delay for cold boot stability
    sleep_ms(200);
    
    // Overclock RP2350 to 240MHz, increase VREG voltage to 1.25V
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    sleep_ms(10);  // Let voltage stabilize
    
    if (!set_sys_clock_khz(CPU_FREQ, true)) {
        return false;
    }
    
    // CRITICAL: Configure stable peripheral clock BEFORE UART init
    // This must happen before kmbox_serial_init() so uart_init() calculates correct divisor
    peri_clock_configure_stable();
    
    // Re-initialize stdio after clock change with proper delay
    sleep_ms(100);  // Allow clock to stabilize
    stdio_init_all();  // No-op when both UART and USB stdio are disabled
    sleep_ms(100);  // Allow system to stabilize
    
    // Initialize KMBox serial handler on UART0 (via RP2350 USB Bridge)
    // UART will now use the stable 48MHz peri clock for accurate 2 Mbaud
    kmbox_serial_init();
    
    // Initialize ADC for temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);
    
    // Initialize smooth injection system for seamless mouse movement blending
    smooth_injection_init();
    
    // Initialize LED control module (neopixel power OFF for now)
    neopixel_init();

    // Initialize USB HID module (USB host power OFF for now)
    usb_hid_init();

    // Initialize watchdog system (but don't start it yet)
    watchdog_init();

    // Initialization complete - proceed to USB init
    return true;
}

static bool initialize_usb_device(void) {
    const bool device_init_success = tud_init(USB_DEVICE_PORT);
    
    if (device_init_success) {
        usb_device_mark_initialized();
    }
    
    return device_init_success;
}



//--------------------------------------------------------------------+
// Button Handling Functions
//--------------------------------------------------------------------+

static void process_button_input(system_state_t* state, uint32_t current_time) {
    // Performance optimization: single GPIO read per call
    const bool button_currently_pressed = !gpio_get(PIN_BUTTON); // Button is active low

    // Handle cooldown after USB reset - early exit for performance
    if (state->usb_reset_cooldown) {
        if (is_time_elapsed(current_time, state->usb_reset_cooldown_start, USB_RESET_COOLDOWN_MS)) {
            state->usb_reset_cooldown = false;
        }
        state->button_pressed_last = button_currently_pressed;
        return; // Skip button processing during cooldown
    }

    // Optimized state machine - avoid redundant checks
    if (button_currently_pressed) {
        if (!state->button_pressed_last) {
            // Button just pressed
            state->last_button_press_time = current_time;
        } else {
            // Button being held - check for reset trigger
                if (is_time_elapsed(current_time, state->last_button_press_time, BUTTON_HOLD_TRIGGER_MS)) {
                    usb_stacks_reset();
                    state->usb_reset_cooldown = true;
                    state->usb_reset_cooldown_start = current_time;
                }
        }
    } else if (state->button_pressed_last) {
        // Button just released - check if it was a short press
        uint32_t hold_duration = current_time - state->last_button_press_time;
        
        if (hold_duration < BUTTON_HOLD_TRIGGER_MS) {
            // Short press - cycle humanization mode
            humanization_mode_t new_mode = smooth_cycle_humanization_mode();
            
            // Send immediate notification to bridge
            kmbox_send_info_to_bridge();
            
            // Show mode with LED flash
            uint32_t mode_color;
            switch (new_mode) {
                case HUMANIZATION_OFF:
                    mode_color = COLOR_HUMANIZATION_OFF;
                    
                    break;
                case HUMANIZATION_LOW:
                    mode_color = COLOR_HUMANIZATION_LOW;
                    
                    break;
                case HUMANIZATION_MEDIUM:
                    mode_color = COLOR_HUMANIZATION_MEDIUM;
                    
                    break;
                case HUMANIZATION_HIGH:
                    mode_color = COLOR_HUMANIZATION_HIGH;
                    
                    break;
                default:
                    mode_color = COLOR_ERROR;
                    
                    break;
            }
            
            neopixel_set_color(mode_color);
            neopixel_trigger_mode_flash(mode_color, 1500);
        }
    }

    state->button_pressed_last = button_currently_pressed;
}

//--------------------------------------------------------------------+
// Reporting Functions
//--------------------------------------------------------------------+

static void report_watchdog_status(uint32_t current_time, uint32_t* watchdog_status_timer) {
    if (!is_time_elapsed(current_time, *watchdog_status_timer, WATCHDOG_STATUS_INTERVAL_MS)) {
        return;
    }

    *watchdog_status_timer = current_time;
    
    
}

//--------------------------------------------------------------------+
// Utility Functions
//--------------------------------------------------------------------+

static __force_inline bool is_time_elapsed(uint32_t current_time, uint32_t last_time, uint32_t interval) {
    return (current_time - last_time) >= interval;
}

//--------------------------------------------------------------------+
// Main Application Loop
//--------------------------------------------------------------------+


static void main_application_loop(void) {
    system_state_t* state = get_system_state();
    system_state_init(state);
    
    // Cache frequently used intervals
    const uint32_t watchdog_interval = WATCHDOG_TASK_INTERVAL_MS;
    const uint32_t visual_interval = VISUAL_TASK_INTERVAL_MS;
    const uint32_t error_interval = ERROR_CHECK_INTERVAL_MS;
    const uint32_t button_interval = BUTTON_DEBOUNCE_MS;
    const uint32_t status_report_interval = WATCHDOG_STATUS_REPORT_INTERVAL_MS;

    // Performance optimization: reduce time sampling frequency
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint16_t loop_counter = 0;
    
    // Initialize DMA after a delay to ensure USB is fully stable
    bool dma_initialized = false;
    uint32_t boot_complete_time = current_time;
    
    // Batch time checks with bit flags for efficiency
    uint8_t task_flags = 0;
    #define WATCHDOG_FLAG   (1 << 0)
    #define VISUAL_FLAG     (1 << 1)
    #define BUTTON_FLAG     (1 << 2)
    #define STATUS_FLAG     (1 << 3)

    while (true) {
        // TinyUSB device task - highest priority
        tud_task();
        
        // Initialize DMA after 3 seconds of stable USB operation
        if (!dma_initialized && (current_time - boot_complete_time) > 3000) {
            kmbox_serial_init_dma();
            dma_initialized = true;
        }
        
        // KMBox serial task - process bridge commands
        // Bridge movements are added to the shared accumulator and combined
        // with physical mouse movements automatically
        kmbox_serial_task();
        
        // HID device task - processes physical mouse/keyboard and sends combined reports
        hid_device_task();
        
        // Sample time less frequently to reduce overhead
        if (++loop_counter >= MAIN_LOOP_TIME_SAMPLE_INTERVAL) {
            current_time = to_ms_since_boot(get_absolute_time());
            loop_counter = 0;
            
            // Batch all time checks into flags for efficiency
            task_flags = 0;
            if ((current_time - state->last_watchdog_time) >= watchdog_interval) {
                task_flags |= WATCHDOG_FLAG;
            }
            if ((current_time - state->last_visual_time) >= visual_interval) {
                task_flags |= VISUAL_FLAG;
            }
            if ((current_time - state->last_button_time) >= button_interval) {
                task_flags |= BUTTON_FLAG;
            }
            if ((current_time - state->watchdog_status_timer) >= status_report_interval) {
                task_flags |= STATUS_FLAG;
            }
            
            // Error check optimization - only when other tasks run
            if (task_flags && (current_time - state->last_error_check_time) >= error_interval) {
                state->last_error_check_time = current_time;
            }
        }
        
        // Execute tasks based on flags (avoids repeated time checks)
        if (task_flags & WATCHDOG_FLAG) {
            watchdog_task();
            watchdog_core0_heartbeat();
            state->last_watchdog_time = current_time;
        }
        
        if (task_flags & BUTTON_FLAG) {
            process_button_input(state, current_time);
            state->last_button_time = current_time;
        }
        
        if (task_flags & VISUAL_FLAG) {
            led_blinking_task();
            neopixel_status_task();
            state->last_visual_time = current_time;
        }
        
        if (task_flags & STATUS_FLAG) {
            report_watchdog_status(current_time, &state->watchdog_status_timer);
            
            // Process deferred flash saves (checks internally if needed)
            // Run at status report interval (~10s) since flash save is deferred anyway
            smooth_process_deferred_save();
        }
    }
}

//--------------------------------------------------------------------+
// Main Function
//--------------------------------------------------------------------+

int main(void) {
    
    // Initialize basic GPIO (clock will be set by initialize_system)
    #ifdef PIN_USB_5V
    gpio_init(PIN_USB_5V);
    gpio_set_dir(PIN_USB_5V, GPIO_OUT);
    gpio_put(PIN_USB_5V, 0);  // Keep USB power OFF initially
    #endif
    
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);  // Turn on LED
    
    printf("=== PIOKMBox Starting ===\n");
    
    // Initialize system components (includes clock configuration)
    if (!initialize_system()) {
        printf("CRITICAL: System initialization failed\n");
        return -1;
    }
    
    // Enable USB host power
    usb_host_enable_power();
    sleep_ms(100);  // Brief power stabilization
    
#if PIO_USB_AVAILABLE
    multicore_reset_core1();
    multicore_launch_core1(core1_main);
    
    // Give core1 time to initialize before USB device init
    sleep_ms(100);
#endif
    
    if (!initialize_usb_device()) {
        printf("CRITICAL: USB Device initialization failed\n");
        return -1;
    }
    
    neopixel_enable_power();    
    printf("=== PIOKMBox Ready ===\n");
    
    // Enter main application loop
    main_application_loop();
    
    return 0;
}
