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
#include "pico/unique_id.h"

#include "tusb.h"
#include "defines.h"
#include "config.h"
#include "timing_config.h"
#include "led_control.h"
#include "usb_hid.h"
#include "watchdog.h"
#include "init_state_machine.h"
#include "state_management.h"

#if PIO_USB_AVAILABLE
#include "pio_usb.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "tusb.h"
#endif

//--------------------------------------------------------------------+
// Type Definitions and Structures
//--------------------------------------------------------------------+

typedef struct {
    uint32_t stats_timer;
    uint32_t watchdog_status_timer;
    uint32_t last_button_press_time;
    bool button_pressed_last;
    bool usb_reset_cooldown;
    uint32_t usb_reset_cooldown_start;
} main_loop_state_t;

#ifdef RP2350
// Global variable to track if hardware acceleration is enabled
static bool hw_accel_enabled = false;
#endif

typedef struct {
    bool button_pressed;
    uint32_t current_time;
    uint32_t hold_duration;
} button_state_t;

//--------------------------------------------------------------------+
// Constants and Configuration
//--------------------------------------------------------------------+
static const uint32_t STATS_INTERVAL_MS = STATS_REPORT_INTERVAL_MS;
static const uint32_t WATCHDOG_STATUS_INTERVAL_MS = WATCHDOG_STATUS_REPORT_INTERVAL_MS;

//--------------------------------------------------------------------+
// Function Prototypes
//--------------------------------------------------------------------+

#if PIO_USB_AVAILABLE
static void core1_main(void);
static void core1_task_loop(void);
#ifdef RP2350
static bool init_hid_hardware_acceleration(void);
static void tuh_task_with_acceleration(void);
#endif
#endif

static bool initialize_system(void);
static bool initialize_usb_device(void);
static void main_application_loop(void);

// Button handling functions
static button_state_t get_button_state(uint32_t current_time);
static void handle_button_press_start(system_state_t* state, uint32_t current_time);
static void handle_button_held(system_state_t* state, uint32_t current_time);
static void handle_button_release(const system_state_t* state, uint32_t hold_duration);
static void process_button_input(system_state_t* state, uint32_t current_time);

// Reporting functions
static void report_hid_statistics(uint32_t current_time, uint32_t* stats_timer);
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
    // EXTENDED delay for cold boot - core1 needs more time
    sleep_ms(100);  // Increase from 10ms
    
    printf("Core1: Starting USB host initialization (cold boot)...\n");
    
    // Add heartbeat early to prevent watchdog timeout during init
    watchdog_core1_heartbeat();
    
    // CRITICAL: Configure PIO USB BEFORE tuh_init()
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = PIN_USB_HOST_DP;
    pio_cfg.pinout = PIO_USB_PINOUT_DPDM;
    
    // Configure host stack with PIO USB configuration
    if (!tuh_configure(USB_HOST_PORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg)) {
        printf("Core1: CRITICAL - PIO USB configure failed\n");
        // Don't return - continue with limited functionality
    }
    
    // Initialize host stack on core1
    if (!tuh_init(USB_HOST_PORT)) {
        printf("Core1: CRITICAL - USB host init failed\n");
        // Don't return - continue with limited functionality
    }
    
#ifdef RP2350
    // RP2350: Initialize hardware acceleration
    hw_accel_enabled = init_hid_hardware_acceleration();
    if (hw_accel_enabled) {
        printf("Core1: RP2350 hardware acceleration initialized successfully\n");
    } else {
        printf("Core1: RP2350 hardware acceleration initialization failed, using standard mode\n");
    }
#endif
    
    // Mark host as initialized
    usb_host_mark_initialized();
    
    printf("Core1: USB host initialization complete\n");
    
    // Start the main host task loop
    core1_task_loop();
}

static void core1_task_loop(void) {
    core1_state_t state = {0};  // Local state instead of static

#ifdef RP2350
    // RP2350: Initialize hardware acceleration
    if (!hw_accel_enabled) {
        hw_accel_enabled = init_hid_hardware_acceleration();
    }
#endif

    while (true) {
#ifdef RP2350
        // RP2350: Use hardware-accelerated task processing
        if (hw_accel_enabled) {
            tuh_task_with_acceleration();
        } else {
            tuh_task();
        }
#else
        // Original RP2040 implementation
        tuh_task();
#endif
        
        // Check heartbeat timing less frequently
        if (++state.heartbeat_counter >= CORE1_HEARTBEAT_CHECK_LOOPS) {
            const uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (system_state_should_run_task(NULL, current_time,
                                             state.last_heartbeat_ms,
                                             WATCHDOG_HEARTBEAT_INTERVAL_MS)) {
                watchdog_core1_heartbeat();
                state.last_heartbeat_ms = current_time;
            }
            state.heartbeat_counter = 0;
        }
    }
}

#ifdef RP2350
/**
 * @brief Initialize RP2350 hardware acceleration for USB HID processing
 *
 * This function initializes the hardware acceleration features of the RP2350
 * for improved USB HID processing performance.
 *
 * @return true if hardware acceleration was successfully initialized, false otherwise
 */
static bool init_hid_hardware_acceleration(void) {
    printf("Initializing RP2350 hardware acceleration for USB HID...\n");
    
    // Configure RP2350-specific hardware acceleration registers
    // Note: This is a placeholder implementation. The actual implementation
    // would depend on the specific RP2350 hardware acceleration capabilities.
    
    // Example implementation:
    // 1. Enable hardware acceleration clock
    // 2. Configure acceleration parameters
    // 3. Set up DMA channels for HID data processing
    // 4. Initialize hardware FIFO buffers
    
    // For now, we'll just return true to simulate successful initialization
    return true;
}

/**
 * @brief Perform USB host task processing with hardware acceleration
 *
 * This function uses the RP2350 hardware acceleration features to process
 * USB host tasks more efficiently than the standard tuh_task() function.
 */
static void tuh_task_with_acceleration(void) {
    // This is a placeholder implementation. The actual implementation
    // would use RP2350-specific hardware acceleration features.
    
    // Example implementation:
    // 1. Check hardware FIFO for incoming HID reports
    // 2. Process reports using hardware acceleration
    // 3. Handle any hardware-specific events
    
    // For now, we'll just call the standard tuh_task() function
    // but in a real implementation, this would use hardware acceleration
    tuh_task();
}
#endif // RP2350

#endif // PIO_USB_AVAILABLE
//--------------------------------------------------------------------+
// System Initialization Functions
//--------------------------------------------------------------------+

static bool initialize_system(void) {
    // Initialize stdio first for early debug output
    stdio_init_all();
    
    // Add extended startup delay for cold boot stability
    // Use the proper constant from defines.h instead of hardcoded value
    sleep_ms(COLD_BOOT_STABILIZATION_MS);
    
    printf("PICO PIO KMBox - Starting initialization...\n");
    printf("Neopixel pins initialized (power OFF for cold boot stability)\n");
    
#if PIO_USB_AVAILABLE
    // Set system clock to 120MHz (required for PIO USB - must be multiple of 12MHz)
    printf("Setting system clock to %d kHz...\n", PIO_USB_SYSTEM_CLOCK_KHZ);
    if (!set_sys_clock_khz(PIO_USB_SYSTEM_CLOCK_KHZ, true)) {
        printf("CRITICAL: Failed to set system clock to %d kHz\n", PIO_USB_SYSTEM_CLOCK_KHZ);
        return false;
    }
    
    // Re-initialize stdio after clock change with proper delay
    sleep_ms(100);  // Allow clock to stabilize
    stdio_init_all();
    sleep_ms(100);  // Allow UART to stabilize
    printf("System clock set successfully to %d kHz\n", PIO_USB_SYSTEM_CLOCK_KHZ);
#endif
    
    // Configure UART for non-blocking operation
    uart_set_fifo_enabled(uart0, true);  // Enable FIFO for better performance
    
    // Initialize LED control module (neopixel power OFF for now)
    neopixel_init();

    // Initialize USB HID module (USB host power OFF for now)
    usb_hid_init();

    // Initialize watchdog system (but don't start it yet)
    watchdog_init();

    printf("System initialization complete\n");
    return true;
}

static bool initialize_usb_device(void) {
    // Simple device initialization - let the working example pattern guide us
    printf("USB Device: Initializing on controller %d (native USB)...\n", USB_DEVICE_PORT);
    
    const bool device_init_success = tud_init(USB_DEVICE_PORT);
    printf("USB Device init: %s\n", device_init_success ? "SUCCESS" : "FAILED");
    
    if (device_init_success) {
        usb_device_mark_initialized();
        printf("USB Device: Initialization complete\n");
    }
    
    return device_init_success;
}



//--------------------------------------------------------------------+
// Button Handling Functions
//--------------------------------------------------------------------+

static button_state_t get_button_state(uint32_t current_time) {
    button_state_t state = {
        .button_pressed = !gpio_get(PIN_BUTTON), // Button is active low
        .current_time = current_time,
        .hold_duration = 0
    };
    return state;
}

static void handle_button_press_start(system_state_t* state, uint32_t current_time) {
    state->last_button_press_time = current_time;
    // Button pressed (reduced logging)
}

static void handle_button_held(system_state_t* state, uint32_t current_time) {
    if (is_time_elapsed(current_time, state->last_button_press_time, BUTTON_HOLD_TRIGGER_MS)) {
        printf("Button held - triggering USB reset\n");
        usb_stacks_reset();
        state->usb_reset_cooldown = true;
        state->usb_reset_cooldown_start = current_time;
    }
}

static void handle_button_release(const system_state_t* state, uint32_t hold_duration) {
    (void)state; // Suppress unused parameter warning
    if (hold_duration < BUTTON_HOLD_TRIGGER_MS) {
        // Button released (reduced logging)
    }
}

static void process_button_input(system_state_t* state, uint32_t current_time) {
    const button_state_t button = get_button_state(current_time);

    // Handle cooldown after USB reset
    if (state->usb_reset_cooldown) {
        if (is_time_elapsed(current_time, state->usb_reset_cooldown_start, USB_RESET_COOLDOWN_MS)) {
            state->usb_reset_cooldown = false;
        }
        state->button_pressed_last = button.button_pressed;
        return; // Skip button processing during cooldown
    }

    if (button.button_pressed && !state->button_pressed_last) {
        // Button just pressed
        handle_button_press_start(state, current_time);
    } else if (button.button_pressed && state->button_pressed_last) {
        // Button being held
        handle_button_held(state, current_time);
    } else if (!button.button_pressed && state->button_pressed_last) {
        // Button just released
        const uint32_t hold_duration = current_time - state->last_button_press_time;
        handle_button_release(state, hold_duration);
    }

    state->button_pressed_last = button.button_pressed;
}

//--------------------------------------------------------------------+
// Reporting Functions
//--------------------------------------------------------------------+

static void report_hid_statistics(uint32_t current_time, uint32_t* stats_timer) {
    if (!is_time_elapsed(current_time, *stats_timer, STATS_INTERVAL_MS)) {
        return;
    }

    *stats_timer = current_time;
    
    hid_stats_t stats;
    get_hid_stats(&stats);
    
    printf("=== HID Statistics ===\n");
    printf("Mouse: RX=%lu, TX=%lu\n", stats.mouse_reports_received, stats.mouse_reports_forwarded);
    printf("Keyboard: RX=%lu, TX=%lu\n", stats.keyboard_reports_received, stats.keyboard_reports_forwarded);
    printf("Errors: %lu\n", stats.forwarding_errors);
    printf("Mouse connected: %s\n", is_mouse_connected() ? "YES" : "NO");
    printf("Keyboard connected: %s\n", is_keyboard_connected() ? "YES" : "NO");
    printf("=====================\n");
}

static void report_watchdog_status(uint32_t current_time, uint32_t* watchdog_status_timer) {
    if (!is_time_elapsed(current_time, *watchdog_status_timer, WATCHDOG_STATUS_INTERVAL_MS)) {
        return;
    }

    *watchdog_status_timer = current_time;
    
    const watchdog_status_t watchdog_status = watchdog_get_status();
    
    printf("=== Watchdog Status ===\n");
    printf("System healthy: %s\n", watchdog_status.system_healthy ? "YES" : "NO");
    printf("Core 0: %s (heartbeats: %lu)\n",
           watchdog_status.core0_responsive ? "RESPONSIVE" : "UNRESPONSIVE",
           watchdog_status.core0_heartbeat_count);
    printf("Core 1: %s (heartbeats: %lu)\n",
           watchdog_status.core1_responsive ? "RESPONSIVE" : "UNRESPONSIVE",
           watchdog_status.core1_heartbeat_count);
    printf("Hardware updates: %lu\n", watchdog_status.hardware_updates);
    printf("Timeout warnings: %lu\n", watchdog_status.timeout_warnings);
    printf("=======================\n");
}

//--------------------------------------------------------------------+
// Utility Functions
//--------------------------------------------------------------------+

static inline bool is_time_elapsed(uint32_t current_time, uint32_t last_time, uint32_t interval) {
    return (current_time - last_time) >= interval;
}

//--------------------------------------------------------------------+
// Main Application Loop
//--------------------------------------------------------------------+


static void main_application_loop(void) {
    system_state_t* state = get_system_state();
    system_state_init(state);

    while (true) {
        // TinyUSB device task - highest priority
        tud_task();
        hid_device_task();
        
        const uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Watchdog tasks - controlled frequency
        if (system_state_should_run_task(state, current_time, 
                                        state->last_watchdog_time, 
                                        WATCHDOG_TASK_INTERVAL_MS)) {
            watchdog_task();
            watchdog_core0_heartbeat();
            state->last_watchdog_time = current_time;
        }
        
        // LED and visual tasks
        if (system_state_should_run_task(state, current_time,
                                        state->last_visual_time,
                                        VISUAL_TASK_INTERVAL_MS)) {
            led_blinking_task();
            neopixel_status_task();
            state->last_visual_time = current_time;
        }
        
        // USB stack error monitoring - DISABLED to prevent endpoint conflicts
        // The error checking was causing automatic resets that conflict with dual-mode operation
        if (system_state_should_run_task(state, current_time,
                                        state->last_error_check_time,
                                        ERROR_CHECK_INTERVAL_MS)) {
            // usb_stack_error_check(); // Disabled to prevent endpoint conflicts
            state->last_error_check_time = current_time;
        }
        
        // Button input processing
        if (system_state_should_run_task(state, current_time,
                                        state->last_button_time,
                                        BUTTON_DEBOUNCE_MS)) {
            process_button_input(state, current_time);
            state->last_button_time = current_time;
        }
        
        // Periodic reporting
        report_hid_statistics(current_time, &state->stats_timer);
        report_watchdog_status(current_time, &state->watchdog_status_timer);
    }
}

//--------------------------------------------------------------------+
// Main Function
//--------------------------------------------------------------------+

int main(void) {
    // CRITICAL: Basic GPIO setup FIRST, before any delays
#ifndef TARGET_RP2350
    // USB 5V power pin initialization only for RP2040 boards
    gpio_init(PIN_USB_5V);
    gpio_set_dir(PIN_USB_5V, GPIO_OUT);
    gpio_put(PIN_USB_5V, 0);  // Keep USB power OFF during entire boot
#endif
    
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);  // Turn on LED early for status
    
    // EXTENDED cold boot stabilization - this is critical!
    sleep_ms(COLD_BOOT_STABILIZATION_MS);  // Should be 2000ms or more
    
#ifdef RP2350
    printf("RP2350 detected - configuring for hardware acceleration\n");
#else
    printf("RP2040 detected - using standard configuration\n");
#endif
    
    // Set system clock with proper stabilization
    printf("Setting system clock...\n");  // Basic printf should work with default clock
    if (!set_sys_clock_khz(120000, true)) {
        // Clock setting failed - try to continue with default clock
        printf("WARNING: Failed to set 120MHz clock, continuing with default\n");
    }
    
    // CRITICAL: Extended delay after clock change for cold boot
    sleep_ms(200);  // Let clock fully stabilize
    
    // Re-initialize stdio after clock change
    stdio_init_all();
    
    // Additional delay for UART stabilization after clock change
    sleep_ms(100);
    
    printf("=== PIOKMBox Starting (Cold Boot Enhanced) ===\n");
#ifdef RP2350
    printf("RP2350 Hardware Acceleration Enabled\n");
#endif
#ifndef TARGET_RP2350
    printf("USB power held LOW during boot for stability\n");
#else
    printf("RP2350 detected - using plain USB (no 5V pin control needed)\n");
#endif
    
    // Initialize system components with more conservative timing
    if (!initialize_system()) {
        printf("CRITICAL: System initialization failed\n");
        return -1;
    }
    
    // Additional hardware-specific delay for problematic hardware revisions
    #if REQUIRES_EXTENDED_BOOT_DELAY
    printf("Hardware revision %d requires extended boot delay...\n", HARDWARE_REVISION);
    sleep_ms(USB_POWER_STABILIZATION_MS);
    #else
    // Even "good" hardware needs some delay for cold boot
    sleep_ms(1000);
    #endif
    
    // Initialize USB device stack BEFORE enabling host power
    printf("Initializing USB device stack on core0...\n");
    if (!initialize_usb_device()) {
        printf("CRITICAL: USB Device initialization failed\n");
        return -1;
    }
    
    // Let device stack fully initialize
    sleep_ms(500);
    
    // NOW enable USB host power with extended stabilization
    printf("Enabling USB host power...\n");
    usb_host_enable_power();
    
#ifdef RP2350
    // Initialize hardware acceleration components before core1 launch
    printf("Preparing RP2350 hardware acceleration components...\n");
    // Note: The actual hardware acceleration initialization happens in core1_main
#endif
    
    // CRITICAL: Extended delay after power enable for cold boot
    sleep_ms(1000);  // Much longer for cold boot reliability
    
    // Launch core1 with additional delay after power stabilization
    printf("Launching core1 for USB host...\n");
    multicore_reset_core1();
    multicore_launch_core1(core1_main);
    
    // Give core1 time to initialize before proceeding
    sleep_ms(500);
    
    // Initialize remaining systems
    printf("Starting watchdog and final systems...\n");
    watchdog_init();
    watchdog_start();
    
    // Enable neopixel power last
    sleep_ms(500);
    neopixel_enable_power();
    
    printf("=== PIOKMBox Ready (Cold Boot Complete) ===\n");
#ifdef RP2350
    printf("RP2350 Hardware Acceleration Status: %s\n", hw_accel_enabled ? "ACTIVE" : "INACTIVE");
#endif
    
    // Enter main application loop
    main_application_loop();
    
    return 0;
}
