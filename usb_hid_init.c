/*
 * Hurricane PIOKMbox Firmware
 */

#include "usb_hid_init.h"
#include "defines.h"
#include "led_control.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "usb_hid_reports.h"
#include "usb_hid_stats.h"
#include <stdio.h>
#include <string.h>

// Function declarations
static bool generate_serial_string(void);
static bool init_gpio_pins(void);
void init_synchronization(void);

// External declarations for variables defined in other modules
extern device_connection_state_t connection_state;
extern char serial_string[SERIAL_STRING_BUFFER_SIZE];

// USB stack initialization tracking
static bool usb_device_initialized = false;
static bool usb_host_initialized = false;

// Error tracking constants
#define MAX_CONSECUTIVE_ERRORS 5  // Maximum consecutive errors before reset

bool usb_hid_init(void)
{
    // Initialize critical sections for thread safety
    init_synchronization();
    
    // Generate unique serial string from chip ID
    if (!generate_serial_string()) {
        LOG_ERROR("Failed to generate serial string");
        return false;
    }
    
    // Initialize GPIO pins
    if (!init_gpio_pins()) {
        LOG_ERROR("Failed to initialize GPIO pins");
        return false;
    }
    
    // Initialize connection state
    memset(&connection_state, 0, sizeof(connection_state));
    
    // Explicitly initialize VID/PID fields to zero
    connection_state.mouse_vid = 0;
    connection_state.mouse_pid = 0;
    connection_state.keyboard_vid = 0;
    connection_state.keyboard_pid = 0;
    connection_state.descriptors_updated = false;
    connection_state.strings_updated = false;
    
    // Initialize string descriptor buffers with default values
    strncpy(connection_state.manufacturer, MANUFACTURER_STRING, USB_STRING_BUFFER_SIZE - 1);
    strncpy(connection_state.product, PRODUCT_STRING, USB_STRING_BUFFER_SIZE - 1);
    strncpy(connection_state.serial, serial_string, USB_STRING_BUFFER_SIZE - 1);
    
    printf("USB HID module initialized successfully\n");
    return true;
}

static bool generate_serial_string(void)
{
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    
    // Board ID is always valid since it's an array, not a pointer
    // No validation needed for board_id.id
    
    // Convert 8 bytes of unique ID to 16 character hex string
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        int result = snprintf(&serial_string[i * SERIAL_HEX_CHARS_PER_BYTE], 
                             SERIAL_SNPRINTF_BUFFER_SIZE, 
                             "%02X", 
                             board_id.id[i]);
        if (result < 0 || result >= SERIAL_SNPRINTF_BUFFER_SIZE) {
            return false;
        }
    }
    
    serial_string[SERIAL_STRING_LENGTH] = '\0'; // Ensure null termination
    return true;
}

static bool init_gpio_pins(void)
{
    // Initialize button pin
    gpio_init(PIN_BUTTON);
    
    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_BUTTON);
    
#ifndef RP2350
    // USB 5V power pin initialization but keep it OFF during early boot
    // Only for RP2040 boards, not needed for RP2350 which uses plain USB
    gpio_init(PIN_USB_5V);
    gpio_set_dir(PIN_USB_5V, GPIO_OUT);
    gpio_put(PIN_USB_5V, 0); // Keep USB power OFF initially for cold boot stability
#endif
    
    return true;
}

/**
 * @brief Initialize critical sections for thread safety
 *
 * This function initializes the critical sections used for protecting
 * shared resources in the USB HID module.
 */
void init_synchronization(void) {
    critical_section_init(&usb_state_lock);
    critical_section_init(&stats_lock);
    LOG_INIT("Critical sections initialized for thread safety");
}

bool usb_host_enable_power(void)
{
    LOG_INIT("Enabling USB host power...");
#ifndef RP2350
    // Only control the 5V power pin on RP2040 boards
    gpio_put(PIN_USB_5V, 1); // Enable USB power
    sleep_ms(100); // Allow power to stabilize
    LOG_INIT("USB host power enabled on pin %d", PIN_USB_5V);
#else
    // RP2350 uses plain USB, no need to control 5V pin
    LOG_INIT("RP2350 detected - using plain USB (no 5V pin control needed)");
#endif
    return true;
}

void usb_device_mark_initialized(void)
{
    usb_device_initialized = true;
}

void usb_host_mark_initialized(void)
{
    usb_host_initialized = true;
}

// USB stack reset functions
bool usb_device_stack_reset(void)
{
    LOG_INIT("Resetting USB device stack...");
    
    // Reset device stack state
    usb_device_initialized = false;
    
    // Reinitialize the device stack
    bool success = tud_init(USB_DEVICE_PORT);
    
    if (success) {
        usb_device_mark_initialized();
        LOG_INIT("USB device stack reset successful");
    } else {
        LOG_ERROR("USB device stack reset failed");
    }
    
    return success;
}

bool usb_host_stack_reset(void)
{
    LOG_INIT("Resetting USB host stack...");
    
    // Reset host stack state
    usb_host_initialized = false;
    
    // Power cycle the USB host port
#ifndef RP2350
    // Only power cycle on RP2040 boards
    gpio_put(PIN_USB_5V, 0);
    sleep_ms(500); // Wait for power to fully discharge
    gpio_put(PIN_USB_5V, 1);
    sleep_ms(500); // Wait for power to stabilize
#else
    // RP2350 uses plain USB, no need to power cycle
    LOG_INIT("RP2350 detected - skipping 5V power cycle (using plain USB)");
    sleep_ms(500); // Still wait for stability
#endif
    
    // Reinitialize the host stack
    bool success = tuh_init(USB_HOST_PORT);
    
    if (success) {
        usb_host_mark_initialized();
        LOG_INIT("USB host stack reset successful");
    } else {
        LOG_ERROR("USB host stack reset failed");
    }
    
    return success;
}

bool usb_stacks_reset(void)
{
    LOG_INIT("Resetting both USB stacks...");
    
    // Reset both device and host stacks
    bool device_reset = usb_device_stack_reset();
    bool host_reset = usb_host_stack_reset();
    
    return device_reset && host_reset;
}

bool usb_hid_dma_init(void)
{
    LOG_INIT("Initializing DMA for USB HID report processing...");
    
#ifdef RP2350
    // Initialize DMA channels and circular buffers for HID reports
    init_hid_dma();
    LOG_INIT("DMA HID report processing initialized successfully");
    return true;
#else
    // For RP2040, we'll still initialize the DMA but with a note
    LOG_INIT("RP2040 detected - DMA performance may be limited compared to RP2350");
    init_hid_dma();
    LOG_INIT("DMA HID report processing initialized successfully");
    return true;
#endif
}

void usb_stack_error_check(void)
{
    // Check for USB stack errors and reset if necessary
    extern usb_error_tracker_t usb_error_tracker;
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Only check periodically
    if (current_time - usb_error_tracker.last_error_check_time < ERROR_CHECK_INTERVAL_MS) {
        return;
    }
    
    usb_error_tracker.last_error_check_time = current_time;
    
    // Check for consecutive errors
    if (usb_error_tracker.consecutive_device_errors > MAX_CONSECUTIVE_ERRORS) {
        LOG_ERROR("Too many consecutive device errors (%lu), resetting device stack",
                 usb_error_tracker.consecutive_device_errors);
        usb_device_stack_reset();
        usb_error_tracker.consecutive_device_errors = 0;
    }
    
    if (usb_error_tracker.consecutive_host_errors > MAX_CONSECUTIVE_ERRORS) {
        LOG_ERROR("Too many consecutive host errors (%lu), resetting host stack",
                 usb_error_tracker.consecutive_host_errors);
        usb_host_stack_reset();
        usb_error_tracker.consecutive_host_errors = 0;
    }
}
