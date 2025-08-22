/*
 * Hurricane PIOKMBox Firmware
 */

#include "usb_hid.h"
#include "defines.h"
#include "led_control.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

#if PIO_USB_AVAILABLE
#include "pio_usb.h"
#endif
#include <stdlib.h>

// Error tracking structure with better organization
typedef struct
{
    uint32_t device_errors;
    uint32_t host_errors;
    uint32_t consecutive_device_errors;
    uint32_t consecutive_host_errors;
    uint32_t last_error_check_time;
    bool device_error_state;
    bool host_error_state;
} usb_error_tracker_t;

// Device connection state with better encapsulation
typedef struct
{
    bool mouse_connected;
    bool keyboard_connected;
    uint8_t mouse_dev_addr;
    uint8_t keyboard_dev_addr;
} device_connection_state_t;

// Device mode state
static bool caps_lock_state = false;

static device_connection_state_t connection_state = {0};

// Serial string buffer (16 hex chars + null terminator)
static char serial_string[SERIAL_STRING_BUFFER_SIZE] = {0};

// Error tracking - now properly typed
static usb_error_tracker_t usb_error_tracker = {0};

// USB stack initialization tracking
static bool usb_device_initialized = false;
static bool usb_host_initialized = false;

// Initialization helpers
static bool generate_serial_string(void);
static bool init_gpio_pins(void);

// Device management helpers
static void handle_device_disconnection(uint8_t dev_addr);
static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol);

// Report processing helpers
static bool process_keyboard_report_internal(const hid_keyboard_report_t *report);
static bool process_mouse_report_internal(const hid_mouse_report_t *report);

// Debug and logging helpers
static void print_device_info(uint8_t dev_addr, const tusb_desc_device_t *desc);

// --- Runtime HID descriptor mirroring storage & helpers ---
#define HID_DESC_BUF_SIZE 256

static const uint8_t desc_hid_keyboard[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD))};

static const uint8_t desc_hid_mouse_default[] = {
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE))};

static const uint8_t desc_hid_consumer[] = {
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(REPORT_ID_CONSUMER_CONTROL))};

// Static fallback concatenated descriptor (used by config descriptor sizeof)
const uint8_t desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE)),
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(REPORT_ID_CONSUMER_CONTROL))};

static uint8_t desc_hid_report_runtime[HID_DESC_BUF_SIZE];
static size_t desc_hid_runtime_len = 0;
static bool desc_hid_runtime_valid = false;

static uint8_t host_mouse_desc[HID_DESC_BUF_SIZE];
static size_t host_mouse_desc_len = 0;
static bool host_mouse_has_report_id = false;
static uint8_t host_mouse_report_id = 0;

static void build_runtime_hid_report_with_mouse(const uint8_t *mouse_desc, size_t mouse_len)
{
    // Concatenate keyboard + mouse_desc (or default) + consumer into runtime buffer
    size_t pos = 0;

    // Copy keyboard
    size_t klen = sizeof(desc_hid_keyboard);
    if (pos + klen >= HID_DESC_BUF_SIZE)
        return;
    memcpy(&desc_hid_report_runtime[pos], desc_hid_keyboard, klen);
    pos += klen;

    // Copy mouse (provided) or default
    if (mouse_desc != NULL && mouse_len > 0)
    {
        if (pos + mouse_len >= HID_DESC_BUF_SIZE)
            return;
        memcpy(&desc_hid_report_runtime[pos], mouse_desc, mouse_len);
        pos += mouse_len;
    }
    else
    {
        size_t dlen = sizeof(desc_hid_mouse_default);
        if (pos + dlen >= HID_DESC_BUF_SIZE)
            return;
        memcpy(&desc_hid_report_runtime[pos], desc_hid_mouse_default, dlen);
        pos += dlen;
    }

    // Copy consumer
    size_t clen = sizeof(desc_hid_consumer);
    if (pos + clen >= HID_DESC_BUF_SIZE)
        return;
    memcpy(&desc_hid_report_runtime[pos], desc_hid_consumer, clen);
    pos += clen;

    // Pad remainder with END_COLLECTION (0xC0) which is a safe terminator
    for (; pos < HID_DESC_BUF_SIZE; ++pos)
    {
        desc_hid_report_runtime[pos] = 0xC0;
    }

    desc_hid_runtime_len = HID_DESC_BUF_SIZE;
    desc_hid_runtime_valid = true;
}

bool usb_hid_init(void)
{
    // Generate unique serial string from chip ID
    if (!generate_serial_string())
    {
        printf("ERROR: Failed to generate serial string\n");
        return false;
    }

    // Initialize GPIO pins
    if (!init_gpio_pins())
    {
        printf("ERROR: Failed to initialize GPIO pins\n");
        return false;
    }

    // Initialize connection state
    memset(&connection_state, 0, sizeof(connection_state));

    // Build default runtime HID report descriptor (keyboard + default mouse + consumer)
    build_runtime_hid_report_with_mouse(NULL, 0);

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
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++)
    {
        int result = snprintf(&serial_string[i * SERIAL_HEX_CHARS_PER_BYTE],
                              SERIAL_SNPRINTF_BUFFER_SIZE,
                              "%02X",
                              board_id.id[i]);
        if (result < 0 || result >= SERIAL_SNPRINTF_BUFFER_SIZE)
        {
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

    // USB 5V power pin initialization but keep it OFF during early boot
    gpio_init(PIN_USB_5V);
    gpio_set_dir(PIN_USB_5V, GPIO_OUT);
    gpio_put(PIN_USB_5V, 0); // Keep USB power OFF initially for cold boot stability

    return true;
}

bool usb_host_enable_power(void)
{
    printf("Enabling USB host power...\n");
    gpio_put(PIN_USB_5V, 1); // Enable USB power
    sleep_ms(100);           // Allow power to stabilize
    printf("USB host power enabled on pin %d\n", PIN_USB_5V);
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

//--------------------------------------------------------------------+
// IMPROVED STATE MANAGEMENT
//--------------------------------------------------------------------+

bool get_caps_lock_state(void)
{
    return caps_lock_state;
}

bool is_mouse_connected(void)
{
    return connection_state.mouse_connected;
}

bool is_keyboard_connected(void)
{
    return connection_state.keyboard_connected;
}

static void handle_device_disconnection(uint8_t dev_addr)
{
    // Only reset connection flags for the specific device that was disconnected
    if (dev_addr == connection_state.mouse_dev_addr)
    {
        connection_state.mouse_connected = false;
        connection_state.mouse_dev_addr = 0;
    }

    if (dev_addr == connection_state.keyboard_dev_addr)
    {
        connection_state.keyboard_connected = false;
        connection_state.keyboard_dev_addr = 0;
    }
}

static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol)
{
    // Validate input parameters
    if (dev_addr == 0)
    {
        printf("ERROR: Invalid device address: %d\n", dev_addr);
        return;
    }

    printf("USB Host: Handling HID device connection - addr=%d, protocol=%d\n", dev_addr, itf_protocol);

    // Track connected device types and store device addresses
    switch (itf_protocol)
    {
    case HID_ITF_PROTOCOL_MOUSE:
        connection_state.mouse_connected = true;
        connection_state.mouse_dev_addr = dev_addr;
        printf("USB Host: Mouse connected at address %d - disabling button-based movement\n", dev_addr);
        neopixel_trigger_mouse_activity(); // Flash magenta for mouse connection
        break;

    case HID_ITF_PROTOCOL_KEYBOARD:
        connection_state.keyboard_connected = true;
        connection_state.keyboard_dev_addr = dev_addr;
        printf("USB Host: Keyboard connected at address %d\n", dev_addr);
        neopixel_trigger_keyboard_activity(); // Flash yellow for keyboard connection
        break;

    default:
        printf("USB Host: Unknown HID protocol: %d\n", itf_protocol);
        break;
    }

    printf("USB Host: Connection state - Mouse: %s, Keyboard: %s\n",
           connection_state.mouse_connected ? "YES" : "NO",
           connection_state.keyboard_connected ? "YES" : "NO");
}

static bool process_keyboard_report_internal(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // Fast path: skip ready check for maximum performance
    // TinyUSB will handle the queuing internally
    bool success = tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
    if (success)
    {
        // Skip error counter reset for performance
        return true;
    }
    else
    {
        return false;
    }
}

static bool process_mouse_report_internal(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // Fast button validation using bitwise AND
    uint8_t valid_buttons = report->buttons & 0x1F; // Keep first 5 bits (L/R/M/S1/S2 buttons)

    // Update physical button states in kmbox (for lock functionality)
    kmbox_update_physical_buttons(valid_buttons);

    // Add physical mouse movement to kmbox accumulators (respecting axis locks)
    if (report->x != 0 || report->y != 0)
    {
        kmbox_add_mouse_movement(report->x, report->y);
    }

    // Add physical wheel movement
    if (report->wheel != 0)
    {
        kmbox_add_wheel_movement(report->wheel);
    }

    // Now get the final movement and button values from kmbox which include
    // both any previously queued command movement and the newly added
    // physical movement.
    uint8_t buttons_to_send;
    int8_t x, y, wheel, pan;
    kmbox_get_mouse_report(&buttons_to_send, &x, &y, &wheel, &pan);

    int8_t final_x = x;
    int8_t final_y = y;
    int8_t final_wheel = wheel;

    // Fast path: skip ready check for maximum performance
    bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, buttons_to_send, final_x, final_y, final_wheel, pan);
    if (success)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void print_device_info(uint8_t dev_addr, const tusb_desc_device_t *desc)
{
    (void)dev_addr; // Suppress unused parameter warning
    if (desc == NULL)
    {
        printf("  Device info unavailable\n");
        return;
    }

    printf("  VID: 0x%04X, PID: 0x%04X, Class: 0x%02X\n",
           desc->idVendor, desc->idProduct, desc->bDeviceClass);
}

void process_kbd_report(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if (++activity_counter % KEYBOARD_ACTIVITY_THROTTLE == 0)
    {
        neopixel_trigger_keyboard_activity();
    }

    // Fast forward the report
    if (process_keyboard_report_internal(report))
    {
        // Report processed successfully
    }
}

void process_mouse_report(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if (++activity_counter % MOUSE_ACTIVITY_THROTTLE == 0)
    {
        neopixel_trigger_mouse_activity();
    }

    // Fast forward the report
    if (process_mouse_report_internal(report))
    {
        // Report processed successfully  
    }
}

bool find_key_in_report(const hid_keyboard_report_t *report, uint8_t keycode)
{
    if (report == NULL)
    {
        return false;
    }

    for (uint8_t i = 0; i < HID_KEYBOARD_KEYCODE_COUNT; i++)
    {
        if (report->keycode[i] == keycode)
        {
            return true;
        }
    }

    return false;
}

void hid_device_task(void)
{
    // Optimized polling: 16ms for better performance (60 FPS equivalent)
    static uint32_t start_ms = 0;
    uint32_t current_ms = to_ms_since_boot(get_absolute_time());

    if (current_ms - start_ms < HID_DEVICE_TASK_INTERVAL_MS)
    {
        return; // Not enough time elapsed
    }
    start_ms = current_ms;

    // Remote wakeup handling
    if (tud_suspended() && !gpio_get(PIN_BUTTON))
    {
        // Wake up host if we are in suspend mode and button is pressed
        tud_remote_wakeup();
        return;
    }

    // Only send reports when USB device is properly mounted and ready
    if (!tud_mounted() || !tud_ready())
    {
        return; // Don't send reports if device is not properly mounted
    }

    // Only send reports when devices are not connected (avoid conflicts)
    if (!connection_state.mouse_connected && !connection_state.keyboard_connected)
    {
        send_hid_report(REPORT_ID_MOUSE);
    }
    else
    {
        // Send empty consumer control report to maintain connection
        send_hid_report(REPORT_ID_CONSUMER_CONTROL);
    }
}

void send_hid_report(uint8_t report_id)
{
    // CRITICAL: Multiple checks to prevent endpoint allocation conflicts
    if (!tud_mounted() || !tud_ready())
    {
        return; // Prevent endpoint conflicts by not sending reports when device isn't ready
    }

    // CRITICAL: Additional check to ensure device stack is stable
    static uint32_t last_mount_check = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_mount_check > 1000)
    { // Check every second
        if (!tud_mounted())
        {
            return; // Device was unmounted, don't send reports
        }
        last_mount_check = current_time;
    }

    switch (report_id)
    {
    case REPORT_ID_KEYBOARD:
        if (!connection_state.keyboard_connected)
        {
            // CRITICAL: Check device readiness before each report
            if (tud_hid_ready())
            {
                // Use static array to avoid stack allocation overhead
                static const uint8_t empty_keycode[HID_KEYBOARD_KEYCODE_COUNT] = {0};
                tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, empty_keycode);
            }
        }
        break;

    case REPORT_ID_MOUSE:
        // Only send button-based mouse movement if no mouse is connected
        if (!connection_state.mouse_connected)
        {
            // CRITICAL: Check device readiness before each report
            if (tud_hid_ready())
            {
                static bool prev_button_state = true; // true = not pressed (active low)
                bool current_button_state = gpio_get(PIN_BUTTON);

                if (!current_button_state)
                { // button pressed (active low)
                    // Mouse move up (negative Y direction)
                    tud_hid_mouse_report(REPORT_ID_MOUSE, MOUSE_BUTTON_NONE,
                                         MOUSE_NO_MOVEMENT, MOUSE_BUTTON_MOVEMENT_DELTA,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT);
                }
                else if (prev_button_state != current_button_state)
                {
                    // Send stop movement when button is released
                    tud_hid_mouse_report(REPORT_ID_MOUSE, MOUSE_BUTTON_NONE,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT);
                }

                prev_button_state = current_button_state;
            }
        }
        break;

    case REPORT_ID_CONSUMER_CONTROL:
    {
        // CRITICAL: Check device readiness before each report
        if (tud_hid_ready())
        {
            static const uint16_t empty_key = 0;
            tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &empty_key, HID_CONSUMER_CONTROL_SIZE);
        }
        break;
    }

    default:
        break;
    }
}

void hid_host_task(void)
{
    // This function can be called from core0 if needed for additional host processing
    // The main host task loop runs on core1 in PIOKMbox.c
}

// Device callbacks with improved error handling
void tud_mount_cb(void)
{
    led_set_blink_interval(LED_BLINK_MOUNTED_MS);
    neopixel_update_status();
}

void tud_umount_cb(void)
{
    led_set_blink_interval(LED_BLINK_UNMOUNTED_MS);

    // Track device unmount as potential error condition
    usb_error_tracker.consecutive_device_errors++;

    neopixel_update_status();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    led_set_blink_interval(LED_BLINK_SUSPENDED_MS);
    neopixel_update_status();
}

void tud_resume_cb(void)
{
    led_set_blink_interval(LED_BLINK_RESUMED_MS);
    neopixel_update_status();
}

// Host callbacks with improved error handling
void tuh_mount_cb(uint8_t dev_addr)
{
    printf("USB Host: Device mounted at address %d\n", dev_addr);

    // Get device descriptor for more info
    tusb_desc_device_t desc_device;
    if (tuh_descriptor_get_device_sync(dev_addr, &desc_device, sizeof(desc_device)))
    {
        print_device_info(dev_addr, &desc_device);
    }
    else
    {
        printf("USB Host: Failed to get device descriptor for address %d\n", dev_addr);
    }

    // Trigger visual feedback
    neopixel_trigger_usb_connection_flash();
    neopixel_update_status();
}

void tuh_umount_cb(uint8_t dev_addr)
{

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    // Track host unmount with improved logic
    static uint32_t last_unmount_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (current_time - last_unmount_time < 5000)
    { // Less than 5 seconds since last unmount
        usb_error_tracker.consecutive_host_errors++;
        // Frequent host unmounts detected (reduced logging)
    }
    else
    {
        // Reset error count for normal disconnections
        usb_error_tracker.consecutive_host_errors = 0;
    }
    last_unmount_time = current_time;

    // Trigger visual feedback
    neopixel_trigger_usb_disconnection_flash();
    neopixel_update_status();
}

// HID host callbacks with improved validation
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, const uint8_t *desc_report, uint16_t desc_len)
{
    // Capture host report descriptor for later mirroring
    host_mouse_has_report_id = false;
    host_mouse_report_id = 0;
    if (desc_report != NULL && desc_len > 0)
    {
        size_t copy_len = desc_len;
        if (copy_len > sizeof(host_mouse_desc))
            copy_len = sizeof(host_mouse_desc);
        memcpy(host_mouse_desc, desc_report, copy_len);
        host_mouse_desc_len = copy_len;

        // Simple scan for Report ID (tag 0x85) in the descriptor to detect if report IDs are used
        for (size_t i = 0; i + 1 < host_mouse_desc_len; ++i)
        {
            if (host_mouse_desc[i] == 0x85)
            {
                host_mouse_has_report_id = true;
                host_mouse_report_id = host_mouse_desc[i + 1];
                break;
            }
        }

        // Rebuild runtime HID report descriptor referencing the host mouse descriptor
        build_runtime_hid_report_with_mouse(host_mouse_desc, host_mouse_desc_len);
        printf("USB Host: Captured host HID report descriptor len=%d, report_id_used=%s\n",
               (int)host_mouse_desc_len, host_mouse_has_report_id ? "YES" : "NO");
    }
    else
    {
        host_mouse_desc_len = 0;
    }

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    printf("USB Host: HID device mounted - addr=%d, instance=%d, protocol=%d\n",
           dev_addr, instance, itf_protocol);

    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE)
    {
        printf("USB Host: Mouse detected!\n");
    }
    else if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD)
    {
        printf("USB Host: Keyboard detected!\n");
    }
    else
    {
        printf("USB Host: Unknown HID protocol: %d\n", itf_protocol);
    }

    // Handle HID device connection
    handle_hid_device_connection(dev_addr, itf_protocol);

    // Start receiving reports
    if (!tuh_hid_receive_report(dev_addr, instance))
    {
        printf("USB Host: Failed to start receiving reports for device %d\n", dev_addr);
    }
    else
    {
        printf("USB Host: Started receiving reports for device %d\n", dev_addr);
    }
    neopixel_update_status();
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    (void)instance; // Suppress unused parameter warning

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    // Trigger visual feedback
    neopixel_trigger_usb_disconnection_flash();
    neopixel_update_status();
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, const uint8_t *report, uint16_t len)
{
    // Fast path: minimal validation for performance
    if (report == NULL || len == 0)
    {
        tuh_hid_receive_report(dev_addr, instance);
        return;
    }

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    // Direct processing without extra copying for better performance
    switch (itf_protocol)
    {
    case HID_ITF_PROTOCOL_KEYBOARD:
        // Ensure we have a full keyboard report before processing. Copy to a
        // local structure to avoid alignment and aliasing issues coming from
        // the USB buffer.
        if (len >= (int)sizeof(hid_keyboard_report_t))
        {
            hid_keyboard_report_t kbd_report;
            memcpy(&kbd_report, report, sizeof(kbd_report));
            process_kbd_report(&kbd_report);
        }
        break;

    case HID_ITF_PROTOCOL_MOUSE:
        // Process mouse reports through kmbox system instead of raw forwarding
        if (len > 0)
        {
            // Handle different mouse report formats
            hid_mouse_report_t mouse_report_local = {0};
            
            if (len == 8) {
                // This is a 16-bit coordinate mouse (8-byte report)
                // Format: [buttons] [pad] [pad] [pad] [x_low] [x_high] [y_low] [y_high]
                mouse_report_local.buttons = report[0];
                
                // Extract 16-bit X and Y coordinates and convert to 8-bit with scaling
                int16_t x16 = (int16_t)(report[4] | (report[5] << 8));
                int16_t y16 = (int16_t)(report[6] | (report[7] << 8));
                
                // Scale down the 16-bit coordinates for smoother movement
                // Right shift by 2 bits (divide by 4) to reduce sensitivity and prevent overflow
                x16 >>= 2;
                y16 >>= 2;
                
                // Clamp to 8-bit range 
                if (x16 > 127) mouse_report_local.x = 127;
                else if (x16 < -128) mouse_report_local.x = -128;
                else mouse_report_local.x = (int8_t)x16;
                
                if (y16 > 127) mouse_report_local.y = 127;
                else if (y16 < -128) mouse_report_local.y = -128;
                else mouse_report_local.y = (int8_t)y16;
                
                // Try different positions for scroll wheel - check all possible bytes
                // Most likely candidates are bytes 1, 2, or 3 (after buttons)
                int8_t wheel_candidate = 0;
                if (report[1] != 0) wheel_candidate = (int8_t)report[1];
                else if (report[2] != 0) wheel_candidate = (int8_t)report[2];
                else if (report[3] != 0) wheel_candidate = (int8_t)report[3];
                
                mouse_report_local.wheel = wheel_candidate;
                mouse_report_local.pan = 0;
                
            } else {
                // Standard 5-byte mouse report format
                size_t copy_sz = len;
                if (copy_sz > sizeof(mouse_report_local))
                    copy_sz = sizeof(mouse_report_local);
                memcpy(&mouse_report_local, report, copy_sz);
            }

            // Process through kmbox system (this handles movement accumulation,
            // axis locks, and generates the final report to send)
            process_mouse_report(&mouse_report_local);
        }
        break;

    default:
        // Unknown HID protocol - ignore
        break;
    }

    // Continue to request reports
    tuh_hid_receive_report(dev_addr, instance);
}

// HID device callbacks with improved validation
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize)
{
    (void)instance;

    if (report_type == HID_REPORT_TYPE_OUTPUT && report_id == REPORT_ID_KEYBOARD)
    {
        // Validate buffer
        if (buffer == NULL || bufsize < MIN_BUFFER_SIZE)
        {
            return;
        }

        uint8_t const kbd_leds = buffer[0];
        bool new_caps_state = (kbd_leds & KEYBOARD_LED_CAPSLOCK) != 0;

        if (new_caps_state != caps_lock_state)
        {
            caps_lock_state = new_caps_state;
            printf("CAPSLOCK %s\n", caps_lock_state ? "ON" : "OFF");
            neopixel_trigger_caps_lock_flash();
        }
    }
}

void tud_hid_report_complete_cb(uint8_t instance, const uint8_t *report, uint16_t len)
{
    (void)instance;
    (void)len;
    (void)report;
}

bool usb_device_stack_reset(void)
{
    printf("USB Device Stack Reset: Starting...\n");
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if device stack was previously initialized
    if (!usb_device_initialized)
    {
        printf("USB Device Stack Reset: Device not initialized, skipping reset\n");
        return true;
    }

    // Disable automatic reset to prevent endpoint conflicts with host stack
    printf("USB Device Stack Reset: Automatic reset disabled to prevent endpoint conflicts\n");
    printf("USB Device Stack Reset: Manual intervention required\n");

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.device_errors = 0;
    usb_error_tracker.consecutive_device_errors = 0;
    usb_error_tracker.device_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    printf("USB Device Stack Reset: SKIPPED (%lu ms)\n", elapsed_time);

    return true; // Return success to prevent repeated attempts
}

bool usb_host_stack_reset(void)
{
#if PIO_USB_AVAILABLE
    printf("USB Host Stack Reset: Starting...\n");
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if host stack was previously initialized
    if (!usb_host_initialized)
    {
        printf("USB Host Stack Reset: Host not initialized, skipping reset\n");
        return true;
    }

    // Disable automatic reset to prevent endpoint conflicts with device stack
    printf("USB Host Stack Reset: Automatic reset disabled to prevent endpoint conflicts\n");
    printf("USB Host Stack Reset: Manual intervention required\n");

    // Reset connection tracking without reinitializing stacks
    memset(&connection_state, 0, sizeof(connection_state));

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.host_errors = 0;
    usb_error_tracker.consecutive_host_errors = 0;
    usb_error_tracker.host_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    printf("USB Host Stack Reset: SKIPPED (%lu ms)\n", elapsed_time);

    return true; // Return success to prevent repeated attempts
#else
    printf("USB Host Stack Reset: PIO USB not available\n");
    return false;
#endif
}

bool usb_stacks_reset(void)
{
    printf("USB Stacks Reset: Starting full USB stack reset...\n");
    neopixel_trigger_usb_reset_pending();

    const bool device_success = usb_device_stack_reset();
    const bool host_success = usb_host_stack_reset();
    const bool overall_success = device_success && host_success;

    if (overall_success)
    {
        printf("USB Stacks Reset: Both stacks reset successfully\n");
        neopixel_trigger_usb_reset_success();
    }
    else
    {
        printf("USB Stacks Reset: Reset failed - Device: %s, Host: %s\n",
               device_success ? "OK" : "FAILED",
               host_success ? "OK" : "FAILED");
        neopixel_trigger_usb_reset_failed();
    }

    return overall_success;
}

void usb_stack_error_check(void)
{
    const uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Only check errors at specified intervals
    if (current_time - usb_error_tracker.last_error_check_time < USB_ERROR_CHECK_INTERVAL_MS)
    {
        return;
    }
    usb_error_tracker.last_error_check_time = current_time;

    // Check device stack health - be more conservative about what constitutes an error
    // Only consider it an error if the device stack is completely non-functional
    const bool device_healthy = tud_ready(); // Remove mount requirement as device can be functional without being mounted
    if (!device_healthy)
    {
        usb_error_tracker.consecutive_device_errors++;
        if (!usb_error_tracker.device_error_state)
        {
            // Device stack unhealthy (reduced logging)
        }
    }
    else
    {
        if (usb_error_tracker.consecutive_device_errors > 0)
        {
            // Device stack recovered (reduced logging)
        }
        usb_error_tracker.consecutive_device_errors = 0;
        usb_error_tracker.device_error_state = false;
    }

    // Trigger reset if error threshold is exceeded
    if (usb_error_tracker.consecutive_device_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.device_error_state)
        {
            printf("USB Error Check: Device error threshold exceeded, but automatic reset disabled to prevent endpoint conflicts\n");
            usb_error_tracker.device_error_state = true;
        }
    }

#if PIO_USB_AVAILABLE
    if (usb_error_tracker.consecutive_host_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.host_error_state)
        {
            printf("USB Error Check: Host error threshold exceeded, but automatic reset disabled to prevent endpoint conflicts\n");
            usb_error_tracker.host_error_state = true;
        }
    }
#endif
}

// Device Descriptors
tusb_desc_device_t const desc_device =
    {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = USB_BCD_VERSION,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = USB_VENDOR_ID,
        .idProduct = USB_PRODUCT_ID,
        .bcdDevice = USB_DEVICE_VERSION,

        .iManufacturer = STRING_DESC_MANUFACTURER_IDX,
        .iProduct = STRING_DESC_PRODUCT_IDX,
        .iSerialNumber = STRING_DESC_SERIAL_IDX,

        .bNumConfigurations = USB_NUM_CONFIGURATIONS};

// Invoked when received GET DEVICE DESCRIPTOR
uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

// HID Report Descriptor
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    if (desc_hid_runtime_valid)
    {
        return desc_hid_report_runtime;
    }
    // Fallback to static concatenation if runtime descriptor not ready
    return desc_hid_report_runtime; // still points to buffer (may contain defaults)
}

// Configuration Descriptor
enum
{
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

uint8_t const desc_configuration[] =
    {
        // Config number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USB_CONFIG_POWER_MA),

        // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
        TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, HID_POLLING_INTERVAL_MS)};

// Invoked when received GET CONFIGURATION DESCRIPTOR
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index; // for multiple configurations
    return desc_configuration;
}

// String Descriptors
char const *string_desc_arr[] =
    {
        (const char[]){USB_LANGUAGE_ENGLISH_US_BYTE1, USB_LANGUAGE_ENGLISH_US_BYTE2}, // 0: is supported language is English (0x0409)
        MANUFACTURER_STRING,                                                          // 1: Manufacturer
        PRODUCT_STRING,                                                               // 2: Product
        serial_string                                                                 // 3: Serial number from unique chip ID
};

static uint16_t _desc_str[MAX_STRING_DESCRIPTOR_CHARS + 1];

// Improved string descriptor conversion with better error handling
static uint8_t convert_string_to_utf16(const char *str, uint16_t *desc_str)
{
    if (str == NULL || desc_str == NULL)
    {
        return 0;
    }

    // Cap at max char
    uint8_t chr_count = strlen(str);
    if (chr_count > MAX_STRING_DESCRIPTOR_CHARS)
    {
        chr_count = MAX_STRING_DESCRIPTOR_CHARS;
    }

    // Convert ASCII string into UTF-16
    for (uint8_t i = 0; i < chr_count; i++)
    {
        desc_str[STRING_DESC_FIRST_CHAR_OFFSET + i] = str[i];
    }

    return chr_count;
}

// Invoked when received GET STRING DESCRIPTOR request
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    uint8_t chr_count;

    if (index == BUFFER_FIRST_ELEMENT_INDEX)
    {
        memcpy(&_desc_str[STRING_DESC_FIRST_CHAR_OFFSET],
               string_desc_arr[BUFFER_FIRST_ELEMENT_INDEX],
               STRING_DESC_HEADER_SIZE);
        chr_count = STRING_DESC_CHAR_COUNT_INIT;
    }
    else
    {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[BUFFER_FIRST_ELEMENT_INDEX])))
        {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        if (str == NULL)
        {
            return NULL;
        }

        // Convert ASCII string to UTF-16 and get character count
        chr_count = convert_string_to_utf16(str, _desc_str);
    }

    // first byte is length (including header), second byte is string type
    _desc_str[BUFFER_FIRST_ELEMENT_INDEX] = (TUSB_DESC_STRING << STRING_DESC_TYPE_SHIFT) |
                                            (STRING_DESC_LENGTH_MULTIPLIER * chr_count + STRING_DESC_HEADER_SIZE);

    return _desc_str;
}