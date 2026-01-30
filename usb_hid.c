/*
 * Hurricane PIOKMBox Firmware
 */

#include "usb_hid.h"
#include "defines.h"
#include "led_control.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "kmbox_serial_handler.h" // Include the header for serial handling
#include "state_management.h"   // Include the header for state management
#include "watchdog.h"           // Include the header for watchdog management
#include "smooth_injection.h"   // Include the header for smooth mouse injection
#include <string.h>             // For strcpy, strlen, memset

uint16_t attached_vid = 0;
uint16_t attached_pid = 0;
bool attached_has_serial = false;

// Dynamic string descriptor storage
static char attached_manufacturer[64] = "";
static char attached_product[64] = "";
static char attached_serial[32] = "";
static bool string_descriptors_fetched = false;

#define LANGUAGE_ID 0x0409  // English (US)

// UTF-16LE string descriptor -> UTF-8 (ASCII-only) conversion.
// Uses the descriptor's bLength to avoid reading past valid data.
static void utf16_to_utf8(uint16_t *utf16_buf, size_t utf16_buf_bytes, char *utf8_buf, size_t utf8_len)
{
    if (!utf16_buf || !utf8_buf || utf8_len == 0)
        return;

    // String descriptor format: [bLength (1B)][bDescriptorType (1B)][UTF-16LE code units...]
    // Determine actual descriptor length from first byte
    const uint8_t *raw = (const uint8_t *)utf16_buf;
    if (!raw)
        return;

    // Cap length by provided buffer size to be safe
    uint8_t bLength = raw[0];
    if (bLength > utf16_buf_bytes)
    {
        bLength = (uint8_t)utf16_buf_bytes;
    }

    // Compute number of 16-bit code units
    size_t code_units = 0;
    if (bLength >= 2)
    {
        code_units = (size_t)(bLength - 2) / 2;
    }

    size_t utf8_pos = 0;
    
    // Unroll loop by 4 for better performance (most strings are short)
    size_t i = 0;
    while (i + 3 < code_units && utf8_pos + 3 < utf8_len - 1) {
        uint16_t u0 = utf16_buf[1 + i];
        uint16_t u1 = utf16_buf[1 + i + 1];
        uint16_t u2 = utf16_buf[1 + i + 2];
        uint16_t u3 = utf16_buf[1 + i + 3];
        
        // Early exit on NUL
        if (u0 == 0) break;
        utf8_buf[utf8_pos++] = (u0 <= 0x7F) ? (char)u0 : '?';
        
        if (u1 == 0) break;
        utf8_buf[utf8_pos++] = (u1 <= 0x7F) ? (char)u1 : '?';
        
        if (u2 == 0) break;
        utf8_buf[utf8_pos++] = (u2 <= 0x7F) ? (char)u2 : '?';
        
        if (u3 == 0) break;
        utf8_buf[utf8_pos++] = (u3 <= 0x7F) ? (char)u3 : '?';
        
        i += 4;
    }
    
    // Handle remaining code units
    for (; i < code_units && utf8_pos < utf8_len - 1; i++)
    {
        uint16_t u = utf16_buf[1 + i];
        if (u == 0) break;
        utf8_buf[utf8_pos++] = (u <= 0x7F) ? (char)u : '?';
    }

    utf8_buf[utf8_pos] = '\0';
}

// Function to set the VID and PID of the attached device
void set_attached_device_vid_pid(uint16_t vid, uint16_t pid) {
    // Only update and re-enumerate if VID/PID has actually changed
    if (attached_vid != vid || attached_pid != pid) {
        attached_vid = vid;
        attached_pid = pid;
        attached_has_serial = false; // Default to no serial number unless device has one
        printf("Updated attached device VID:PID to %04x:%04x\n", vid, pid);
        
        // Force USB re-enumeration to update descriptor
        force_usb_reenumeration();
    } else {
        printf("VID:PID unchanged (%04x:%04x), skipping re-enumeration\n", vid, pid);
    }
}

void force_usb_reenumeration() {
    printf("Forcing USB re-enumeration with new descriptor...\n");
    
    // Disconnect from USB host
    tud_disconnect();
    
    // Wait for host to recognize disconnection (250ms minimum for Windows/macOS)
    sleep_ms(500);
    
    // Reconnect with new descriptor
    tud_connect();
    
    // Wait for reconnection
    sleep_ms(250);
    
    printf("USB re-enumeration complete\n");
}

// Function to fetch string descriptors from attached device
static void fetch_device_string_descriptors(uint8_t dev_addr) {
    // Reset string descriptors
    memset(attached_manufacturer, 0, sizeof(attached_manufacturer));
    memset(attached_product, 0, sizeof(attached_product));
    memset(attached_serial, 0, sizeof(attached_serial));
    string_descriptors_fetched = false;
    
    // Temporary buffers for UTF-16 strings
    uint16_t temp_manufacturer[32];
    uint16_t temp_product[48];
    uint16_t temp_serial[16];

    // Ensure buffers are zeroed to avoid stray data being interpreted
    memset(temp_manufacturer, 0, sizeof(temp_manufacturer));
    memset(temp_product, 0, sizeof(temp_product));
    memset(temp_serial, 0, sizeof(temp_serial));
    
    // Get manufacturer string
    if (tuh_descriptor_get_manufacturer_string_sync(dev_addr, LANGUAGE_ID, temp_manufacturer, sizeof(temp_manufacturer)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_manufacturer, sizeof(temp_manufacturer), attached_manufacturer, sizeof(attached_manufacturer));
        printf("Fetched manufacturer: %s\n", attached_manufacturer);
        kmbox_send_status(attached_manufacturer);
    } else {
        strcpy(attached_manufacturer, MANUFACTURER_STRING);  // Fallback
        printf("Failed to fetch manufacturer, using fallback: %s\n", attached_manufacturer);
    }
    
    // Get product string
    if (tuh_descriptor_get_product_string_sync(dev_addr, LANGUAGE_ID, temp_product, sizeof(temp_product)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_product, sizeof(temp_product), attached_product, sizeof(attached_product));
        printf("Fetched product: %s\n", attached_product);
        kmbox_send_status(attached_product);
    } else {
        strcpy(attached_product, PRODUCT_STRING);  // Fallback
        printf("Failed to fetch product, using fallback: %s\n", attached_product);
    }
    
    // Get serial string (optional)
    if (tuh_descriptor_get_serial_string_sync(dev_addr, LANGUAGE_ID, temp_serial, sizeof(temp_serial)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_serial, sizeof(temp_serial), attached_serial, sizeof(attached_serial));
        printf("Fetched serial: %s\n", attached_serial);
        char serial_msg[64];
        snprintf(serial_msg, sizeof(serial_msg), "Serial: %s", attached_serial);
        kmbox_send_status(serial_msg);
        attached_has_serial = (strlen(attached_serial) > 0);
    } else {
        attached_has_serial = false;
        printf("No serial number available\n");
    }
    
    string_descriptors_fetched = true;
}

// Function to reset string descriptors when device is disconnected
static void reset_device_string_descriptors(void) {
    memset(attached_manufacturer, 0, sizeof(attached_manufacturer));
    memset(attached_product, 0, sizeof(attached_product));
    memset(attached_serial, 0, sizeof(attached_serial));
    string_descriptors_fetched = false;
    attached_has_serial = false;
    printf("String descriptors reset\n");
}

// Function to get the VID of the attached device
uint16_t get_attached_vid(void) {
    return attached_vid;
}

// Function to get the PID of the attached device
uint16_t get_attached_pid(void) {
    return attached_pid;
}

// Function to get dynamic serial string
const char* get_dynamic_serial_string() {
    static char dynamic_serial[64];
    if (attached_vid && attached_pid) {
        snprintf(dynamic_serial, sizeof(dynamic_serial), "PIOKMbox_%04X_%04X", attached_vid, attached_pid);
        return dynamic_serial;
    }
    return "PIOKMbox_v1.0";
}

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

// Error tracking - now properly typed
static usb_error_tracker_t usb_error_tracker = {0};

// USB stack initialization tracking
static bool usb_device_initialized = false;
static bool usb_host_initialized = false;

// Device management helpers
static void handle_device_disconnection(uint8_t dev_addr);
static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol);

// Report processing helpers
static bool process_keyboard_report_internal(const hid_keyboard_report_t *report);
static bool process_mouse_report_internal(const hid_mouse_report_t *report);

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
    gpio_init(PIN_BUTTON);

    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_BUTTON);

    // Initialize connection state
    memset(&connection_state, 0, sizeof(connection_state));

    // Build default runtime HID report descriptor (keyboard + default mouse + consumer)
    build_runtime_hid_report_with_mouse(NULL, 0);

    (void)0; // suppressed init log
    return true;
}



bool usb_host_enable_power(void)
{
    #ifdef PIN_USB_5V
    gpio_put(PIN_USB_5V, 1); // Enable USB power
    #endif
    sleep_ms(100);           // Allow power to stabilize
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
        // Invalid device address - avoid heavy logging in hot path
        return;
    }

    // Track connected device types and store device addresses.
    // Use LED activity to indicate connection instead of console logging.
    switch (itf_protocol)
    {
    case HID_ITF_PROTOCOL_MOUSE:
        connection_state.mouse_connected = true;
        connection_state.mouse_dev_addr = dev_addr;
        neopixel_trigger_activity_flash_color(COLOR_MOUSE_ACTIVITY); // Flash magenta for mouse connection
        kmbox_send_status("Mouse connected");
        break;

    case HID_ITF_PROTOCOL_KEYBOARD:
        connection_state.keyboard_connected = true;
        connection_state.keyboard_dev_addr = dev_addr;
        neopixel_trigger_activity_flash_color(COLOR_KEYBOARD_ACTIVITY); // Flash yellow for keyboard connection
        kmbox_send_status("Keyboard connected");
        break;

    default:
        // Unknown HID protocol: silently ignore logging in the hot path
        kmbox_send_status("Unknown HID device connected");
        break;
    }

    // Update LED status instead of verbose console output
    neopixel_update_status();
}

static bool __not_in_flash_func(process_keyboard_report_internal)(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // Fast path: skip ready check for maximum performance
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

static bool __not_in_flash_func(process_mouse_report_internal)(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // Check if USB device is ready to send reports
    if (!tud_mounted() || !tud_ready())
    {
        return false;
    }

    // Fast button validation using bitwise AND
    uint8_t valid_buttons = report->buttons & 0x1F; // Keep first 5 bits (L/R/M/S1/S2 buttons)

    // Update physical button states in kmbox (for lock functionality)
    kmbox_update_physical_buttons(valid_buttons);

    // Record physical movement for velocity tracking (smooth injection)
    if (report->x != 0 || report->y != 0)
    {
        smooth_record_physical_movement(report->x, report->y);
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

    // Process smooth injection queue and blend with physical movement
    // This provides sub-pixel precision and velocity-aware blending
    int8_t smooth_x = 0, smooth_y = 0;
    if (smooth_has_pending())
    {
        smooth_process_frame(&smooth_x, &smooth_y);
        // Add smooth injection to the physical/kmbox movement
        // Clamp the total to int8_t range
        int16_t total_x = (int16_t)x + (int16_t)smooth_x;
        int16_t total_y = (int16_t)y + (int16_t)smooth_y;
        x = (total_x > 127) ? 127 : ((total_x < -128) ? -128 : (int8_t)total_x);
        y = (total_y > 127) ? 127 : ((total_y < -128) ? -128 : (int8_t)total_y);
    }

    int8_t final_x = x;
    int8_t final_y = y;
    int8_t final_wheel = wheel;

    // Check if HID interface is ready
    if (!tud_hid_ready())
    {
        return false;
    }

    // Fast path: skip ready check for maximum performance
    bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, buttons_to_send, final_x, final_y, final_wheel, pan);
    return success;
}

void __not_in_flash_func(process_kbd_report)(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if (++activity_counter % KEYBOARD_ACTIVITY_THROTTLE == 0)
    {
        neopixel_trigger_activity_flash_color(COLOR_KEYBOARD_ACTIVITY);
    }

    // Fast forward the report
    if (process_keyboard_report_internal(report))
    {
        // Report processed successfully
    }
}

void __not_in_flash_func(process_mouse_report)(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if (++activity_counter % MOUSE_ACTIVITY_THROTTLE == 0)
    {
        neopixel_trigger_activity_flash_color(COLOR_MOUSE_ACTIVITY);
    }

    // Fast forward the report
    if (process_mouse_report_internal(report))
    {
        // Report processed successfully  
    }

    // If the report contains movement, advance the rainbow hue based on movement
    if (report->x != 0 || report->y != 0)
    {
        neopixel_rainbow_on_movement(report->x, report->y);
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
        return;
    }

    // Only send reports when devices are not connected
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
    // Multiple checks to prevent endpoint allocation conflicts
    if (!tud_mounted() || !tud_ready())
    {
        return; // Prevent endpoint conflicts by not sending reports when device isn't ready
    }

    // Additional check to ensure device stack is stable
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
            // Check device readiness before each report
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
            // Check device readiness before each report
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
    // The main host task runs on core1 in PIOKMbox.c
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
    neopixel_trigger_activity_flash_color(COLOR_USB_CONNECTION);
    neopixel_update_status();
}

void tuh_umount_cb(uint8_t dev_addr)
{

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    static uint32_t last_unmount_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // If unmounts happen in quick succession, count as consecutive errors
    if (current_time - last_unmount_time < 5000)
    {
        usb_error_tracker.consecutive_host_errors++;
    }
    else
    {
        // Reset error count for normal disconnections
        usb_error_tracker.consecutive_host_errors = 0;
    }
    last_unmount_time = current_time;

    // Trigger visual feedback
    neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
    neopixel_update_status();
}

// HID host callbacks with improved validation
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, const uint8_t *desc_report, uint16_t desc_len)
{
    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    printf("HID device mounted, VID: %04x, PID: %04x\n", vid, pid);
    
    // Send status to bridge
    char status_msg[128];
    snprintf(status_msg, sizeof(status_msg), "HID Device: VID=%04X PID=%04X", vid, pid);
    kmbox_send_status(status_msg);
    
    // Fetch string descriptors from the attached device
    fetch_device_string_descriptors(dev_addr);
    
    set_attached_device_vid_pid(vid, pid);

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
    }
    else
    {
        host_mouse_desc_len = 0;
    }

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    // Handle HID device connection
    handle_hid_device_connection(dev_addr, itf_protocol);

    // Start receiving reports
    if (!tuh_hid_receive_report(dev_addr, instance))
    {
    // Receiving reports failed - indicate via LED error flash
    neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
    }
    else
    {
    // Successfully started receiving reports; update status LED
    neopixel_update_status();
    }
    neopixel_update_status();
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    (void)instance; // Suppress unused parameter warning

    // Reset string descriptors when device is disconnected
    reset_device_string_descriptors();

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    // Trigger visual feedback
    neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
    neopixel_update_status();
}

void __not_in_flash_func(tuh_hid_report_received_cb)(uint8_t dev_addr, uint8_t instance, const uint8_t *report, uint16_t len)
{
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
        // Process directly from USB buffer - it's already aligned
        if (len >= (int)sizeof(hid_keyboard_report_t))
        {
            process_kbd_report((const hid_keyboard_report_t*)report);
        }
        break;

    case HID_ITF_PROTOCOL_MOUSE:
        // Process mouse reports through kmbox system - optimize for zero-copy
        if (len >= 3)
        {
            // Stack-allocated report for fast processing (no heap)
            hid_mouse_report_t mouse_report_local;
            
            if (len == 8) {
                // 16-bit coordinate mouse - direct extraction
                mouse_report_local.buttons = report[0];
                
                // Extract and scale 16-bit coordinates inline
                int16_t x16 = (int16_t)(report[4] | (report[5] << 8)) >> 2;
                int16_t y16 = (int16_t)(report[6] | (report[7] << 8)) >> 2;
                
                // Clamp inline
                mouse_report_local.x = (x16 > 127) ? 127 : (x16 < -128) ? -128 : (int8_t)x16;
                mouse_report_local.y = (y16 > 127) ? 127 : (y16 < -128) ? -128 : (int8_t)y16;
                
                // Wheel detection
                mouse_report_local.wheel = (report[1] != 0) ? (int8_t)report[1] : 
                                           (report[2] != 0) ? (int8_t)report[2] : (int8_t)report[3];
                mouse_report_local.pan = 0;
                
            } else {
                // Standard format - can cast directly if length allows
                if (len >= sizeof(hid_mouse_report_t)) {
                    // Direct cast for standard reports
                    process_mouse_report((const hid_mouse_report_t*)report);
                    break;
                }
                
                // Partial report - build manually
                mouse_report_local.buttons = report[0];
                mouse_report_local.x = (int8_t)report[1];
                mouse_report_local.y = (int8_t)report[2];
                mouse_report_local.wheel = (len >= 4) ? (int8_t)report[3] : 0;
                mouse_report_local.pan = (len >= 5) ? (int8_t)report[4] : 0;
            }

            // Process through kmbox system
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
            // Indicate caps lock change with LED flash instead of console logging
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
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if device stack was previously initialized
    if (!usb_device_initialized)
    {
        return true;
    }

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.device_errors = 0;
    usb_error_tracker.consecutive_device_errors = 0;
    usb_error_tracker.device_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    (void)elapsed_time; // suppressed timing log

    return true; // Return success to prevent repeated attempts
}

bool usb_host_stack_reset(void)
{
#if PIO_USB_AVAILABLE
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if host stack was previously initialized
    if (!usb_host_initialized)
    {
        return true;
    }

    // Reset connection tracking without reinitializing stacks
    memset(&connection_state, 0, sizeof(connection_state));

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.host_errors = 0;
    usb_error_tracker.consecutive_host_errors = 0;
    usb_error_tracker.host_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    (void)elapsed_time; // suppressed timing log

    return true; // Return success to prevent repeated attempts
#else
    (void)0; // suppressed log
    return false;
#endif
}

bool usb_stacks_reset(void)
{
    neopixel_trigger_usb_reset_pending();

    const bool device_success = usb_device_stack_reset();
    const bool host_success = usb_host_stack_reset();
    const bool overall_success = device_success && host_success;

    if (overall_success)
    {
        neopixel_trigger_usb_reset_success();
    }
    else
    {
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

    const bool device_healthy = tud_ready(); // Remove mount requirement as device can be functional without being mounted
    if (!device_healthy)
    {
        usb_error_tracker.consecutive_device_errors++;
    }
    else
    {
        usb_error_tracker.consecutive_device_errors = 0;
        usb_error_tracker.device_error_state = false;
    }

    // Trigger reset if error threshold is exceeded
    if (usb_error_tracker.consecutive_device_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.device_error_state)
        {
            usb_error_tracker.device_error_state = true;
        }
    }

#if PIO_USB_AVAILABLE
    if (usb_error_tracker.consecutive_host_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.host_error_state)
        {
            usb_error_tracker.host_error_state = true;
        }
    }
#endif
}

// Device Descriptors
// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
    static tusb_desc_device_t desc_device;

    desc_device = (tusb_desc_device_t) {
        .bLength            = sizeof(tusb_desc_device_t),
        .bDescriptorType    = TUSB_DESC_DEVICE,
        .bcdUSB             = 0x0200,
        .bDeviceClass       = 0x00,
        .bDeviceSubClass    = 0x00,
        .bDeviceProtocol    = 0x00,
        .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor           = (get_attached_vid() != 0) ? get_attached_vid() : USB_VENDOR_ID,
        .idProduct          = (get_attached_pid() != 0) ? get_attached_pid() : USB_PRODUCT_ID,
        .bcdDevice          = 0x0100,

        .iManufacturer      = 0x01,
        .iProduct           = 0x02,
        .iSerialNumber      = attached_has_serial ? 0x03 : 0x00,

        .bNumConfigurations = 0x01
    };

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
        "PIOKMbox_fallback"                                                           // 3: Fallback serial (unused - dynamic serial used instead)
};

static uint16_t _desc_str[MAX_STRING_DESCRIPTOR_CHARS + 1];

// Convert ASCII string into UTF-16
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

        // Use dynamic string descriptors if available
        if (string_descriptors_fetched) {
            switch (index) {
                case STRING_DESC_MANUFACTURER_IDX:
                    str = attached_manufacturer;
                    break;
                case STRING_DESC_PRODUCT_IDX:
                    str = attached_product;
                    break;
                case STRING_DESC_SERIAL_IDX:
                    if (attached_has_serial && strlen(attached_serial) > 0) {
                        str = attached_serial;
                    } else {
                        str = get_dynamic_serial_string();
                    }
                    break;
                default:
                    // Use default for other indices
                    break;
            }
        } else {
            // Fallback to default strings if not fetched yet
            if (index == STRING_DESC_SERIAL_IDX) {
                str = get_dynamic_serial_string();
            }
        }

        // Convert ASCII string to UTF-16 and get character count
        chr_count = convert_string_to_utf16(str, _desc_str);
    }

    // first byte is length (including header), second byte is string type
    _desc_str[BUFFER_FIRST_ELEMENT_INDEX] = (TUSB_DESC_STRING << STRING_DESC_TYPE_SHIFT) |
                                            (STRING_DESC_LENGTH_MULTIPLIER * chr_count + STRING_DESC_HEADER_SIZE);

    return _desc_str;
}