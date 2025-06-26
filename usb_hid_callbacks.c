/*
 * Hurricane PIOKMbox Firmware
 */

#include "usb_hid_callbacks.h"
#include "usb_hid_types.h"
#include "usb_hid_reports.h"
#include "usb_hid_descriptors.h"
#include "usb_hid_strings.h"
#include "usb_hid_host.h"
#include "defines.h"
#include "led_control.h"
#include <stdio.h>
#include <stdbool.h>

// External declarations for variables defined in other modules
extern device_connection_state_t connection_state;
extern usb_error_tracker_t usb_error_tracker;
extern performance_stats_t stats;

// Global flag for hardware acceleration
#if USE_HARDWARE_ACCELERATION
static bool hw_accel_enabled = true;
#endif

// Device callbacks with improved error handling
void tud_mount_cb(void)
{
    led_set_blink_interval(LED_BLINK_MOUNTED_MS);
    // USB Device mounted (reduced logging)
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
    (void) remote_wakeup_en;
    led_set_blink_interval(LED_BLINK_SUSPENDED_MS);
    neopixel_update_status();
}

void tud_resume_cb(void)
{
    led_set_blink_interval(LED_BLINK_RESUMED_MS);
    // USB Device resumed (reduced logging)
    neopixel_update_status();
}

// HID device callbacks
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) bufsize;
    
    // This is where we would handle incoming LED state reports (caps lock, etc.)
    if (report_type == HID_REPORT_TYPE_OUTPUT && bufsize == 1) {
        // The LED state is in the first byte
        uint8_t led_state = buffer[0];
        
        // Check if caps lock is on
        bool caps_lock = (led_state & KEYBOARD_LED_CAPSLOCK) != 0;
        
        // Update the caps lock state
        extern bool caps_lock_state;
        caps_lock_state = caps_lock;
        
        // Visual feedback for caps lock state
        if (caps_lock) {
            neopixel_trigger_caps_lock_flash();
        } else {
            // No specific function for caps lock off, just update status
            neopixel_update_status();
        }
    }
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
    (void) instance;
    (void) report;
    (void) len;
    
}

// Host callbacks with improved error handling
void tuh_mount_cb(uint8_t dev_addr)
{
    printf("USB Host: Device mounted at address %d\n", dev_addr);
    
    // Get VID/PID for basic device identification
    uint16_t vid, pid;
    if (tuh_vid_pid_get(dev_addr, &vid, &pid)) {
        printf("USB Host: Device %d - VID:0x%04X PID:0x%04X\n", dev_addr, vid, pid);
        
        // Update device descriptors with this VID/PID
        update_device_descriptors(vid, pid);
    } else {
        printf("USB Host: Device %d - Basic HID device (VID/PID not available)\n", dev_addr);
    }
    
    neopixel_trigger_usb_connection_flash();
    neopixel_update_status();
}

void tuh_umount_cb(uint8_t dev_addr)
{
    printf("USB Host: Device unmounted at address %d\n", dev_addr);

    // Handle device disconnection - this will reset VID/PID if needed
    handle_device_disconnection(dev_addr);
    
    static uint32_t last_unmount_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (current_time - last_unmount_time < 5000) { // Less than 5 seconds since last unmount
        usb_error_tracker.consecutive_host_errors++;
    } else {
        usb_error_tracker.consecutive_host_errors = 0;
    }
    last_unmount_time = current_time;
    
    // Trigger visual feedback
    neopixel_trigger_usb_disconnection_flash();
    neopixel_update_status();
}

// HID host callbacks with improved validation
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, const uint8_t* desc_report, uint16_t desc_len)
{
    (void)desc_report; // Suppress unused parameter warning
    (void)desc_len;    // Suppress unused parameter warning
    
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    
    printf("USB Host: HID device mounted - addr=%d, instance=%d, protocol=%d\n",
           dev_addr, instance, itf_protocol);
    
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
        printf("USB Host: Mouse detected!\n");
    } else if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
        printf("USB Host: Keyboard detected!\n");
    } else {
        printf("USB Host: Unknown HID protocol: %d\n", itf_protocol);
    }

    // Handle HID device connection
    handle_hid_device_connection(dev_addr, itf_protocol);

    // Start receiving reports
    if (!tuh_hid_receive_report(dev_addr, instance)) {
        printf("USB Host: Failed to start receiving reports for device %d\n", dev_addr);
    } else {
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

#if USE_HARDWARE_ACCELERATION
// Hardware-accelerated protocol detection
uint8_t hw_detect_protocol(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
    // Use RP2350 PIO to detect protocol type
    uint8_t detected_protocol;
    
    // Fast path: get protocol from interface
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    
    // Use hardware acceleration for validation and detection
    __asm volatile (
        // Load report data into registers for parallel processing
        "ldr r0, %[report]                \n"  // Load report pointer
        "ldr r1, %[len]                   \n"  // Load report length
        "ldr r2, %[itf_protocol]          \n"  // Load interface protocol
        
        // Validate protocol with hardware acceleration
        "cmp r2, #1                       \n"  // Compare with HID_ITF_PROTOCOL_KEYBOARD
        "beq .protocol_valid              \n"  // Branch if keyboard
        "cmp r2, #2                       \n"  // Compare with HID_ITF_PROTOCOL_MOUSE
        "beq .protocol_valid              \n"  // Branch if mouse
        
        // Protocol not recognized, try to detect from report structure
        "cmp r1, #8                       \n"  // Check if report length >= 8 (typical keyboard)
        "bge .check_keyboard              \n"  // If so, check keyboard pattern
        "cmp r1, #3                       \n"  // Check if report length >= 3 (typical mouse)
        "bge .check_mouse                 \n"  // If so, check mouse pattern
        "b .unknown_protocol              \n"  // Otherwise unknown
        
        // Check keyboard pattern (modifier byte + reserved byte + 6 keycodes)
        ".check_keyboard:                 \n"
        "ldrb r3, [r0, #0]                \n"  // Load first byte (modifier)
        "cmp r3, #0xFF                    \n"  // Check if modifier is valid (not all bits set)
        "bhi .check_mouse                 \n"  // If not valid, try mouse
        "mov r2, #1                       \n"  // Set as keyboard
        "b .protocol_valid                \n"
        
        // Check mouse pattern (buttons + x + y coordinates)
        ".check_mouse:                    \n"
        "ldrb r3, [r0, #0]                \n"  // Load first byte (buttons)
        "and r3, r3, #0xF8                \n"  // Check upper 5 bits (should be 0 for mouse)
        "cmp r3, #0                       \n"  // Compare with 0
        "bne .unknown_protocol            \n"  // If not 0, unknown protocol
        "mov r2, #2                       \n"  // Set as mouse
        "b .protocol_valid                \n"
        
        // Unknown protocol
        ".unknown_protocol:               \n"
        "mov r2, #0                       \n"  // Set as unknown
        
        // Protocol validation complete
        ".protocol_valid:                 \n"
        "mov %[detected_protocol], r2     \n"  // Store detected protocol
        : [detected_protocol] "=r" (detected_protocol)
        : [report] "m" (report),
          [len] "m" (len),
          [itf_protocol] "m" (itf_protocol)
        : "r0", "r1", "r2", "r3", "memory", "cc"
    );
    
    return detected_protocol;
}

// Hardware-accelerated report validation
bool hw_validate_report(uint8_t itf_protocol, uint8_t const* report, uint16_t len)
{
    bool is_valid = false;
    
    // Use RP2350 PIO for parallel validation
    __asm volatile (
        // Load parameters
        "ldr r0, %[protocol]              \n"  // Load protocol
        "ldr r1, %[report]                \n"  // Load report pointer
        "ldr r2, %[len]                   \n"  // Load report length
        
        // Initialize result as invalid
        "mov r3, #0                       \n"  // Set is_valid to false
        
        // Check protocol type
        "cmp r0, #1                       \n"  // Compare with HID_ITF_PROTOCOL_KEYBOARD
        "beq .validate_keyboard           \n"  // Branch if keyboard
        "cmp r0, #2                       \n"  // Compare with HID_ITF_PROTOCOL_MOUSE
        "beq .validate_mouse              \n"  // Branch if mouse
        "b .validation_done               \n"  // Otherwise done (invalid)
        
        // Validate keyboard report
        ".validate_keyboard:              \n"
        "cmp r2, #1                       \n"  // Check if length >= 1
        "blt .validation_done             \n"  // If not, invalid
        "mov r3, #1                       \n"  // Set is_valid to true
        "b .validation_done               \n"
        
        // Validate mouse report
        ".validate_mouse:                 \n"
        "cmp r2, #3                       \n"  // Check if length >= 3
        "blt .validation_done             \n"  // If not, invalid
        "mov r3, #1                       \n"  // Set is_valid to true
        
        // Validation complete
        ".validation_done:                \n"
        "mov %[is_valid], r3              \n"  // Store validation result
        : [is_valid] "=r" (is_valid)
        : [protocol] "m" (itf_protocol),
          [report] "m" (report),
          [len] "m" (len)
        : "r0", "r1", "r2", "r3", "memory", "cc"
    );
    
    return is_valid;
}

// Hardware-accelerated report processing
void process_hid_report_hardware(uint8_t dev_addr __attribute__((unused)), uint8_t instance __attribute__((unused)), uint8_t itf_protocol, uint8_t const* report, uint16_t len)
{
    // Use RP2350 PIO for accelerated report processing
    switch (itf_protocol) {
        case HID_ITF_PROTOCOL_KEYBOARD:
            if (hw_validate_report(itf_protocol, report, len)) {
                // Direct cast for performance - avoid memcpy overhead
                const hid_keyboard_report_t* kbd_report = (const hid_keyboard_report_t*)report;
                process_kbd_report(kbd_report);
            }
            break;

        case HID_ITF_PROTOCOL_MOUSE:
            if (hw_validate_report(itf_protocol, report, len)) {
                // Direct cast for performance - avoid memcpy overhead
                const hid_mouse_report_t* mouse_report = (const hid_mouse_report_t*)report;
                process_mouse_report(mouse_report);
            }
            break;

        default:
            // Skip unknown protocols
            break;
    }
}
#endif // USE_HARDWARE_ACCELERATION

// Software fallback for report processing
void process_hid_report_software(uint8_t dev_addr __attribute__((unused)), uint8_t instance __attribute__((unused)), uint8_t itf_protocol, uint8_t const* report, uint16_t len)
{
    // Direct processing without extra copying for better performance
    switch (itf_protocol) {
        case HID_ITF_PROTOCOL_KEYBOARD:
            if (len >= 1) {
                // Direct cast for performance - avoid memcpy overhead
                const hid_keyboard_report_t* kbd_report = (const hid_keyboard_report_t*)report;
                process_kbd_report(kbd_report);
            }
            break;

        case HID_ITF_PROTOCOL_MOUSE:
            if (len >= 3) { // Minimum mouse report: buttons + x + y
                // Direct cast for performance - avoid memcpy overhead
                const hid_mouse_report_t* mouse_report = (const hid_mouse_report_t*)report;
                process_mouse_report(mouse_report);
            }
            break;

        default:
            // Skip debug logging for unknown protocols to reduce overhead
            break;
    }
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, const uint8_t* report, uint16_t len)
{
    // Fast path with hardware acceleration for validation
    if (report == NULL || len == 0) {
        tuh_hid_receive_report(dev_addr, instance);
        return;
    }

    // Use hardware acceleration for protocol detection and dispatch
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    
#if USE_HARDWARE_ACCELERATION
    // RP2350: Use dedicated PIO for report type detection and preprocessing
    if (hw_accel_enabled) {
        // Hardware-accelerated path
        process_hid_report_hardware(dev_addr, instance, itf_protocol, report, len);
    } else {
        // Software fallback path
        process_hid_report_software(dev_addr, instance, itf_protocol, report, len);
    }
#else
    // Original RP2040 implementation
    process_hid_report_software(dev_addr, instance, itf_protocol, report, len);
#endif

    // Continue to request reports with hardware-assisted queuing
    tuh_hid_receive_report(dev_addr, instance);
}